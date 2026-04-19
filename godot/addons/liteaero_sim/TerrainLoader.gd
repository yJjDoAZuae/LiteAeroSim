## TerrainLoader.gd — Programmatic terrain dataset loader.
##
## Design authority: docs/architecture/terrain_build.md §OQ-TB-2 and §OQ-TB-3
##                   docs/architecture/godot_plugin.md §TerrainLoader Integration
##                   docs/architecture/godot_plugin.md §Mesh Appearance Shader
##
## Reads godot/terrain/terrain_config.json at scene start, loads the terrain GLB
## programmatically via ResourceLoader, instantiates it into the scene tree,
## applies per-node LOD visibility ranges, loads the aircraft mesh, and sets
## the world origin on SimulationReceiver — all before the first UDP packet arrives.
##
## Appearance controls (adjustable in the Inspector while the scene runs):
##   terrain_saturation / terrain_brightness / terrain_contrast / terrain_transparency
##   aircraft_saturation / aircraft_brightness / aircraft_contrast / aircraft_transparency
##
## Workflow after a terrain build:
##   1. Run build_terrain (Python).
##   2. Press Play in Godot.
##   No editor drag-and-drop or Inspector edits required.

extends Node

const _CONFIG_PATH := "res://terrain/terrain_config.json"

const _SHADER_CODE := """
shader_type spatial;
render_mode diffuse_lambert, specular_disabled, cull_back;

uniform float saturation   : hint_range(0.0, 2.0) = 1.0;
uniform float brightness   : hint_range(0.0, 2.0) = 1.0;
uniform float contrast     : hint_range(0.0, 2.0) = 1.0;
uniform float transparency : hint_range(0.0, 1.0) = 1.0;
uniform bool  use_vertex_color = true;
uniform vec4  base_color : source_color = vec4(0.290, 0.498, 0.757, 1.0);

void fragment() {
	vec3 col = use_vertex_color ? COLOR.rgb : base_color.rgb;
	col *= brightness;
	col = (col - 0.5) * contrast + 0.5;
	float lum = dot(col, vec3(0.2126, 0.7152, 0.0722));
	col = mix(vec3(lum), col, saturation);
	ALBEDO = clamp(col, 0.0, 1.0);
	ALPHA  = transparency;
}
"""

## LOD visibility range lower bounds (metres) — switch-to-finer hysteresis thresholds.
## Index = LOD level (0–6).  From terrain.md §LOD Hysteresis Band.
const _LOD_VIS_BEGIN_M: Array[float] = [
	0.0,      # L0
	255.0,    # L1
	765.0,    # L2
	2295.0,   # L3
	6885.0,   # L4
	20655.0,  # L5
	61965.0,  # L6
]

## LOD visibility range upper bounds (metres) — switch-to-coarser hysteresis thresholds.
## 0.0 means no upper limit (infinity in Godot's convention for visibility_range_end = 0).
const _LOD_VIS_END_M: Array[float] = [
	345.0,    # L0
	1035.0,   # L1
	3105.0,   # L2
	9315.0,   # L3
	27945.0,  # L4
	83835.0,  # L5
	0.0,      # L6  (infinity)
]

# ---------------------------------------------------------------------------
# Appearance controls — terrain
# ---------------------------------------------------------------------------

@export_group("Terrain Appearance")

## Saturation: 0.0 = greyscale, 1.0 = source color, 2.0 = double saturation.
@export_range(0.0, 2.0) var terrain_saturation: float = 1.0:
	set(v):
		terrain_saturation = v
		if _terrain_material != null:
			_terrain_material.set_shader_parameter("saturation", v)

## Brightness: 0.0 = black, 1.0 = source, 2.0 = double brightness.
@export_range(0.0, 2.0) var terrain_brightness: float = 1.0:
	set(v):
		terrain_brightness = v
		if _terrain_material != null:
			_terrain_material.set_shader_parameter("brightness", v)

## Contrast: 0.0 = flat grey, 1.0 = source, 2.0 = double contrast.
@export_range(0.0, 2.0) var terrain_contrast: float = 1.0:
	set(v):
		terrain_contrast = v
		if _terrain_material != null:
			_terrain_material.set_shader_parameter("contrast", v)

## Transparency: 0.0 = invisible, 1.0 = fully opaque.
@export_range(0.0, 1.0) var terrain_transparency: float = 1.0:
	set(v):
		terrain_transparency = v
		if _terrain_material != null:
			_terrain_material.set_shader_parameter("transparency", v)

# ---------------------------------------------------------------------------
# Appearance controls — aircraft
# ---------------------------------------------------------------------------

@export_group("Aircraft Appearance")

## Saturation: 0.0 = greyscale, 1.0 = source color, 2.0 = double saturation.
@export_range(0.0, 2.0) var aircraft_saturation: float = 1.0:
	set(v):
		aircraft_saturation = v
		_update_aircraft_color()

## Brightness multiplier applied to the base color.  0.0 = black, 1.0 = source.
@export_range(0.0, 2.0) var aircraft_brightness: float = 1.0:
	set(v):
		aircraft_brightness = v
		_update_aircraft_color()

## Contrast: 0.0 = flat grey, 1.0 = source, 2.0 = double contrast.
@export_range(0.0, 2.0) var aircraft_contrast: float = 1.0:
	set(v):
		aircraft_contrast = v
		_update_aircraft_color()

## Transparency: 0.0 = invisible, 1.0 = fully opaque.
@export_range(0.0, 1.0) var aircraft_transparency: float = 1.0:
	set(v):
		aircraft_transparency = v
		_update_aircraft_color()

# ---------------------------------------------------------------------------
# Camera controls
# ---------------------------------------------------------------------------

@export_group("Camera")

## Distance from camera to aircraft origin (metres).
## Default: wingspan (~16 m) projects to 1/3 of horizontal view at FOV 90°.
@export_range(5.0, 2000.0) var camera_distance: float = 24.0:
	set(v):
		camera_distance = v
		_update_camera()

# ---------------------------------------------------------------------------
# Aircraft CG offset controls
# ---------------------------------------------------------------------------

@export_group("Aircraft CG")

## Longitudinal CG position as a fraction of the mesh X extent.
## 0.0 = nose tip, 1.0 = tail tip.  Tune visually against the mesh.
@export_range(0.0, 1.0, 0.01) var aircraft_cg_x_fraction: float = 0.5:
	set(v):
		aircraft_cg_x_fraction = v
		_update_aircraft_cg_offset()

## Vertical CG position as a fraction of the mesh Z extent.
## 0.0 = belly, 1.0 = top of vertical stabilizer.
@export_range(0.0, 1.0, 0.01) var aircraft_cg_z_fraction: float = 0.2:
	set(v):
		aircraft_cg_z_fraction = v
		_update_aircraft_cg_offset()

# ---------------------------------------------------------------------------
# Private state
# ---------------------------------------------------------------------------

var _terrain_material:      ShaderMaterial    = null
var _aircraft_material:     StandardMaterial3D = null
var _camera:                Camera3D           = null
var _aircraft_center_world_y: float            = 0.0
var _aircraft_mesh_node:    Node3D             = null
var _aircraft_cg_prepos:    Vector3            = Vector3.ZERO  # CG in pre-position world space

# ---------------------------------------------------------------------------

func _process(_delta: float) -> void:
	_update_camera()


func _ready() -> void:
	_create_materials()

	var config := _load_config()
	if config.is_empty():
		return

	var terrain_node := _load_terrain_scene(config["glb_path"])
	if terrain_node == null:
		return

	add_child(terrain_node)
	_apply_visibility_ranges(terrain_node)
	if _terrain_material != null:
		call_deferred("_apply_material_deferred", terrain_node)
	else:
		push_error("TerrainLoader: terrain_material is null — shader creation failed")
	_load_aircraft_mesh(config)
	_set_world_origin(config)
	_update_camera()

# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

## Position the scene Camera3D behind and above the Vehicle origin so the
## aircraft wingspan (~16 m) fills 1/3 of the horizontal view at FOV 90°.
## Called once on _ready() and whenever camera_distance changes via the slider.
func _update_camera() -> void:
	if not is_inside_tree():
		return
	if _camera == null:
		_camera = _find_camera(get_tree().root)
	if _camera == null:
		return
	_camera.fov = 90.0
	_camera.keep_aspect = Camera3D.KEEP_WIDTH
	var look_target := Vector3.ZERO
	var vehicle := _find_vehicle(get_tree().root)
	if vehicle != null:
		look_target = vehicle.global_position
	_camera.position = Vector3(look_target.x, look_target.y + camera_distance * 0.15, look_target.z + camera_distance)
	_camera.look_at(look_target, Vector3.UP)


## Recompute mesh_node.position when CG fractions change in the Inspector.
func _update_aircraft_cg_offset() -> void:
	if _aircraft_mesh_node == null:
		return
	# Recompute CG prepos from stored value scaled by new fractions requires
	# re-reading the AABB, which means re-running the full placement logic.
	# Since the mesh is already instantiated, find the MeshInstance3D and recompute.
	var mi := _find_first_mesh_instance(_aircraft_mesh_node)
	if mi == null:
		return
	var aabb := mi.get_aabb()
	var cg_local := Vector3(
		aabb.position.x + aircraft_cg_x_fraction * aabb.size.x,
		aabb.position.y + 0.5 * aabb.size.y,
		aabb.position.z + aircraft_cg_z_fraction * aabb.size.z)
	_aircraft_cg_prepos = _aircraft_mesh_node.basis * (mi.transform * cg_local)
	_aircraft_mesh_node.position = -_aircraft_cg_prepos


## Depth-first search for the first Camera3D in the scene tree.
func _find_camera(node: Node) -> Camera3D:
	if node is Camera3D:
		return node as Camera3D
	for child: Node in node.get_children():
		var result := _find_camera(child)
		if result != null:
			return result
	return null

# ---------------------------------------------------------------------------
# Material creation
# ---------------------------------------------------------------------------

func _create_materials() -> void:
	var shader := Shader.new()
	shader.code = _SHADER_CODE

	_terrain_material = ShaderMaterial.new()
	_terrain_material.shader = shader
	_terrain_material.set_shader_parameter("use_vertex_color", false)
	_terrain_material.set_shader_parameter("saturation",   terrain_saturation)
	_terrain_material.set_shader_parameter("brightness",   terrain_brightness)
	_terrain_material.set_shader_parameter("contrast",     terrain_contrast)
	_terrain_material.set_shader_parameter("transparency", terrain_transparency)

	# Aircraft uses StandardMaterial3D for proper PBR lighting and shadow
	# casting/reception.  Color controls are applied via _update_aircraft_color().
	_aircraft_material = StandardMaterial3D.new()
	_update_aircraft_color()


## Recompute the aircraft StandardMaterial3D albedo color from the base color
## (#4A7FC1 per PP-F25) modulated by brightness, contrast, and saturation.
## Transparency is applied via the material's alpha channel.
func _update_aircraft_color() -> void:
	if _aircraft_material == null:
		return
	var base := Color(0.290, 0.498, 0.757)  # #4A7FC1 per PP-F25
	var col := base
	col = Color(col.r * aircraft_brightness,
				col.g * aircraft_brightness,
				col.b * aircraft_brightness)
	col = Color((col.r - 0.5) * aircraft_contrast + 0.5,
				(col.g - 0.5) * aircraft_contrast + 0.5,
				(col.b - 0.5) * aircraft_contrast + 0.5)
	var lum: float = col.r * 0.2126 + col.g * 0.7152 + col.b * 0.0722
	col = Color(lerpf(lum, col.r, aircraft_saturation),
				lerpf(lum, col.g, aircraft_saturation),
				lerpf(lum, col.b, aircraft_saturation))
	col = col.clamp()
	col.a = aircraft_transparency
	_aircraft_material.albedo_color = col
	_aircraft_material.flags_transparent = aircraft_transparency < 1.0


func _apply_material_deferred(terrain_node: Node) -> void:
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.3, 0.6, 0.2)
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	_apply_material_to_tree(terrain_node, mat, true)

## Recursively apply mat to every MeshInstance3D under node.
## is_terrain controls shadow casting: terrain receives but does not cast shadows.
func _apply_material_to_tree(node: Node, mat: Material, is_terrain: bool = false) -> void:
	if node is MeshInstance3D:
		var mi := node as MeshInstance3D
		mi.material_override = mat
		mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF if is_terrain \
			else GeometryInstance3D.SHADOW_CASTING_SETTING_ON
	for child: Node in node.get_children():
		_apply_material_to_tree(child, mat, is_terrain)

# ---------------------------------------------------------------------------

func _load_config() -> Dictionary:
	if not FileAccess.file_exists(_CONFIG_PATH):
		push_error(
			"TerrainLoader: terrain_config.json not found at %s — run build_terrain first"
			% _CONFIG_PATH
		)
		return {}

	var f := FileAccess.open(_CONFIG_PATH, FileAccess.READ)
	if f == null:
		push_error("TerrainLoader: cannot open %s" % _CONFIG_PATH)
		return {}

	var parsed: Variant = JSON.parse_string(f.get_as_text())
	if parsed == null or not (parsed is Dictionary):
		push_error("TerrainLoader: terrain_config.json is not a valid JSON object")
		return {}

	return parsed as Dictionary


func _load_terrain_scene(glb_path: String) -> Node3D:
	var packed: Resource = ResourceLoader.load(
		glb_path, "", ResourceLoader.CACHE_MODE_IGNORE
	)
	if packed == null:
		push_error("TerrainLoader: failed to load GLB from %s" % glb_path)
		return null

	if not (packed is PackedScene):
		push_error("TerrainLoader: resource at %s is not a PackedScene" % glb_path)
		return null

	return (packed as PackedScene).instantiate() as Node3D


## Load the aircraft mesh from aircraft_mesh_path in config and attach it to
## the Vehicle node as a child.  Applies body-frame correction (nose=+X ->
## Godot -Z forward) per OQ-LS-9 Option B, then scales to real wingspan.
func _load_aircraft_mesh(config: Dictionary) -> void:
	var mesh_path: String = config.get("aircraft_mesh_path", "")
	if mesh_path.is_empty():
		push_warning("TerrainLoader: no aircraft_mesh_path in terrain_config.json — skipping mesh load")
		return

	var packed: Resource = ResourceLoader.load(mesh_path, "", ResourceLoader.CACHE_MODE_IGNORE)
	if packed == null or not (packed is PackedScene):
		push_error("TerrainLoader: failed to load aircraft mesh from %s" % mesh_path)
		return

	var mesh_node: Node3D = (packed as PackedScene).instantiate() as Node3D
	mesh_node.name = "AircraftMesh"

	# Layout-frame → body-frame correction.
	# Aircraft mesh source files use the layout frame:
	#   +X = aft (increasing fuselage station), +Y = right buttline, +Z = up waterline.
	# SimulationReceiver applies r_ned_to_godot * q_b2n where q_b2n is the
	# body-to-NED quaternion.  r_ned_to_godot expects body frame (+X=fwd, +Y=right,
	# +Z=down).  Ry(180°) converts layout → body: +X→-X(fwd), +Y→+Y, +Z→-Z(down).
	# Result: nose (-X layout face) points Godot -Z (into scene), right (+Y) → +X (east),
	# up (+Z layout) → +Y (world up).  Chase camera at +Z sees the aft face.
	mesh_node.rotation_degrees = Vector3(0, 180, 0)

	var mi := _find_first_mesh_instance(mesh_node)
	if mi != null:
		var aabb := mi.get_aabb()

		# Scale mesh so its wingspan matches the real aircraft wingspan.
		# The wingspan spans local Y (right buttline); aabb.size.y is the full span.
		var mesh_wingspan_m: float = aabb.size.y
		if config.has("aircraft_wingspan_m") and mesh_wingspan_m > 0.001:
			var target_wingspan_m: float = float(config["aircraft_wingspan_m"])
			mesh_node.scale = Vector3.ONE * (target_wingspan_m / mesh_wingspan_m)

		if config.has("aircraft_cg_x_fraction"):
			aircraft_cg_x_fraction = float(config["aircraft_cg_x_fraction"])
		if config.has("aircraft_cg_z_fraction"):
			aircraft_cg_z_fraction = float(config["aircraft_cg_z_fraction"])

		# CG in mesh local space; transform through mi and mesh_node basis to world.
		var cg_local := Vector3(
			aabb.position.x + aircraft_cg_x_fraction * aabb.size.x,
			aabb.position.y + 0.5 * aabb.size.y,
			aabb.position.z + aircraft_cg_z_fraction * aabb.size.z)
		_aircraft_cg_prepos = mesh_node.basis * (mi.transform * cg_local)

		# Place mesh so its body CG coincides with the Vehicle node origin.
		# The physics simulation positions Vehicle at the aircraft CG; no
		# terrain-relative offset is applied here.
		_aircraft_center_world_y = 0.0  # CG is at Vehicle origin by definition
		mesh_node.position = -_aircraft_cg_prepos
		_aircraft_mesh_node = mesh_node

	_apply_material_to_tree(mesh_node, _aircraft_material)

	var vehicle := _find_vehicle(get_tree().root)
	if vehicle == null:
		push_error("TerrainLoader: Vehicle node not found — cannot attach aircraft mesh")
		return

	vehicle.add_child(mesh_node)


## Recursively walk node and set visibility_range_* on every MeshInstance3D
## whose name matches the tile_L{N}_* convention written by export_gltf.py.
##
## Godot culls by distance from camera to mesh node origin (centroid).  For large
## tiles the centroid may be far from the camera even when the camera is inside
## the tile bbox.  We compensate by adding the tile's AABB half-diagonal to both
## begin and end thresholds, converting centroid-distance culling to approximate
## nearest-edge-distance culling.
func _apply_visibility_ranges(node: Node) -> void:
	if node is MeshInstance3D:
		var lod := _parse_lod_from_name(node.name)
		if lod >= 0:
			var mi := node as MeshInstance3D
			var radius := mi.get_aabb().size.length() * 0.5
			mi.visibility_range_begin = max(0.0, _LOD_VIS_BEGIN_M[lod] - radius)
			mi.visibility_range_end   = (_LOD_VIS_END_M[lod] + radius) if _LOD_VIS_END_M[lod] > 0.0 else 0.0

	for child: Node in node.get_children():
		_apply_visibility_ranges(child)


## Parse LOD level from a node name of the form "tile_L{N}_...".
## Returns the integer LOD (0–6) or -1 if the name does not match.
func _parse_lod_from_name(node_name: String) -> int:
	if not node_name.begins_with("tile_L") or node_name == "tile_L0_flat":
		return -1
	var rest := node_name.substr(6)  # characters after "tile_L"
	var underscore := rest.find("_")
	var lod_str: String
	if underscore < 0:
		lod_str = rest
	else:
		lod_str = rest.substr(0, underscore)
	if not lod_str.is_valid_int():
		return -1
	var lod := lod_str.to_int()
	if lod < 0 or lod > 6:
		return -1
	return lod


## Depth-first search for the first MeshInstance3D under node.
func _find_first_mesh_instance(node: Node) -> MeshInstance3D:
	if node is MeshInstance3D:
		return node as MeshInstance3D
	for child: Node in node.get_children():
		var result := _find_first_mesh_instance(child)
		if result != null:
			return result
	return null


## Depth-first search for the Vehicle node by name.
func _find_vehicle(node: Node) -> Node3D:
	if node.name == "Vehicle" and node is Node3D:
		return node as Node3D
	for child: Node in node.get_children():
		var result := _find_vehicle(child)
		if result != null:
			return result
	return null


## Set world origin on the SimulationReceiver node found in the scene tree.
## Supports both the native GDExtension type and the GDScript placeholder.
func _set_world_origin(config: Dictionary) -> void:
	var receiver := _find_simulation_receiver(get_tree().root)
	if receiver == null:
		push_warning("TerrainLoader: SimulationReceiver not found in scene tree")
		return

	var lat_rad := float(config.get("world_origin_lat_rad", 0.0))
	var lon_rad := float(config.get("world_origin_lon_rad", 0.0))
	var h_m     := float(config.get("world_origin_height_m", 0.0))

	if receiver.get_class() == "SimulationReceiver":
		# GDExtension native type — use the bound method.
		receiver.set_world_origin(lat_rad, lon_rad, h_m)
	else:
		# GDScript placeholder — direct property assignment.
		receiver.world_origin_lat_rad = lat_rad
		receiver.world_origin_lon_rad = lon_rad
		receiver.world_origin_h_m     = h_m
		receiver.world_origin_set     = true


## Depth-first search for the SimulationReceiver node.
## Detects both the native GDExtension type (by class name) and the GDScript
## placeholder (by script resource path) so TerrainLoader works in either state.
func _find_simulation_receiver(node: Node) -> Node:
	# Native GDExtension type — get_class() returns the registered C++ class name.
	if node.get_class() == "SimulationReceiver":
		return node
	# GDScript placeholder — check script path.
	var script: Script = node.get_script() as Script
	if script != null and script.resource_path.ends_with("SimulationReceiver.gd"):
		return node
	for child: Node in node.get_children():
		var result := _find_simulation_receiver(child)
		if result != null:
			return result
	return null
