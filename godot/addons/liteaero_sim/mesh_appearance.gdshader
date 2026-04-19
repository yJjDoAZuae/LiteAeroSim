// mesh_appearance.gdshader — Appearance shader for terrain tiles and aircraft mesh.
//
// Design authority: docs/architecture/godot_plugin.md §Mesh Appearance Shader
//
// Shared by terrain and aircraft ShaderMaterial instances created by TerrainLoader.
// Terrain uses vertex colors (use_vertex_color = true).
// Aircraft uses a flat base color (use_vertex_color = false, base_color = #4A7FC1).
//
// Controls (all adjustable at runtime via ShaderMaterial uniforms):
//   saturation   — 0.0 = greyscale, 1.0 = source color, 2.0 = double saturation
//   brightness   — 0.0 = black,     1.0 = source,       2.0 = double brightness
//   contrast     — 0.0 = flat grey, 1.0 = source,       2.0 = double contrast
//   transparency — 0.0 = invisible, 1.0 = fully opaque
//
shader_type spatial;
render_mode blend_mix, depth_draw_always, cull_back, diffuse_lambert, specular_disabled;

uniform float saturation   : hint_range(0.0, 2.0) = 1.0;
uniform float brightness   : hint_range(0.0, 2.0) = 1.0;
uniform float contrast     : hint_range(0.0, 2.0) = 1.0;
uniform float transparency : hint_range(0.0, 1.0) = 1.0;

// When true, reads per-vertex color from the mesh (terrain).
// When false, uses base_color (aircraft).
uniform bool use_vertex_color = true;

// Flat base color used when use_vertex_color is false.
// Default: medium blue #4A7FC1 per PP-F25.
uniform vec4 base_color : source_color = vec4(0.290, 0.498, 0.757, 1.0);

void fragment() {
	vec3 col = use_vertex_color ? COLOR.rgb : base_color.rgb;

	// Brightness: multiplicative scale around zero.
	col *= brightness;

	// Contrast: scale around mid-grey (0.5).
	col = (col - 0.5) * contrast + 0.5;

	// Saturation: interpolate toward perceptual luminance.
	float lum = dot(col, vec3(0.2126, 0.7152, 0.0722));
	col = mix(vec3(lum), col, saturation);

	ALBEDO = clamp(col, 0.0, 1.0);
	ALPHA  = transparency;
}
