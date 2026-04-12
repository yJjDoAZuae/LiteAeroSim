# Godot Plugin — GDExtension Design Specification

Design authority for the Godot 4 plugin that receives `SimulationFrameProto` UDP
datagrams and drives the Godot scene. Closely related to
[`live_sim_view.md`](live_sim_view.md), which specifies the C++ simulation
broadcast path and overall live-sim architecture.

---

## Current State

A **GDScript placeholder** exists at `godot/addons/liteaero_sim/`:

| File | Status |
| --- | --- |
| `SimulationReceiver.gd` | Functional GDScript placeholder — hand-rolled proto3 wire-format parser |
| `TerrainLoader.gd` | Implemented in GDScript — no C++ needed (pure Godot API calls) |

`SimulationReceiver.gd` is self-documented as a placeholder. It works for
development but the hand-rolled binary parser is fragile: any addition to
`SimulationFrameProto` (e.g., new fields, nested messages) requires manual
parser updates and is undetected at compile time.

The GDExtension C++ implementation replaces `SimulationReceiver.gd` with a
class compiled against the protobuf C++ runtime, eliminating the fragile
binary parser. `TerrainLoader.gd` is pure Godot API and remains GDScript (see
[OQ-GP-1](#oq-gp-1--terrain-loader-language)).

---

## Motivation for GDExtension

| Concern | GDScript placeholder | GDExtension C++ |
| --- | --- | --- |
| Protobuf parsing | Hand-rolled wire-format decoder | `SimulationFrameProto::ParseFromArray()` — standard, type-safe |
| Schema evolution | Parser breaks silently on new fields | Unknown fields ignored by protobuf runtime; new required fields caught at compile time |
| Performance | GDScript overhead at 400 Hz datagram rate | Near-zero overhead; all parsing in native C++ |
| Type safety | `Dictionary` with untyped `float` values | Typed proto message struct |
| Maintainability | Parser must be updated in sync with `liteaerosim.proto` | `#include "liteaerosim.pb.h"` — automatically in sync |

---

## File Structure

```text
godot/
├── addons/
│   └── liteaero_sim/
│       ├── liteaero_sim.gdextension       ← plugin manifest (Godot reads this)
│       ├── SimulationReceiver.gd           ← replaced by C++ once GDExtension is built
│       ├── TerrainLoader.gd               ← remains GDScript (no C++ needed)
│       └── src/
│           ├── CMakeLists.txt             ← GDExtension shared library build
│           ├── register_types.hpp
│           ├── register_types.cpp         ← GDExtensionBool entry point + ClassDB registration
│           ├── SimulationReceiver.hpp
│           └── SimulationReceiver.cpp
└── scenes/
    └── World.tscn
```

Once the GDExtension is built and the shared library placed at the path declared
in `liteaero_sim.gdextension`, Godot loads it at startup and makes
`SimulationReceiver` available as a native node type. `SimulationReceiver.gd` is
then no longer attached to the scene node (the C++ type is used directly).

---

## Dependencies

| Dependency | Version | License | Integration method |
| --- | --- | --- | --- |
| [godot-cpp](https://github.com/godotengine/godot-cpp) | Must match Godot editor (4.6) | MIT | FetchContent (CMake) |
| protobuf | 3.21.12 | BSD-3-Clause | Conan — already in `conanfile.txt` |

`godot-cpp` provides the C++ bindings for the GDExtension API. It must be built
for the same Godot version as the editor being used. The version is determined by
the `config/features` line in `godot/project.godot` (currently `"4.6"`).

`godot-cpp` is fetched via CMake `FetchContent` using a tag that matches the
Godot editor version. It is not available in ConanCenter.

---

## Build System

### `godot/addons/liteaero_sim/src/CMakeLists.txt`

The build file is implemented at
[`godot/addons/liteaero_sim/src/CMakeLists.txt`](../../godot/addons/liteaero_sim/src/CMakeLists.txt).
It is the authoritative source; the sections below describe its structure.

**Enabled from the root** via:

```cmake
# Root CMakeLists.txt
option(LITEAERO_SIM_BUILD_GODOT_PLUGIN
    "Build the GDExtension C++ plugin for Godot 4" OFF)

if(LITEAERO_SIM_BUILD_GODOT_PLUGIN)
    add_subdirectory(godot/addons/liteaero_sim/src)
endif()
```

**Version parsing** — derives the `godot-cpp` git tag from
`godot/project.godot` automatically (see [OQ-GP-2](#oq-gp-2--godot-cpp-version-tracking)
for the full failure-mode table).

**Library target** — `liteaero_sim_gdext` shared library, linking `godot-cpp`
and `protobuf::libprotobuf`, with static winpthread on MinGW, output to
`godot/addons/liteaero_sim/bin/` (excluded from version control).

**GP-1 gate** — if the C++ source files (`register_types.cpp`,
`SimulationReceiver.cpp`) are not present, CMake halts with a clear message
pointing to the GP-1 roadmap item before any library target is defined.

---

## `.gdextension` Manifest

**File:** `godot/addons/liteaero_sim/liteaero_sim.gdextension`

```ini
[configuration]
entry_symbol = "liteaero_sim_init"
compatibility_minimum = "4.1"

[libraries]
windows.debug.x86_64   = "res://addons/liteaero_sim/bin/liteaero_sim_gdext.dll"
windows.release.x86_64 = "res://addons/liteaero_sim/bin/liteaero_sim_gdext.dll"
```

The manifest path is relative to the Godot project root (`godot/`), so
`res://addons/liteaero_sim/bin/` maps to
`godot/addons/liteaero_sim/bin/` on disk.

---

## `register_types` Entry Point

**Files:** `register_types.hpp`, `register_types.cpp`

```cpp
// register_types.hpp
#pragma once
#include <godot_cpp/core/class_db.hpp>

void initialize_liteaero_sim(godot::ModuleInitializationLevel p_level);
void uninitialize_liteaero_sim(godot::ModuleInitializationLevel p_level);
```

```cpp
// register_types.cpp
#include "register_types.hpp"
#include "SimulationReceiver.hpp"
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_liteaero_sim(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE)
        return;
    ClassDB::register_class<SimulationReceiver>();
}

void uninitialize_liteaero_sim(ModuleInitializationLevel p_level) {
    (void)p_level;
}

extern "C" GDExtensionBool GDE_EXPORT liteaero_sim_init(
    GDExtensionInterfaceGetProcAddress p_get_proc_address,
    const GDExtensionClassLibraryPtr p_library,
    GDExtensionInitialization *r_initialization)
{
    GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
    init_obj.register_initializer(initialize_liteaero_sim);
    init_obj.register_terminator(uninitialize_liteaero_sim);
    init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
    return init_obj.init();
}
```

---

## `SimulationReceiver` Class

### Header — `SimulationReceiver.hpp`

```cpp
#pragma once
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <cstdint>

namespace godot {

class SimulationReceiver : public Node3D {
    GDCLASS(SimulationReceiver, Node3D)

public:
    SimulationReceiver();
    ~SimulationReceiver() override;

    void _ready() override;
    void _process(double delta) override;

    // Exported properties — visible in the Godot Inspector.
    void set_broadcast_port(int port);
    int  get_broadcast_port() const;

    void set_max_datagrams_per_frame(int n);
    int  get_max_datagrams_per_frame() const;

    // Called by TerrainLoader._ready() before any UDP packet arrives.
    void set_world_origin(double lat_rad, double lon_rad, double h_m);

protected:
    static void _bind_methods();

private:
    void _open_socket();
    void _close_socket();
    void _apply_frame(const uint8_t* data, int size);

    int    broadcast_port_          = 14560;
    int    max_datagrams_per_frame_ = 10;

    double world_origin_lat_rad_ = 0.0;
    double world_origin_lon_rad_ = 0.0;
    double world_origin_h_m_     = 0.0;
    bool   world_origin_set_     = false;

    int socket_fd_ = -1;  // POSIX / WinSock UDP socket, non-blocking
};

} // namespace godot
```

### `_bind_methods()` — Inspector properties and scripting API

```cpp
void SimulationReceiver::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_broadcast_port", "port"),
                         &SimulationReceiver::set_broadcast_port);
    ClassDB::bind_method(D_METHOD("get_broadcast_port"),
                         &SimulationReceiver::get_broadcast_port);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "broadcast_port"),
                 "set_broadcast_port", "get_broadcast_port");

    ClassDB::bind_method(D_METHOD("set_max_datagrams_per_frame", "n"),
                         &SimulationReceiver::set_max_datagrams_per_frame);
    ClassDB::bind_method(D_METHOD("get_max_datagrams_per_frame"),
                         &SimulationReceiver::get_max_datagrams_per_frame);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "max_datagrams_per_frame"),
                 "set_max_datagrams_per_frame", "get_max_datagrams_per_frame");

    ClassDB::bind_method(D_METHOD("set_world_origin", "lat_rad", "lon_rad", "h_m"),
                         &SimulationReceiver::set_world_origin);
}
```

### `_ready()` — Socket initialization

Open a non-blocking UDP socket and bind to `127.0.0.1:<broadcast_port>`.

```cpp
void SimulationReceiver::_ready() {
    _open_socket();
}
```

Socket implementation uses `socket()` / `bind()` / `ioctlsocket()` (Windows) or
`fcntl()` (POSIX) to set `O_NONBLOCK`. Must handle `WSAEWOULDBLOCK` /
`EAGAIN` gracefully in `recvfrom()`.

### `_process()` — Datagram poll and transform update

```cpp
void SimulationReceiver::_process(double /*delta*/) {
    if (socket_fd_ < 0) return;

    uint8_t buf[512];
    int frames = 0;
    while (frames < max_datagrams_per_frame_) {
        int n = recvfrom(socket_fd_, buf, sizeof(buf), 0, nullptr, nullptr);
        if (n <= 0) break;
        _apply_frame(buf, n);
        ++frames;
    }
}
```

### `_apply_frame()` — Protobuf deserialization and transform application

```cpp
void SimulationReceiver::_apply_frame(const uint8_t* data, int size) {
    liteaero::simulation::SimulationFrameProto frame;
    if (!frame.ParseFromArray(data, size)) return;

    if (!world_origin_set_) {
        ERR_PRINT("SimulationReceiver: world origin not set");
        return;
    }

    constexpr double R_EARTH_M = 6371000.0;
    double dlat    = frame.latitude_rad()  - world_origin_lat_rad_;
    double dlon    = frame.longitude_rad() - world_origin_lon_rad_;
    double north_m = dlat * R_EARTH_M;
    double east_m  = dlon * R_EARTH_M * std::cos(world_origin_lat_rad_);
    double up_m    = frame.height_wgs84_m() - world_origin_h_m_;

    // ENU -> Godot: X=East, Y=Up, Z=-North
    get_parent()->set("position", Vector3(
        static_cast<float>(east_m),
        static_cast<float>(up_m),
        static_cast<float>(-north_m)));

    // Body-to-NED quaternion -> body-to-Godot quaternion.
    // NED->Godot rotation matrix (from live_sim_view.md §Coordinate System):
    //   [ 0  1  0 ]
    //   [ 0  0 -1 ]
    //   [-1  0  0 ]
    // Quaternion representation: w=0.5, x=0.5, y=0.5, z=-0.5
    Quaternion r_ned_to_godot(0.5f, 0.5f, 0.5f, -0.5f);
    Quaternion q_b2n(frame.q_w(), frame.q_x(), frame.q_y(), frame.q_z());
    q_b2n = q_b2n.normalized();
    get_parent()->set("quaternion", (r_ned_to_godot * q_b2n).normalized());
}
```

---

## TerrainLoader Integration

`TerrainLoader.gd` performs three tasks in `_ready()`: load terrain, load the
aircraft mesh, and set the world origin on `SimulationReceiver`. All three are
driven by `terrain_config.json`.

### Aircraft mesh loading

The aircraft mesh path is stored in `terrain_config.json` as `aircraft_mesh_path`
(a `res://` path). `TerrainLoader.gd` reads this field and instantiates the mesh
as a child of the `Vehicle` node at runtime. No static `AircraftMesh` node exists
in `World.tscn` — the mesh is fully programmatic.

The mesh body-frame correction (OQ-LS-9, Option B: nose=+X → Godot −Z forward)
is applied by `TerrainLoader` when instantiating the node:

```gdscript
func _load_aircraft_mesh(config: Dictionary) -> void:
    var mesh_path: String = config.get("aircraft_mesh_path", "")
    if mesh_path.is_empty():
        push_warning("TerrainLoader: no aircraft_mesh_path in terrain_config.json")
        return
    var packed: Resource = ResourceLoader.load(mesh_path, "", ResourceLoader.CACHE_MODE_IGNORE)
    if packed == null or not (packed is PackedScene):
        push_error("TerrainLoader: failed to load aircraft mesh from %s" % mesh_path)
        return
    var mesh_node: Node3D = (packed as PackedScene).instantiate() as Node3D
    mesh_node.name = "AircraftMesh"
    # Body-frame correction: nose=+X -> Godot -Z (forward). See OQ-LS-9.
    mesh_node.rotation_degrees = Vector3(0, 90, 0)
    var vehicle := _find_vehicle(get_tree().root)
    if vehicle == null:
        push_error("TerrainLoader: Vehicle node not found — cannot attach aircraft mesh")
        return
    vehicle.add_child(mesh_node)
```

`aircraft_mesh_path` is written into `terrain_config.json` by `build_terrain.py`
from the aircraft config's `visualization.mesh_res_path` field. See
[§Aircraft Config Visualization Section](#aircraft-config-visualization-section)
and [`terrain_build.md §Integration with Live Simulation`](terrain_build.md#integration-with-live-simulation).

If `aircraft_mesh_path` is absent from `terrain_config.json` (e.g., generated by
an older build), `TerrainLoader` warns and continues — the simulation runs without
a visible aircraft mesh.

### Aircraft config visualization section

The `visualization` section of the aircraft config JSON specifies the Godot mesh
asset to use for this aircraft category:

```json
"visualization": {
    "mesh_res_path": "res://assets/aircraft_lp.glb"
}
```

| Field | Type | Description |
| --- | --- | --- |
| `mesh_res_path` | string | Godot `res://` path to the aircraft mesh GLB. Must be a file present in the Godot project (i.e., under `godot/`). |

`build_terrain.py` reads `visualization.mesh_res_path` and writes it to
`terrain_config.json` as `aircraft_mesh_path`. If the field is absent from the
aircraft config, `build_terrain.py` uses the default `"res://assets/aircraft_lp.glb"`.

To swap the aircraft mesh for a test case without rebuilding terrain, edit
`godot/terrain/terrain_config.json` directly and change `aircraft_mesh_path`.
This is the only field in `terrain_config.json` that is safe to edit by hand —
the world origin and GLB path must not be changed without a terrain rebuild.

### SimulationReceiver discovery (GDScript placeholder vs. GDExtension)

`TerrainLoader.gd` locates the `SimulationReceiver` node to call
`set_world_origin()`. The discovery method differs by plugin state:

| Plugin state | Node class | Discovery method |
| --- | --- | --- |
| GDScript placeholder | Script type `SimulationReceiver.gd` | `node.get_script().resource_path.ends_with("SimulationReceiver.gd")` |
| GDExtension active | Native C++ type `SimulationReceiver` | `node.get_class() == "SimulationReceiver"` |

`TerrainLoader.gd` must check both so it works in either state:

```gdscript
func _find_simulation_receiver(node: Node) -> Node:
    if node.get_class() == "SimulationReceiver":
        return node
    var script: Script = node.get_script() as Script
    if script != null and script.resource_path.ends_with("SimulationReceiver.gd"):
        return node
    for child in node.get_children():
        var result := _find_simulation_receiver(child)
        if result != null:
            return result
    return null
```

Once the GDExtension is in use, the `set_world_origin()` bound method replaces
direct property assignment:

```gdscript
# GDExtension path (single method call):
receiver.set_world_origin(lat_rad, lon_rad, h_m)

# GDScript placeholder path (direct property assignment — existing code):
receiver.world_origin_lat_rad = lat_rad
receiver.world_origin_lon_rad = lon_rad
receiver.world_origin_h_m     = h_m
receiver.world_origin_set     = true
```

`TerrainLoader.gd` detects which path to use by checking
`receiver.get_class() == "SimulationReceiver"` (native) vs. checking for the
script property.

---

## WinSock Initialization (Windows)

On Windows, `WSAStartup()` must be called once before any socket operations.
The correct location is the GDExtension initialization function:

```cpp
void initialize_liteaero_sim(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) return;
#ifdef _WIN32
    WSADATA wsa_data;
    WSAStartup(MAKEWORD(2, 2), &wsa_data);
#endif
    ClassDB::register_class<SimulationReceiver>();
}

void uninitialize_liteaero_sim(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) return;
#ifdef _WIN32
    WSACleanup();
#endif
}
```

---

## Coordinate System

All coordinate transformations are identical to those in the GDScript
placeholder and specified in
[`live_sim_view.md §Coordinate System`](live_sim_view.md#coordinate-system).

**Geodetic → ENU offset from world origin:**

$$\text{north}_m = (\varphi - \varphi_0) \cdot R_\oplus$$
$$\text{east}_m  = (\lambda - \lambda_0) \cdot R_\oplus \cdot \cos(\varphi_0)$$
$$\text{up}_m    = h - h_0$$

**ENU → Godot position:** $\text{pos} = (\text{east}, \text{up}, -\text{north})$

**NED → Godot frame rotation quaternion:** $q_r = (w=0.5,\ x=0.5,\ y=0.5,\ z=-0.5)$

**Vehicle quaternion:** $q_\text{Godot} = q_r \otimes q_{b2n}$ (normalized)

The aircraft mesh correction rotation (OQ-LS-9, Option B) is a fixed
`rotation_degrees` on the `AircraftMesh` node in the scene — not applied by
`SimulationReceiver`.

---

## Open Questions

### OQ-GP-1 — Terrain loader language

**Resolved — Option A (keep GDScript) for now, with acknowledged maintainability concern.**

`TerrainLoader` remains GDScript. The operations it performs (JSON read,
`ResourceLoader`, scene-tree traversal) are pure Godot API; there is no
correctness or performance argument for C++.

**Acknowledged concern:** A mixed GDScript/C++ addon creates a cross-language
API boundary between `TerrainLoader` and `SimulationReceiver`. Every change to
`set_world_origin()` (signature, semantics) must be coordinated across two
languages. This is the same class of maintainability risk that motivates the
GDExtension in the first place. If the plugin grows beyond the current two
components, this concern should be re-evaluated and the question re-opened as
a prerequisite to any expansion.

**Current status:** Option A is a pragmatic choice for the initial implementation.
It is not a permanent architectural commitment.

---

### OQ-GP-2 — godot-cpp version tracking

**Resolved — Option B (auto-parse `project.godot`).**

The `godot-cpp` GIT_TAG must match the Godot editor version exactly.
A mismatch causes Godot to print "GDExtension API mismatch" at startup and
disable the plugin entirely — a failure that is confusing without knowing to
look for it.

#### Options

##### Option A — Manual GIT_TAG in CMakeLists.txt

The developer edits the `GIT_TAG` line in `CMakeLists.txt` when upgrading the
Godot editor.

- Pro: No tooling. Simple to understand.
- Con: Relies entirely on developer discipline. No signal from the build system
  that the tag is stale. The mismatch symptom is a Godot runtime error, not a
  build error — meaning the mistake is not caught until the scene is opened.
- Con: Two places to update when the editor version changes: `project.godot`
  (done by the Godot editor automatically) and `CMakeLists.txt` (done manually).

##### Option B — Parse `project.godot` in CMake to derive the tag (recommended)

`godot/project.godot` already contains the authoritative version in the line:

```ini
config/features=PackedStringArray("4.6", "Forward Plus")
```

CMake reads this file, extracts the version string, and constructs the
`GIT_TAG` automatically:

The implemented CMake code handles five failure modes with informative
`FATAL_ERROR` messages:

| # | Failure mode | Detection | Message guidance |
| --- | --- | --- | --- |
| 1 | `project.godot` not found | `if(NOT EXISTS ...)` | Path to fix in CMakeLists |
| 2 | `config/features=` line missing | `file(STRINGS ... REGEX)` empty | Re-open project in Godot editor |
| 3 | No version token in features line | regex match fails | Open in stable Godot 4 |
| 4 | Non-stable version (rc/beta/dev) | digits-and-dots check | Switch to stable release |
| 5 | FetchContent fetch fails | pre-flight STATUS messages | Tag URL + network guidance |

For failure mode 5, CMake cannot validate that the derived tag exists before
attempting the fetch. Three STATUS messages are printed immediately before
`FetchContent_MakeAvailable` so that a git error ("revision not found",
"could not resolve host") is accompanied by the derived tag name, a link to
the godot-cpp tags page, and a note about the network-access requirement.

- Pro: Single source of truth. Upgrading the editor (which rewrites
  `project.godot`) automatically causes the next CMake configure to fetch
  the matching `godot-cpp` version. No manual step.
- Pro: Mismatch is structurally impossible — the version in `project.godot` and
  the version used to build the plugin are the same by construction.
- Con: Assumes the `godot-N.M-stable` tag naming convention holds. This is true
  for all stable releases. Release candidates (e.g., `4.6-rc1`) are rejected
  at failure mode 4; the project tracks stable releases only.
- Patch releases (e.g., `4.6.1`) produce tag `godot-4.6.1-stable`, which is a
  valid godot-cpp tag and works correctly.

##### Option C — Explicit version file (`godot/godot_version.cmake`)

A separate `set(GODOT_EDITOR_VERSION "4.6")` file is included by CMakeLists.
Both files must be updated when the editor changes.

- Same manual discipline requirement as Option A, but with a cleaner separation.
- Still two places to update. No improvement over Option A in practice.

#### Recommendation

**Option B.** The version is already authoritatively declared in
`project.godot` by the Godot editor itself. Parsing it in CMake is five lines
of regex and eliminates the only manual synchronization step. The `*-rc*`
exception is not a concern for a project tracking stable releases.

---

### OQ-GP-3 — GDExtension build integration with main CMakeLists

**Resolved — Option B (`add_subdirectory` from root CMakeLists).**

The root `CMakeLists.txt` gains an option and a conditional `add_subdirectory`:

```cmake
option(LITEAERO_SIM_BUILD_GODOT_PLUGIN
    "Build the GDExtension C++ plugin for Godot 4" OFF)

if(LITEAERO_SIM_BUILD_GODOT_PLUGIN)
    add_subdirectory(godot/addons/liteaero_sim/src)
endif()
```

Defaulting to `OFF` because `godot-cpp` pulls in a large FetchContent dependency
that is irrelevant to C++ unit tests and Python-only workflows. Developers doing
Godot work set `-DLITEAERO_SIM_BUILD_GODOT_PLUGIN=ON` explicitly, consistent with
how `-DLITEAERO_SIM_BUILD_PYTHON_BINDINGS` was structured before it was made the
default.

The `godot/addons/liteaero_sim/bin/` output directory (where the `.dll` lands) is
added to `.gitignore`. `docs/installation/README.md` documents the configure flag.

---

## Decision Records

### GP-DR-1 — GDExtension over GDScript for `SimulationReceiver`

**Decision:** `SimulationReceiver` is implemented as a GDExtension C++ class.

**Rationale:** The primary function of `SimulationReceiver` is protobuf
deserialization. Doing this in GDScript requires a hand-rolled wire-format
decoder that must be maintained in sync with `liteaerosim.proto`. The C++
runtime handles all parsing including future field additions and is the correct
tool for the job. The GDScript placeholder serves until the GDExtension build is
added to the project.

### GP-DR-2 — `TerrainLoader` remains GDScript (provisional)

**Decision:** `TerrainLoader` remains GDScript for the initial implementation
(OQ-GP-1 Option A).

**Rationale:** `TerrainLoader` performs no operations that require C++ — it reads
a JSON file, loads a GLB resource, and traverses the scene tree. All of these
are idiomatic GDScript operations. Moving it to C++ adds no correctness or
performance benefit.

**Acknowledged risk:** The cross-language boundary between `TerrainLoader.gd`
and the C++ `SimulationReceiver` (the `set_world_origin()` call) is a
maintenance liability. If the plugin grows beyond its current two components,
this decision should be revisited.

### GP-DR-3 — godot-cpp version parsed from `project.godot`

**Decision:** The `godot-cpp` FetchContent `GIT_TAG` is derived automatically
by parsing `godot/project.godot` in CMake (OQ-GP-2 Option B).

**Rationale:** `project.godot` is the authoritative Godot editor version source,
updated automatically by the editor on every upgrade. Parsing it eliminates the
only manual synchronization step between the editor version and the godot-cpp
build dependency.

### GP-DR-4 — GDExtension built via root CMakeLists `add_subdirectory`

**Decision:** The plugin is built under `LITEAERO_SIM_BUILD_GODOT_PLUGIN=OFF`
(default), enabled explicitly by developers doing Godot work (OQ-GP-3 Option B).

**Rationale:** Consistent with the project's pattern for optional build targets
(Python bindings). A single CMake configure covers the full project when the flag
is set; the large `godot-cpp` FetchContent is not pulled into CI or C++-only
builds.

---

## Roadmap

The GDExtension implementation is roadmap item **GP-1**. Prerequisite: protobuf
generated sources are available in the main build tree (already satisfied).
Predecessor: GDScript placeholder (already functional — no blocking dependency
on GP-1 for early live-sim testing).

GP-1 is not blocking for the first live-sim run. The GDScript placeholder is
sufficient for development. GP-1 should be implemented before the plugin is
considered production-quality.
