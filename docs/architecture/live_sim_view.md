# Live Simulation Viewer — Architecture and Design

Design authority for the live simulation pipeline: the C++ `SimulationFrame` broadcast
path and the Godot 4 visualization scene that receives it.

Implemented by roadmap item LS-1. Blocking dependencies: SB-1 (Aircraft / SimRunner
Python bindings — required for `SimSession` scripted input path), MI-1 (`JoystickInput`,
`ScriptedInput`, `AircraftCommand`).

**SB-2 (ring buffer) is not a blocking dependency.** The MVP live sim rendering path is
`SimRunner → SimulationFrame → UDP → Godot 4`. The ring buffer is not in that path.
SB-2 is useful for Python-side logging and post-run analysis but is not required for LS-1.

**PP-2 is not a blocking dependency.** The Vispy post-processing infrastructure is not
used for live rendering.

---

## Purpose

The live simulation viewer closes the loop between the C++ simulation engine and a
real-time 3D display. `SimRunner` drives `Aircraft::step()` in `RealTime` mode; after
each step it populates a `SimulationFrame` value object and broadcasts it via UDP to
a fixed local port; a **Godot 4** scene receives the datagrams and updates the vehicle
actor transform and ribbon trail each render frame.

Godot 4 is the selected real-time renderer — see
[`docs/architecture/system/future/decisions.md`](system/future/decisions.md) decision
row 26 and [`docs/architecture/terrain.md §Game Engine Integration`](terrain.md#game-engine-integration).
Rationale: MIT license, native glTF 2.0 / GLB import, Vulkan-based renderer, C++
integration via GDExtension.

This is the first item in the roadmap where the full loop closes:

```text
Operator / Script
    | AircraftCommand (JoystickInput or ScriptedInput, via Python SimSession)
    v
SimRunner::runLoop()  (C++ background thread, RealTime mode)
    | Aircraft::step()  ->  KinematicState
    | populate SimulationFrame
    v
ISimulationBroadcaster::broadcast()
    | UDP datagram  ->  fixed local port (default 14560)
    v
Godot 4 GDExtension plugin
    | convert geodetic -> ENU offset from world origin
    | apply body-to-Godot rotation from quaternion
    v
Godot 3D scene  (terrain GLB tiles, aircraft mesh GLB, ribbon trail, HUD overlay)
```

---

## Use Case Decomposition

| ID | Use Case | Primary Actor | Mechanism |
| --- | --- | --- | --- |
| UC-LS1 | Run live simulation with joystick FBW input | Operator | CLI launcher; `JoystickInput` → `SimSession` → `SimRunner` |
| UC-LS2 | Run live simulation with scripted input | Developer | Notebook / CLI; `ScriptedInput.push()` from Python |
| UC-LS3 | Observe 3D trajectory in real time | Developer / Operator | Godot scene trailing camera tracks aircraft |
| UC-LS4 | Change camera mode during flight | Operator | Godot scene UI |
| UC-LS5 | Stop the simulation and close the window | Operator | Close Godot window; `SimSession.stop()` in Python |
| UC-LS6 | Launch from a Jupyter notebook | Developer | `SimSession` in notebook cell; Godot window launched separately |

---

## Architectural Issues and Required Corrections

The following C++ gaps block a correct live simulation implementation.

### Issue 1 — `SimulationFrame` and `ISimulationBroadcaster` are not yet implemented

`terrain.md §Live Simulation Streaming` specifies the `SimulationFrame` value object
and the `ISimulationBroadcaster` interface but neither is implemented. The required
new files are:

- `include/SimulationFrame.hpp` — plain value struct, no I/O
- `include/broadcaster/ISimulationBroadcaster.hpp` — pure interface
- `include/broadcaster/UdpSimulationBroadcaster.hpp`
- `src/broadcaster/UdpSimulationBroadcaster.cpp`

### Issue 2 — `SimRunner` does not call a broadcaster

`SimRunner::runLoop()` calls `aircraft_.step()` but does not populate or broadcast a
`SimulationFrame`. Required change: add `SimRunner::set_broadcaster(ISimulationBroadcaster*)`;
call `broadcaster_->broadcast(frame)` after each `step()` if a broadcaster is set.

### Issue 3 — `SimRunner.set_broadcaster()` is not bound to Python

Once Issue 2 is resolved in C++, `bind_runner.cpp` must expose
`SimRunner.set_broadcaster(broadcaster)` so Python can wire `UdpSimulationBroadcaster`
into the session.

### Issue 4 — `SimRunner.setManualInput()` is not exposed to Python; `JoystickInput` is only partially bound

`SimRunner::setManualInput(ManualInput*)` is not bound in `bind_runner.cpp`. `JoystickInput`
is bound only for its static `enumerate_devices()` method. See OQ-LS-5.

### Issue 5 — GDExtension C++ plugin not yet implemented (GDScript placeholder in use)

The Godot project (`godot/`) and a functional GDScript placeholder exist.
`SimulationReceiver.gd` implements the full UDP receive and coordinate transform
pipeline in GDScript with a hand-rolled proto3 wire-format parser. This is
sufficient for development but the hand-rolled parser is fragile — see the
detailed design specification in
[`docs/architecture/godot_plugin.md`](godot_plugin.md) for the GDExtension C++
implementation that replaces it. The GDScript placeholder is not blocking for
the first live-sim run.

---

## C++ Component Design

### `SimulationFrame`

**File:** `include/SimulationFrame.hpp`

Plain value struct in the Domain Layer. No I/O, no serialization machinery. Defined in
[`terrain.md §Live Simulation Streaming`](terrain.md#live-simulation-streaming).

```cpp
// include/SimulationFrame.hpp  (Domain Layer value object — no I/O)
namespace liteaero::simulation {

struct SimulationFrame {
    double timestamp_s;          // simulation time (s)
    double latitude_rad;         // vehicle geodetic latitude, WGS84 (rad)
    double longitude_rad;        // vehicle geodetic longitude, WGS84 (rad)
    float  height_wgs84_m;       // vehicle ellipsoidal altitude (m)
    float  q_w, q_x, q_y, q_z; // body-to-NED attitude quaternion
    float  velocity_north_mps;
    float  velocity_east_mps;
    float  velocity_down_mps;
};

} // namespace liteaero::simulation
```

### `ISimulationBroadcaster`

**File:** `include/broadcaster/ISimulationBroadcaster.hpp`

Pure interface in the Interface Layer. `SimRunner` depends on this interface, not on
any transport implementation.

```cpp
namespace liteaero::simulation {

class ISimulationBroadcaster {
public:
    virtual ~ISimulationBroadcaster() = default;
    virtual void broadcast(const SimulationFrame& frame) = 0;
};

} // namespace liteaero::simulation
```

### `UdpSimulationBroadcaster`

**File:** `include/broadcaster/UdpSimulationBroadcaster.hpp`,
`src/broadcaster/UdpSimulationBroadcaster.cpp`

UDP implementation of `ISimulationBroadcaster`. Serializes `SimulationFrame` as a
protobuf message and sends it as a UDP datagram to `127.0.0.1:<port>` (default port
14560). Uses a new `SimulationFrameProto` message added to `liteaerosim.proto`
(resolved by OQ-LS-7 Option B).

```cpp
namespace liteaero::simulation {

class UdpSimulationBroadcaster : public ISimulationBroadcaster {
public:
    explicit UdpSimulationBroadcaster(uint16_t port = 14560);
    ~UdpSimulationBroadcaster() override;

    void broadcast(const SimulationFrame& frame) override;

private:
    int socket_fd_;
    uint16_t port_;
};

} // namespace liteaero::simulation
```

### `SimRunner` modifications

Two additions to `SimRunner`:

1. `set_broadcaster(ISimulationBroadcaster* broadcaster)` — stores a non-owning pointer;
   pass `nullptr` to disable broadcasting. Called before `start()`.
2. After each `Aircraft::step()` call in `runLoop()`, if `broadcaster_ != nullptr`:
   populate a `SimulationFrame` from the current `KinematicState` and call
   `broadcaster_->broadcast(frame)`.

`SimRunner` does not own the broadcaster. The caller (Python session or C++ main) is
responsible for broadcaster lifetime; the broadcaster must outlive the `SimRunner` run.

---

## Python `SimSession` Design

**File:** `python/tools/live_sim_session.py`

`SimSession` owns the C++ objects for the lifetime of the simulation. It constructs
`Aircraft`, `SimRunner`, and `UdpSimulationBroadcaster` via pybind11 bindings, wires
them together, and starts the run. It does not own a display window.

```python
class SimSession:
    def __init__(
        self,
        aircraft_config: str,           # JSON string or file path
        mode: str = "realtime",
        dt_s: float = 0.02,
        duration_s: float = 0.0,        # 0 = run until stop()
        broadcast_port: int = 14560,
    ) -> None: ...
    # Constructs Aircraft, SimRunner, UdpSimulationBroadcaster.
    # Calls runner.set_broadcaster(broadcaster).

    def start(self) -> None: ...        # calls runner.start(); returns immediately
    def stop(self) -> None: ...         # calls runner.stop(); blocks until stopped
    def is_running(self) -> bool: ...

    def set_scripted_input(self, scripted: ScriptedInput) -> None: ...
    def set_joystick_input(self, joystick_config: str) -> None: ...

    @property
    def scripted_input(self) -> ScriptedInput: ...
```

`drain_kinematic()` is no longer required on `SimSession` for rendering — the Godot
scene receives state via UDP. It may still be exposed for logging and test purposes (the
ring buffer is still populated by `SimRunner`).

**Lifetime contract:** `SimSession` keeps `Aircraft`, `SimRunner`, and
`UdpSimulationBroadcaster` alive for its entire lifetime. `stop()` must be called before
`SimSession` is released.

---

## Godot Plugin Design

**Repository location:** `godot/` (Godot 4 project root)

The Godot plugin consists of two components:

| Component | Language | Status |
| --- | --- | --- |
| `SimulationReceiver` | GDScript placeholder → GDExtension C++ | Placeholder functional; GDExtension specified but not yet built |
| `TerrainLoader` | GDScript | Implemented |

Full design specification, file structure, build system, class API, and open
questions are in [`docs/architecture/godot_plugin.md`](godot_plugin.md).

### Scene hierarchy

```text
World (Node3D)
├── WorldEnvironment (ambient light, energy=0.1)
├── TerrainLoader (Node — TerrainLoader.gd)
│     instantiates terrain GLB tiles as children at runtime
├── DirectionalLight3D  (primary sun — shadow on, energy=1.5)
├── DirectionalLight3D  (fill — shadow off, energy=1.5, illuminates aft faces)
├── Vehicle (Node3D)
│   ├── SimulationReceiver (Node3D — GDExtension C++)
│   └── AircraftMesh (instanced by TerrainLoader from mesh_res_path GLB)
└── Camera3D
```

### `SimulationReceiver` behaviour per `_process()` frame

1. Read all pending UDP datagrams (non-blocking socket; up to `max_datagrams_per_frame`).
2. For the latest datagram: deserialize `SimulationFrameProto` (via protobuf C++ runtime
   in GDExtension; via hand-rolled parser in GDScript placeholder).
3. Compute ENU offset from world origin (set by `TerrainLoader._ready()`):

   ```text
   east_m  = (lon - lon_0) * R_earth * cos(lat_0)
   north_m = (lat - lat_0) * R_earth
   up_m    = height_wgs84_m - h_0
   ```

   Godot position: `Vector3(east_m, up_m, -north_m)`.
4. Convert body-to-NED quaternion to body-to-Godot quaternion using the
   NED→Godot rotation quaternion $(x=0.5,\ y=0.5,\ z=-0.5,\ w=0.5)$
   (Godot `Quaternion` constructor order is `(x, y, z, w)`).
5. Set `Vehicle.position` and `Vehicle.quaternion`.

### World origin

`TerrainLoader._ready()` reads `terrain_config.json` and sets the world origin
on `SimulationReceiver` before any UDP packet arrives. If terrain is not loaded,
`SimulationReceiver` logs an error on the first received frame.

---

## Coordinate System

The Godot scene uses ENU coordinates as specified in
[`terrain.md §Coordinate Frame Mapping`](terrain.md#coordinate-frame-mapping):

$$\text{Godot/glTF}(X,\ Y,\ Z) = \text{ENU}(+\text{East},\ +\text{Up},\ -\text{North})$$

| Frame | East | Up | −North | Notes |
| --- | --- | --- | --- | --- |
| ENU (terrain local grid) | +X | +Z | −Y | Domain Layer storage |
| glTF 2.0 canonical | +X | +Y | +Z | GLB export format |
| Godot 4 | +X | +Y | +Z | Same as glTF — no rotation needed for terrain |

### NED-to-Godot frame rotation

`SimulationFrame` carries a body-to-NED quaternion `q_b2n`. To drive the Godot vehicle
transform, the GDExtension plugin converts it to body-to-Godot:

```text
R_NED_to_Godot:
    NED +North -> Godot -Z    (North = -Z_Godot)
    NED +East  -> Godot +X    (East  = +X_Godot)
    NED +Down  -> Godot -Y    (Down  = -Y_Godot)
```

As a rotation matrix acting on NED column vectors:

```text
       [  0   1   0  ]
R =    [  0   0  -1  ]
       [ -1   0   0  ]
```

The `Vehicle` node's Godot quaternion is set by `SimulationReceiver`:

$$q_\text{Godot} = R_\text{NED\to Godot} \otimes q_{b2n}$$

The aircraft mesh is a child `AircraftMesh` node of `Vehicle` with a fixed local
rotation `R_\text{correction}` (see §Aircraft Mesh Coordinate Frame below).
The mesh correction is **not** folded into `q_Godot` — it is a static node rotation
applied once at scene setup by `TerrainLoader`.

### Aircraft Mesh Coordinate Frame

#### Three coordinate frames

There are three distinct frames that must be kept clearly separate:

**1 — Mesh layout frame** (vertices as stored in aircraft GLBs, generated from layout
drawings with fuselage station / buttline / waterline axes):

| Mesh layout axis | Physical direction |
| --- | --- |
| +X | Aft (increasing fuselage station) |
| +Y | Starboard (right buttline) |
| +Z | Up (waterline) |

Godot's GLTF importer reads vertex positions as-is; no axis conversion is applied.

**2 — NED body frame** (physics convention used by `SimulationFrame.q_b2n`):

| NED body axis | Physical direction |
| --- | --- |
| +X | Nose (forward / North in level flight) |
| +Y | Starboard (East in level flight) |
| +Z | Down |

At level, wings-level flight the body frame is aligned with NED:
body +X = North, body +Y = East, body +Z = Down.

**3 — Godot world frame:**

| Godot axis | Physical / ENU direction |
| --- | --- |
| +X | East |
| +Y | Up |
| +Z | −North |

#### Layout-to-body frame correction

`TerrainLoader` applies `mesh_node.rotation_degrees = Vector3(0, 180, 0)` to all
aircraft meshes. This is a pure Ry(180°) rotation:

$$R_\text{correction} = R_y(180°) = \begin{bmatrix} -1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & -1 \end{bmatrix}$$

This maps layout axes to NED body axes:

| Layout mesh axis | After Ry(180°) | Physical meaning |
| --- | --- | --- |
| +X (aft) | −X | Forward (nose) |
| +Y (right) | +Y | Starboard (unchanged) |
| +Z (up) | −Z | Down (NED body convention) |

The corrected mesh frame (+X=fwd, +Y=right, +Z=down) is exactly the NED body frame,
so `q_b2n` from the simulation drives the mesh orientation correctly with no residual
attitude error.

#### Live simulation rotation chain

During simulation, `SimulationReceiver` sets:

$$q_\text{Vehicle} = R_\text{NED\to Godot} \otimes q_{b2n}$$

The `AircraftMesh` global rotation is:

$$R_\text{mesh,global} = q_\text{Vehicle} \cdot R_y(180°) = R_\text{NED\to Godot} \cdot q_{b2n} \cdot R_y(180°)$$

At level flight ($q_{b2n}$ = identity), the mesh +X axis (fwd in layout) maps to Godot
−Z (−North in world frame).
All three rotation axes (roll, pitch, yaw) have been verified with ±30° perturbations
and a compound 75°roll + 30°pitch test case using the `test_prism.glb` asset with
embossed axis labels.

---

## Implementation Pitfalls

The following issues were discovered during initial implementation and are documented
here to prevent regression.

### Godot `Quaternion` constructor argument order

Godot's `Quaternion` constructor takes `(x, y, z, w)` — **not** `(w, x, y, z)`.

```cpp
// WRONG — passes proto q_w as Godot x, producing identity mapped to (x=1,y=0,z=0,w=0):
Quaternion q_b2n(frame.q_w(), frame.q_x(), frame.q_y(), frame.q_z());

// CORRECT:
Quaternion q_b2n(frame.q_x(), frame.q_y(), frame.q_z(), frame.q_w());
```

The NED→Godot quaternion is similarly specified in Godot's (x, y, z, w) order:

```cpp
// x=0.5, y=0.5, z=-0.5, w=0.5  (NOT w=0.5, x=0.5, y=0.5, z=-0.5)
Quaternion r_ned_to_godot(0.5f, 0.5f, -0.5f, 0.5f);
```

### `is_editor_hint()` guard in GDExtension `_ready()` and `_process()`

GDExtension `_ready()` is called when the scene is opened in the Godot editor, not
only at runtime. If `SimulationReceiver::_ready()` opens a UDP socket without an
editor guard, the socket is bound in the editor session. When the game starts, the
second `bind()` silently fails and no datagrams are received.

Both `_ready()` and `_process()` must begin with:

```cpp
if (Engine::get_singleton()->is_editor_hint()) return;
```

### Initial attitude in aircraft config

`Aircraft.cpp` reads the following optional fields from `initial_state` in the
aircraft config JSON:

| Field | Unit | Default | Description |
| --- | --- | --- | --- |
| `roll_rad` | rad | 0.0 | Initial bank angle (positive = right wing down) |
| `pitch_rad` | rad | 0.0 | Initial pitch angle (positive = nose up) |
| `heading_rad` | rad | 0.0 | Initial heading (0 = North, positive = clockwise) |

The initial attitude quaternion is `q_nb = Rz(heading) · Ry(pitch) · Rx(roll)`
(ZYX Euler, standard aerospace convention). Example:

```json
"initial_state": {
    "roll_rad": 0.5236,
    "pitch_rad": 0.0,
    "heading_rad": 0.0
}
```

Test configs `test_axis_prism_roll30.json`, `test_axis_prism_pitch30.json`,
`test_axis_prism_yaw30.json`, and `test_axis_prism_roll75_pitch30.json` demonstrate
these fields. Landing gear is omitted from orientation test configs to prevent gear
spring forces from correcting the initial attitude.

---

## Launch Modes

### CLI — `python/tools/live_sim.py`

The Python CLI starts the simulation and UDP broadcaster. The Godot scene is launched
separately (either from the Godot editor or as a pre-built executable). See OQ-LS-8.

```bash
# Start simulation with scripted input (broadcasts to port 14560):
python tools/live_sim.py \
    --config aircraft_configs/ga_aircraft.json \
    --scripted \
    --dt 0.02

# With joystick:
python tools/live_sim.py \
    --config aircraft_configs/ga_aircraft.json \
    --joystick gx12_config.json
```

**Arguments:**

| Argument | Type | Description |
| --- | --- | --- |
| `--config` | path | Aircraft JSON config (required) |
| `--scripted` | flag | Use `ScriptedInput` with neutral commands (default if `--joystick` absent) |
| `--joystick` | path | `JoystickInput` JSON config; enables joystick mode |
| `--dt` | float | Simulation timestep in seconds (default `0.02`) |
| `--port` | int | UDP broadcast port (default `14560`) |

### Notebook — `python/live_sim_demo.ipynb`

```python
# Cell 1 — create and start session
from live_sim_session import SimSession

session = SimSession("aircraft_configs/ga_aircraft.json")
session.start()
# Godot scene already open and listening on port 14560

# Cell 2 — inject scripted commands
from liteaero_sim_py import AircraftCommand, ScriptedInput
scripted = ScriptedInput()
session.set_scripted_input(scripted)
scripted.push(AircraftCommand(n_z=1.0, throttle_nd=0.5))

# Cell 3 — stop
session.stop()
```

---

## Threading Model

```text
Python main thread (notebook / CLI)
└── SimSession: constructs C++ objects, calls runner.start()

C++ background thread (spawned by SimRunner.start())
└── runLoop():
        Aircraft::step()
        → populate SimulationFrame from KinematicState
        → ISimulationBroadcaster::broadcast()  (UDP send, non-blocking)
        → ChannelRegistry::publish()  (ring buffer, for logging)

Godot main thread (separate process)
└── _process() each render frame:
        SimulationReceiver::poll_socket()
        → update Vehicle transform
        → update RibbonTrail
        → update HudOverlay
```

The UDP send in `UdpSimulationBroadcaster::broadcast()` is a non-blocking `sendto()`
on a pre-opened socket. It does not block the simulation thread. Dropped datagrams
(e.g., when Godot is not running) are silently discarded.

---

## Test Strategy

**C++ tests** (`test/SimulationBroadcaster_test.cpp`):

| Test | Pass criterion |
| --- | --- |
| `test_simulation_frame_size` | `sizeof(SimulationFrame)` matches expected byte count |
| `test_udp_broadcaster_sends_datagram` | `UdpSimulationBroadcaster` sends a datagram readable by a loopback UDP receiver |
| `test_sim_runner_broadcasts_after_step` | After `runner.start()` + 0.1 s + `runner.stop()`, test UDP receiver has received ≥ 1 datagram |

**Python tests** (`python/test/test_live_sim_session.py`):

| Test | Pass criterion |
| --- | --- |
| `test_sim_session_initializes` | `SimSession(config)` constructs without error; `is_running()` is False |
| `test_sim_session_start_stop` | `start()` + `stop()` completes without deadlock; `is_running()` is False after stop |
| `test_sim_session_broadcasts` | After `start()` + 0.1 s sleep + `stop()`, a loopback UDP socket on port 14560 has received ≥ 1 datagram |
| `test_sim_session_datagram_size` | Received datagram size matches `sizeof(SimulationFrame)` |
| `test_scripted_input_wiring` | `set_scripted_input()` + `push(AircraftCommand(...))` does not raise |
| `test_cli_argument_parsing` | `argparse` namespace contains expected attributes for all argument combinations |

---

## File Map

| File | Role |
| --- | --- |
| `include/SimulationFrame.hpp` | `SimulationFrame` value struct |
| `include/broadcaster/ISimulationBroadcaster.hpp` | Pure broadcaster interface |
| `include/broadcaster/UdpSimulationBroadcaster.hpp` | UDP implementation header |
| `src/broadcaster/UdpSimulationBroadcaster.cpp` | UDP implementation |
| `test/SimulationBroadcaster_test.cpp` | C++ broadcaster tests |
| `python/tools/live_sim_session.py` | `SimSession` — Python scripted/notebook launcher |
| `python/tools/live_sim.py` | Python CLI launcher (scripted input only; no joystick) |
| `tools/live_sim.cpp` | C++ joystick + terrain launcher; loads `TerrainMesh` from `las_terrain_path` in `terrain_config.json`; calls `aircraft.setTerrain()` |
| `python/live_sim_demo.ipynb` | Notebook demo |
| `python/test/test_live_sim_session.py` | Python session tests |
| `python/assets/aircraft_lp.glb` | Low-poly aircraft mesh (323 triangles) |
| `godot/` | Godot 4 project root |
| `godot/addons/liteaero_sim/` | GDExtension plugin source |

---

## Open Questions

### OQ-LS-5 — `JoystickInput` wiring approach

**Resolved — Option A.**

`JoystickInput` is wired to `SimRunner` directly in C++. A `live_sim` C++ executable
(`src/tools/live_sim.cpp`) owns `Aircraft`, `SimRunner`, `JoystickInput`, and
`UdpSimulationBroadcaster`; wires them together; and blocks on the run loop. No Python
binding for `JoystickInput` is required or added. Python `SimSession` handles scripted
and notebook use only.

**Required new file:** `src/tools/live_sim.cpp` with a corresponding CMake executable
target. `JoystickInput` config path and device index are CLI arguments to the binary.

---

### OQ-LS-6 — Float precision of ring buffer position channels

**Deferred to SB-3 (ring buffer redesign).**

The `SB-2` ring buffer `Sample::value` is scalar `float`, which is inadequate not just
for geodetic position precision but as a general design — it cannot represent vectors or
quaternions atomically. The appropriate fix is a redesign of the channel type system
(roadmap item SB-3), not a patch to `SimRunner`. This question is not blocking for
LS-1: Godot receives `double` geodetic coordinates directly in `SimulationFrame` and
does not use the ring buffer.

---

### OQ-LS-7 — UDP datagram serialization format for `SimulationFrame`

**Resolved — Option B.**

`UdpSimulationBroadcaster` serializes `SimulationFrame` as a protobuf message.
A new `SimulationFrameProto` message is added to `liteaerosim.proto`:

```proto
message SimulationFrameProto {
    double timestamp_s         = 1;
    double latitude_rad        = 2;
    double longitude_rad       = 3;
    float  height_wgs84_m      = 4;
    float  q_w                 = 5;  // body-to-NED attitude quaternion
    float  q_x                 = 6;
    float  q_y                 = 7;
    float  q_z                 = 8;
    float  velocity_north_mps  = 9;
    float  velocity_east_mps   = 10;
    float  velocity_down_mps   = 11;
}
```

This extends the `TrajectoryFrame` schema (fields 1–8) with velocity fields (9–11).
The schema is self-describing, version-tolerant, and endian-safe. Protobuf serialization
overhead at 400 Hz is negligible (~microseconds per frame, < 20 KB/s). The Godot
GDExtension plugin receives and deserializes the message using the C++ protobuf runtime
(or the GDScript protobuf addon).

---

### OQ-LS-8 — Godot scene launch mechanism

**Resolved — Option C.**

Godot runs independently; Python does not manage the Godot process. The developer
opens the Godot scene once per session (from the editor or a pre-built export). The
Python CLI and C++ `live_sim` binary broadcast UDP regardless of whether Godot is
running — datagrams are silently dropped if no listener is present. This matches the
fire-and-forget UDP model naturally and requires no process management code.

---

### OQ-LS-9 — Aircraft mesh coordinate frame correction in Godot

**Resolved — layout-frame meshes with `rotation_degrees = Vector3(0, 180, 0)`.**

Aircraft meshes are generated in the **layout frame** (+X=aft, +Y=right, +Z=up).
`TerrainLoader` applies `mesh_node.rotation_degrees = Vector3(0, 180, 0)` — a pure
Ry(180°) — which maps the layout frame to the NED body frame (+X=fwd, +Y=right,
+Z=down). The corrected mesh frame matches `q_b2n` exactly, producing correct
attitude display for both static and live-simulation cases.

This supersedes the earlier empirical `Vector3(-90, 0, 90)` approach, which was
derived for a mesh assumed to be in a different convention and produced incorrect
live-simulation attitude. The correct rotation has been verified with roll, pitch,
yaw, and compound rotation test cases using `test_prism.glb`.

---

### OQ-LS-11 — Mesh Z-axis sign mismatch and live-simulation attitude error

**Resolved — root cause eliminated by adopting layout-frame mesh convention.**

The original problem arose because meshes were assumed to be in a body-like frame
(+X=nose, +Y=right, +Z=up) which differs from the NED body frame (+Z=down) in Z
sign, with no proper rotation able to reconcile them.

**Resolution:** All aircraft meshes are now defined in the **layout frame**
(+X=aft, +Y=right, +Z=up). The layout-to-body correction `Ry(180°)` maps this to
exactly the NED body frame (+X=fwd, +Y=right, +Z=down). There is no axis mismatch
and no residual attitude error in live simulation.

This question is closed. See OQ-LS-9 for the resolved correction rotation.

---

### OQ-LS-10 — Terrain wiring in `live_sim.cpp` for landing gear contact

**Resolved — Option B.**

> **Naming note:** The terrain abstract base class in `liteaero-flight` is currently
> named `V_Terrain` (`liteaero/terrain/V_Terrain.hpp`). That name violates the
> project's `PascalCase`-no-prefix naming convention. It is used below as the current
> code name only; a rename cleanup is required (see `liteaero-flight` roadmap).

`LandingGear::step()` requires a const reference to the terrain abstract base, passed
through `Aircraft::setTerrain()`. Without this call, `Aircraft::step()` skips the gear
contact block (`_terrain == nullptr` guard in `Aircraft.cpp`), producing incorrect
physics for any ground-start scenario. Terrain wiring must occur in C++ — not via
`SimSession` — because `Aircraft::setTerrain()` is not bound in the Python pybind11
layer.

**Option B — `TerrainMesh` from `.las_terrain` file.**

`live_sim.cpp` reads `terrain_config.json` (written by `build_terrain.py`) to locate
the `.las_terrain` binary; loads `TerrainMesh` from it; calls
`aircraft.setTerrain(terrain_mesh)`. If `terrain_config.json` is absent or the
`.las_terrain` file cannot be loaded, the binary exits with an error — there is no
silent fallback to `FlatTerrain`. A terrain build is a required precondition for
running `live_sim.exe`.

`live_sim.cpp` constructs the `TerrainMesh` before calling `runner.initialize()`, calls
`aircraft.setTerrain(terrain_mesh)`, and keeps the mesh alive for the duration of the
run. The mesh load may add several seconds of startup time for large datasets.

---

## Decision Records

### LS-DR-1 — Renderer for live simulation

**Decision:** Godot 4 (MIT license, Vulkan, native glTF import, GDExtension C++ API).

Decided during terrain mesh implementation (Step 11/12); documented in
[`terrain.md §Game Engine Integration`](terrain.md#game-engine-integration) and
[`decisions.md` row 26](system/future/decisions.md). The live simulation viewer is a
Godot scene receiving UDP `SimulationFrame` datagrams from the C++ `SimRunner`.
Vispy is used only for post-processing (`TrajectoryView`); it has no role in the live
viewer.

### LS-DR-2 — Transport mechanism and serialization format

**Decision:** UDP unicast to `127.0.0.1:14560` (default port configurable);
serialized as protobuf `SimulationFrameProto` (OQ-LS-7 Option B).

Fire-and-forget; dropped datagrams are silently discarded. The simulation thread is
never blocked by the renderer. Consistent with the transport model specified in
`terrain.md §Live Simulation Streaming`. Protobuf is used (not raw binary) for
endian-safety, version tolerance, and consistency with the rest of the project's
serialization strategy.

### LS-DR-3 — `SimulationFrame` field set

**Decision:** Use the field set specified in `terrain.md §Live Simulation Streaming`:
position (geodetic double), attitude (body-to-NED quaternion float), velocity NED
(float). This is sufficient for Godot position + orientation + velocity HUD display.

### LS-DR-4 — Python does not own the Godot window

**Decision:** `SimSession` is a pure simulation controller (starts/stops `SimRunner` +
broadcaster). It does not launch, monitor, or close the Godot process. The Godot window
is managed independently by the developer (OQ-LS-8 Option C).

### LS-DR-5 — `JoystickInput` wiring approach

**Decision:** Option A — C++ `live_sim` executable owns `JoystickInput` directly.

No Python binding for `JoystickInput` is required. `src/tools/live_sim.cpp` is the
operator-facing joystick launcher; Python `SimSession` handles scripted and notebook
use only. Clean separation: no SDL lifecycle management from Python.

### LS-DR-6 — Ring buffer drain rate

**Decision:** No longer relevant for live rendering — the ring buffer is not the
rendering data path. The ring buffer is populated at `1/dt_s` Hz by `SimRunner` for
logging and post-processing consumers. Ring buffer drain rate for those consumers is
determined by their own polling interval.

---

## Open Questions — Status Summary

| ID | Question | Status | Blocking |
| --- | --- | --- | --- |
| OQ-LS-1 | `LiveSimView` class relationship (Vispy) | Superseded — Vispy not used for live rendering; Godot 4 is the renderer (LS-DR-1) | N/A |
| OQ-LS-2 | Incremental ribbon trail strategy (Vispy) | Superseded — ribbon trail is a Godot scene concern | N/A |
| OQ-LS-3 | Shadow rendering approach (Vispy) | Superseded — shadow is a Godot scene concern | N/A |
| OQ-LS-4 | Aircraft mesh asset sourcing | Resolved — `python/assets/aircraft_lp.glb` committed (323 triangles) | No |
| OQ-LS-5 | `JoystickInput` wiring approach | Resolved → Option A (C++ `live_sim` binary owns `JoystickInput` directly; no Python binding) | No |
| OQ-LS-6 | Float precision for ring buffer position | Deferred to SB-3 (ring buffer redesign) — not a SimRunner patch | No |
| OQ-LS-7 | UDP datagram serialization format | Resolved → Option B (protobuf `SimulationFrameProto`) | No |
| OQ-LS-8 | Godot scene launch mechanism | Resolved → Option C (developer opens Godot manually; Python/C++ broadcast regardless) | No |
| OQ-LS-9 | Aircraft mesh coordinate frame correction | Resolved — layout-frame meshes + `rotation_degrees = Vector3(0, 180, 0)` correct for both static and live simulation | No |
| OQ-LS-10 | Terrain wiring in `live_sim.cpp` for landing gear contact | Resolved → Option B (`TerrainMesh` from `.las_terrain`; error if absent — no `FlatTerrain` fallback) | No |
| OQ-LS-11 | Mesh Z-axis sign mismatch: live-simulation attitude error | Resolved — eliminated by layout-frame mesh convention; see OQ-LS-9 | No |
