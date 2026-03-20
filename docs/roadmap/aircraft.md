# Aircraft Class — Roadmap

`Aircraft` is the top-level physics model for a single simulated aircraft. It owns the
`KinematicState`, the aerodynamics model, and the propulsion model, and exposes a single
`step()` method that advances all three by one timestep given commanded load factors and
throttle. It lives in the Domain Layer and has no I/O, no display logic, and no unit
conversions.

Items are listed in dependency order. Each item follows TDD: write a failing test before
writing production code.

---

## Current State

| Component | File | Status |
| ----------- | ------ | -------- |
| `KinematicState` | `include/KinematicState.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `LiftCurveModel` | `include/aerodynamics/LiftCurveModel.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `LoadFactorAllocator` | `include/aerodynamics/LoadFactorAllocator.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `WGS84_Datum` | `include/navigation/WGS84.hpp` | ✅ Implemented |
| `AeroPerformance` | `include/aerodynamics/AeroPerformance.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `AirframePerformance` | `include/airframe/AirframePerformance.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `Inertia` | `include/airframe/Inertia.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `Propulsion` | `include/propulsion/Propulsion.hpp` | ✅ Implemented — derives from `DynamicElement`; replaces `V_Propulsion` |
| `PropulsionJet` | `include/propulsion/PropulsionJet.hpp` | ✅ Implemented + serialization (JSON + proto) — see [propulsion.md](../architecture/propulsion.md) |
| `PropulsionEDF` | `include/propulsion/PropulsionEDF.hpp` | ✅ Implemented + serialization (JSON + proto) — see [propulsion.md](../architecture/propulsion.md) |
| `PropellerAero` | `include/propulsion/PropellerAero.hpp` | ✅ Implemented — see [propulsion.md](../architecture/propulsion.md) |
| `Motor` | `include/propulsion/Motor.hpp` | ✅ Implemented — stateless abstract interface; replaces `V_Motor` |
| `MotorElectric` | `include/propulsion/MotorElectric.hpp` | ✅ Implemented — see [propulsion.md](../architecture/propulsion.md) |
| `MotorPiston` | `include/propulsion/MotorPiston.hpp` | ✅ Implemented — see [propulsion.md](../architecture/propulsion.md) |
| `PropulsionProp` | `include/propulsion/PropulsionProp.hpp` | ✅ Implemented + serialization (JSON + proto) — see [propulsion.md](../architecture/propulsion.md) |
| `Aircraft` | `include/Aircraft.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `Atmosphere` | `include/environment/Atmosphere.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `AtmosphericState` | `include/environment/AtmosphericState.hpp` | ✅ Implemented |
| `Wind` | `include/environment/Wind.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `Turbulence` | `include/environment/Turbulence.hpp` | ✅ Implemented + serialization (JSON) |
| `Gust` | `include/environment/Gust.hpp` | ✅ Implemented |
| `TurbulenceVelocity` | `include/environment/TurbulenceVelocity.hpp` | ✅ Implemented |
| `EnvironmentState` | `include/environment/EnvironmentState.hpp` | ✅ Implemented |
| `SurfaceGeometry` / `AircraftGeometry` | `include/aerodynamics/AircraftGeometry.hpp` | ✅ Implemented |
| `AeroCoeffEstimator` | `include/aerodynamics/AeroCoeffEstimator.hpp` | ✅ Implemented |
| `DynamicElement` | `include/DynamicElement.hpp` | ✅ Implemented — see [dynamic_element.md](../architecture/dynamic_element.md) |
| `SisoElement` | `include/SisoElement.hpp` | ✅ Implemented — NVI SISO wrapper over `DynamicElement` |
| `SensorAirData` | `include/sensor/SensorAirData.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `SensorGnss` | `include/sensor/SensorGnss.hpp` | 🔲 Stub only |
| `SensorLaserAlt` | `include/sensor/SensorLaserAlt.hpp` | 🔲 Stub only |
| `SensorMag` | `include/sensor/SensorMag.hpp` | 🔲 Stub only |
| `SensorInsSimulation` | `include/sensor/SensorInsSimulation.hpp` | 🔲 Stub only |
| `SensorAA` | `include/sensor/SensorAA.hpp` | 🔲 Stub only |
| `SensorAAR` | `include/sensor/SensorAAR.hpp` | 🔲 Stub only |
| `SensorRadAlt` | `include/sensor/SensorRadAlt.hpp` | 🔲 Stub only |
| `SensorForwardTerrainProfile` | `include/sensor/SensorForwardTerrainProfile.hpp` | 🔲 Stub only |
| `SensorTrackEstimator` | `include/sensor/SensorTrackEstimator.hpp` | 🔲 Stub only |
| `NavigationFilter` | `include/estimation/NavigationFilter.hpp` | 🔲 Stub only — see [navigation_filter.md](../architecture/navigation_filter.md) |
| `WindEstimator` | `include/estimation/WindEstimator.hpp` | 🔲 Stub only — see [wind_estimator.md](../architecture/wind_estimator.md) |
| `FlowAnglesEstimator` | `include/estimation/FlowAnglesEstimator.hpp` | 🔲 Stub only — see [flow_angles_estimator.md](../architecture/flow_angles_estimator.md) |
| `V_PathSegment` | `include/path/V_PathSegment.hpp` | 🔲 Stub only |
| `PathSegmentHelix` | `include/path/PathSegmentHelix.hpp` | 🔲 Stub only |
| `Path` | `include/path/Path.hpp` | 🔲 Stub only |
| `PathGuidance` | `include/guidance/PathGuidance.hpp` | 🔲 Stub only |
| `VerticalGuidance` | `include/guidance/VerticalGuidance.hpp` | 🔲 Stub only |
| `ParkTracking` | `include/guidance/ParkTracking.hpp` | 🔲 Stub only |
| `Autopilot` | `include/control/Autopilot.hpp` | 🔲 Stub only |

---

## Delivered

Design authority for all delivered items: [`docs/architecture/aircraft.md`](../architecture/aircraft.md).

| # | Item | Tests |
| --- | ------ | ------- |
| 1 | `AirframePerformance` — field renames, serialization (JSON + proto), `aircraft_config_v1` schema | `AirframePerformance_test.cpp` — 6 tests |
| 2 | `Inertia` — serialization (JSON + proto), `aircraft_config_v1` schema | `Inertia_test.cpp` — 6 tests |
| 3 | `Aircraft` class — `AircraftCommand`, `initialize()`, `reset()`, `state()` | `Aircraft_test.cpp` — 3 tests |
| 4 | `Aircraft::step()` — 9-step physics loop | `Aircraft_test.cpp` — 3 tests |
| 5 | `Aircraft` serialization — JSON + proto round-trips, schema version checks | `Aircraft_test.cpp` — 4 tests |
| 6 | JSON initialization — fixture-file tests (3 configs) and missing-field error path | `Aircraft_test.cpp` — 4 tests |
| 7 | `Logger` design — architecture, data model, MCAP + CSV formats, C++ interface reference | [`docs/architecture/logger.md`](../architecture/logger.md) |
| 8 | `Logger` implementation — `Logger`, `LogSource`, `LogReader`; MCAP + `FloatArray` proto; 6 tests | `test/Logger_test.cpp` — 6 tests |
| 9 | Environment model design — `Atmosphere` (ISA + ∆ISA + humidity), `Wind`, `Turbulence` (Dryden), `Gust` (1-cosine); rotational turbulence coupling to trim aero model defined | [`docs/architecture/environment.md`](../architecture/environment.md) |
| 10 | Aerodynamic coefficient estimation — derivation of all trim aero model inputs from wing/tail/fuselage geometry; DATCOM lift slope, Hoerner Oswald, Raymer $C_{D_0}$ buildup, $C_{L_q}$, $C_{Y_\beta}$, $C_{Y_r}$ | [`docs/algorithms/aerodynamics.md`](../algorithms/aerodynamics.md) |
| 11 | `Atmosphere` — ISA 3-layer + ΔT + humidity + density altitude; JSON + proto serialization | `Atmosphere_test.cpp` — 12 tests |
| 12 | `Wind` (Constant/PowerLaw/Log), `Turbulence` (Dryden 6-filter, Tustin-discretized), `Gust` (1-cosine MIL-SPEC-8785C); JSON serialization | `Wind_test.cpp` — 6 tests, `Turbulence_test.cpp` — 5 tests, `Gust_test.cpp` — 6 tests |
| 13 | `AeroCoeffEstimator` — geometry-to-coefficient derivation (Parts 1–8: AR, MAC, $C_{L_\alpha}$, $C_{L_\text{max}}$, Oswald $e$, $C_{D_0}$ buildup, $C_{L_q}$, $C_{Y_\beta}$, $C_{Y_r}$); `AeroPerformanceConfig` struct; four new `AeroPerformance` fields (`cl_q_nd`, `mac_m`, `cy_r_nd`, `fin_arm_m`); `AircraftGeometry`/`SurfaceGeometry` structs; `AeroPerformance` schema bumped to v2 | `AeroCoeffEstimator_test.cpp` — 11 tests |
| 14 | `Terrain` subsystem — `V_Terrain`, `FlatTerrain`, `TerrainMesh` (7 LODs, LOS, glTF export), `LodSelector`, `MeshQualityVerifier`, `SimulationFrame`/`TrajectoryFile`; Python ingestion pipeline (download, mosaic, geoid correction, triangulate, colorize, simplify, verify, export) | C++: 53 tests across 6 test files; Python: 28 tests pass + 1 skip — see [`terrain-implementation-plan.md`](terrain-implementation-plan.md) |
| 15 | `DynamicElement` refactoring — unified root base for all stateful components; `SisoElement` NVI SISO wrapper; `Filter` → `SisoElement`; `Propulsion` / `Motor` bases; deleted `DynamicBlock`, `DynamicFilterBlock`, `DynamicLimitBlock`, `V_Sensor`, `V_Propulsion`, `V_Motor` | No new tests — all 378 pre-existing tests pass |
| 16 | `SISOBlock` removal — deleted `SISOBlock.hpp`; `SisoElement` derives from `DynamicElement` only; `LimitBase`/`Limit`/`RateLimit`/`Integrator`/`Derivative`/`Unwrap` migrated to `SisoElement` with full `DynamicElement` lifecycle; `SisoElement::step()` NVI call order fixed (previous `in_` available during `onStep()`); `resetTo()` replaces `reset(float)` overloads | `Limit_test.cpp`, `RateLimit_test.cpp`, `Integrator_test.cpp`, `Derivative_test.cpp`, `Unwrap_test.cpp` — 10 new tests; all 394 tests pass |
| 17 | `Antiwindup` redesign — `AntiwindupConfig` struct with `enum class Direction`; `update(float)` replaces `operator=(float)`; `configure()`, `reset()`, `serializeJson()`/`deserializeJson()` added; `name` field removed; uninitialized-boolean bug fixed; `Integrator` serialization extended to embed `"antiwindup"` array | `Antiwindup_test.cpp` — 12 tests; `Integrator_test.cpp` — 2 new tests; all 408 tests pass |
| 18 | Control subsystem refactoring (Steps A–J, all nine issues in [`control_interface_review.md`](../architecture/control_interface_review.md)) — `FilterError`/`DiscretizationMethod` promoted to `enum class`; `LimitBase` deleted, `Limit`/`RateLimit` rebased to `SisoElement`; `Gain` API cleaned up (`set()`, `value()`, stubs removed); `FilterSS2Clip`, `FilterTF2`, `FilterTF`, `FilterFIR`, `FilterSS` migrated to NVI (`onStep()`/`onSerializeJson()`/`onDeserializeJson()`); shadow `_in`/`_out` members and no-op `Filter` defaults removed; `Unwrap` `ref_` field added (`setReference()`, NVI routing, serialization); `SISOPIDFF` derives from `DynamicElement` with full lifecycle, `snake_case_` member renames (`Kp`→`proportional_gain_`, `I`→`integrator_`, etc.), private limits, `ControlLoop` accessor renames (`out()`→`output()`, `pid`→`controller_`); `Integrator`/`Derivative` private member renames (`_dt`→`dt_s_`, `_Tau`→`tau_s_`, `limit`→`limit_`) | `FilterSS2Clip_test.cpp`, `FilterTF2_test.cpp`, `FilterFIR_test.cpp` (new), `FilterSS_test.cpp`, `Unwrap_test.cpp`, `SISOPIDFF_test.cpp` (new) — 29 new tests; 435 pass, 2 pre-existing `FilterTFTest` failures unchanged |
| 19 | `SensorAirData` — pitot-static air data computer; differential pressure ($q_c$) and static pressure ($P_s$) transducers with Gaussian noise, first-order Tustin lag, and fuselage crossflow pressure error (two-port symmetric crosslinked model); derives IAS, CAS, EAS, TAS, Mach, barometric altitude (Kollsman-referenced, troposphere + tropopause), OAT; RNG pimpl with seed + advance serialization; JSON + proto round-trips | `SensorAirData_test.cpp` — 19 tests; 454 pass, 2 pre-existing `FilterTFTest` failures unchanged |

---

## 1. Gain Scheduling — Design and Implementation

`Gain<T, NumAxes>` currently holds template parameters for value type and scheduling
axis count, but the scheduling logic is unimplemented (stubs removed in delivered item 18,
Step C). This item defines and implements the full gain scheduling architecture.

### Scope to Define

The design must address at minimum:

- **Lookup methods** — what interpolation strategies are supported (e.g. rectilinear
  table bilinear, nearest-neighbor, polynomial fit) and how they are selected.
- **Axis dimensions** — how `NumAxes` maps to physical scheduling variables
  (e.g. airspeed, altitude, angle of attack); how axes are labeled and units enforced.
- **Runtime update** — how a scheduled gain is evaluated at each step given the current
  scheduling variable values; whether evaluation is synchronous with `SISOPIDFF::step()`
  or driven externally.
- **Serialization** — how the gain table is stored and restored (JSON + proto).
- **Initialization** — whether the gain table is loaded from a file, embedded in config
  JSON, or populated programmatically.

### Deliverables — Gain Scheduling

Design authority document at `docs/architecture/gain_scheduling.md` to be written
before implementation begins.

Implementation follows TDD: failing tests before production code.

---

## 2. Landing Gear — Ground Contact Model

`LandingGear` is a Domain Layer physics component that models the contact forces and
moments exerted on the airframe by the landing gear during ground operations (taxi,
takeoff roll, and landing roll). It produces forces and moments in the body frame that
are added to the aerodynamic and propulsion contributions in `Aircraft::step()`.

No design work is to be performed at this stage. The item is placed here to reflect
priority relative to the autopilot chain; detailed design precedes implementation.

### Scope

The design must address at minimum:

- **Wheel contact model** — Pacejka "magic formula" for tyre longitudinal and lateral
  force generation as a function of slip ratio and slip angle. The formula captures the
  nonlinear saturation of tyre force with slip and reproduces the qualitative shape of
  measured tyre data without requiring high-fidelity tyre datasets.
- **Suspension dynamics** — second-order spring-damper model per strut. Configuration
  parameters are spring stiffness (N/m), damper coefficient (N·s/m), and strut
  preload force (N). Suspension travel is constrained between fully extended and
  fully compressed limits.
- **Wheel geometry** — each wheel unit is defined by its attachment point in the body
  frame and a unit vector defining the suspension travel axis, both expressed in body
  coordinates. This supports tricycle, taildragger, and multi-bogey layouts without
  special-casing.
- **Computational efficiency** — the landing gear contact model must not force the
  aircraft simulation to run at a higher timestep rate than the rigid-body integrator.
  High-frequency strut and tyre dynamics that do not materially influence the aircraft
  trajectory response should be suppressed or filtered. An inner-step sub-loop — similar
  to the approach used for high-bandwidth FBW axis dynamics — should be evaluated as a
  mechanism to decouple stiff strut dynamics from the outer rigid-body loop if it allows
  the outer step rate to remain at the standard simulation rate.
- **Ground plane interface** — the model queries terrain height and surface normal at
  the projected wheel contact point; this interface must be compatible with both
  `FlatTerrain` and `TerrainMesh`.
- **Serialization** — full JSON and proto round-trip serialization of suspension state
  (strut deflection and deflection rate per wheel unit); RNG state if stochastic runway
  roughness is added.

### Deliverables — Landing Gear

Design authority document at `docs/architecture/landing_gear.md` to be written
before implementation begins.

Implementation follows TDD: failing tests before production code.

---

## 3. Autopilot Gain Design — Python Tooling

Python workflow that derives autopilot control gains from the aircraft model. This is a
prerequisite for item 4 (`Autopilot`) — the C++ implementation is parameterized by gains
computed here.

Scope to be defined when this item is scheduled. Expected to use Python control-system
libraries (e.g. `python-control`, `scipy.signal`) applied to linearized models extracted
from `Aircraft` trim and `AeroCoeffEstimator` outputs.

---

## 4. Autopilot — Inner Loop Knobs-Mode Tracking


Stub header exists at `include/control/Autopilot.hpp`.

`Autopilot` implements the inner closed-loop layer. It tracks pilot-style set point commands
corresponding to "knobs" modes — altitude hold, vertical speed hold, heading hold, and roll
attitude hold — and produces an `AircraftCommand` for `Aircraft::step()`. Guidance (item 6)
is the outer loop that wraps around it and supplies the set point commands. Control gains are
derived by the Python gain design workflow (item 3).

### Interface sketch

_To be defined during design. Inputs will include `KinematicState`, `AtmosphericState`,
and a set point struct (target altitude, target vertical speed, target heading, target roll
attitude). Output is `AircraftCommand`._

### Tests

- Altitude hold: starting from a displaced altitude, output drives altitude error to zero
  within the expected settling time.
- Heading hold: starting from a heading offset, output drives heading error to zero without
  overshoot beyond a specified bound.
- Roll attitude hold: commanded roll angle is tracked with correct steady-state and transient.
- Vertical speed hold: commanded climb rate is tracked correctly.
- JSON and proto round-trips preserve filter states.

### CMake

Add `src/control/Autopilot.cpp` to `liteaerosim`.
Add `test/Autopilot_test.cpp` to the test executable.

---

## 5. Path Representation — `V_PathSegment`, `PathSegmentHelix`, `Path`

Stub headers exist in `include/path/` for all three classes.

A `Path` is an ordered sequence of `V_PathSegment` objects. Each segment exposes a
cross-track error, along-track distance, and desired heading at a query position. The
initial concrete segment type is `PathSegmentHelix` (straight line is a degenerate helix
with infinite radius).

### Interface sketch

```cpp
// include/path/V_PathSegment.hpp
namespace liteaerosim::path {

struct PathQuery {
    Eigen::Vector3f position_NED_m;
    float heading_rad;
};

struct PathResponse {
    float crosstrack_error_m;     // positive = right of path
    float along_track_m;          // distance from segment start
    float desired_heading_rad;    // commanded heading at query point
    float desired_altitude_m;
    bool  segment_complete;       // true when along_track_m >= segment_length_m
};

class V_PathSegment {
public:
    virtual PathResponse query(const PathQuery& q) const = 0;
    virtual float length_m() const = 0;
    virtual ~V_PathSegment() = default;
};

} // namespace liteaerosim::path
```

### Tests — `PathSegmentHelix`

- On a straight segment (infinite radius), cross-track error equals the perpendicular
  distance from the line.
- At the midpoint of a circular arc, `desired_heading_rad` is tangent to the arc.
- `segment_complete` is false before the end and true after `along_track_m >= length_m`.

### Tests — `Path`

- A path with two segments advances to the second segment when the first is complete.
- `Path::query()` delegates to the active segment.

### CMake

Add `src/path/PathSegmentHelix.cpp` and `src/path/Path.cpp` to `liteaerosim`.
Add `test/Path_test.cpp` to the test executable.

---

## 6. Guidance — `PathGuidance`, `VerticalGuidance`, `ParkTracking`

Stub headers exist in `include/guidance/` for all three classes.

Guidance is the outer loop that wraps around the `Autopilot`. It converts path and altitude
errors into set point commands (target altitude, vertical speed, heading, roll attitude) that
are fed to `Autopilot::step()`. Guidance classes live in the Domain Layer and have no I/O.
Each is a stateful element (contains filter state) and implements `reset()`, `step()`, and
JSON + proto serialization.

### `PathGuidance` — Lateral Path Tracking

Implements a nonlinear guidance law (L1 or similar) that commands a target heading or roll
attitude set point to null cross-track error against a path segment.

### `VerticalGuidance` — Altitude / Climb-Rate Hold

Commands target altitude or vertical speed set points to track a target altitude or climb
rate profile.

### `ParkTracking` — Loiter / Station-Keep

Commands the aircraft to orbit a fixed ground point at a specified radius and altitude by
producing heading and altitude set point commands to `Autopilot`.

### Tests — `PathGuidance`

- With zero cross-track error and correct heading, commanded heading set point matches
  current heading (no corrective input).
- With a large cross-track error, the commanded heading correction is bounded within a
  specified maximum bank angle equivalent.

### Tests — `VerticalGuidance`

- With aircraft at target altitude, commanded altitude set point matches current altitude
  (no corrective input).
- With aircraft below target altitude, commanded vertical speed set point is positive.

### Tests — Serialization

- JSON and proto round-trips preserve filter state; next `step()` output matches between
  original and restored instances.

### CMake

Add guidance source files to `liteaerosim`.
Add `test/Guidance_test.cpp` to the test executable.

---

## 7. Plot Visualization — Python Post-Processing Tools

Python scripts to load logger output and produce time-series plots for simulation
post-flight analysis. These are Application Layer tools and live under `python/tools/`.

### Deliverables

- `python/tools/plot_flight.py`: CLI script that reads a `.csv` log file produced by
  `Logger` and plots a configurable set of channels vs. time. Outputs a `.png` or
  displays interactively.
- Channel groups: kinematics (position, velocity, attitude), aerodynamics (α, β, CL, CD),
  propulsion (thrust, throttle), guidance (cross-track error, altitude error).

### Tests

- `python/test/test_plot_flight.py`: verifies that `load_log(path)` returns a DataFrame
  with the expected columns and correct dtype; does not test rendering (matplotlib is
  not invoked in the test suite).

### Python dependencies

Add to `python/pyproject.toml`:

```toml
[dependency-groups]
dev = [
    ...,
    "matplotlib>=3.8",
    "pandas>=2.0",
]
```

---

## 8. Manual Input — Joystick and Keyboard

Manual input adapters translate human control inputs (joystick axes, keyboard state) into
an `AircraftCommand`. These live in the Interface Layer and have no physics logic.

### Deliverables

- `KeyboardInput`: maps configurable key bindings to throttle, roll, pitch, yaw increments.
  Operates at the simulation timestep rate.
- `JoystickInput`: reads a raw joystick device (SDL2 or platform API) and maps axes and
  buttons to the `AircraftCommand` fields. Applies a configurable dead zone and axis scaling.

### Interface sketch

```cpp
// include/input/ManualInput.hpp
namespace liteaerosim::input {

class V_ManualInput {
public:
    virtual AircraftCommand read() = 0;   // non-blocking; returns latest command
    virtual ~V_ManualInput() = default;
};

} // namespace liteaerosim::input
```

### Tests

- `KeyboardInput` with no keys pressed returns zero lateral/directional commands and
  configured idle throttle.
- `JoystickInput` axis value at dead-zone boundary maps to zero output.
- Axis scaling: full-deflection axis value maps to the configured maximum command.

### CMake

Add `src/input/KeyboardInput.cpp` and `src/input/JoystickInput.cpp` to `liteaerosim`.
Add a platform-conditional dependency on SDL2 for `JoystickInput`.

---

## 9. Execution Modes — Real-Time, Scaled, and Batch Runners

The simulation runner controls the wall-clock relationship to simulation time. Three modes
are required:

| Mode | Description |
|------|-------------|
| **Real-time** | Each `step()` is paced to its real elapsed wall time (`dt_s` per step). |
| **Scaled real-time** | Same as real-time but with a configurable speed multiplier (0.5×, 2×, etc.). |
| **Full-rate batch** | Steps run as fast as possible; no sleep; used for CI, Monte Carlo, and data generation. |

### Interface sketch

```cpp
// include/runner/SimRunner.hpp
namespace liteaerosim::runner {

enum class ExecutionMode { RealTime, ScaledRealTime, Batch };

struct RunnerConfig {
    ExecutionMode mode         = ExecutionMode::Batch;
    float         time_scale   = 1.0f;   // used only in ScaledRealTime mode
    double        duration_s   = 0.0;    // 0 = run until stop() is called
};

class SimRunner {
public:
    void initialize(const RunnerConfig& config, Aircraft& aircraft);
    void start();    // begins the run loop (blocks in Batch mode; spawns thread otherwise)
    void stop();
    bool is_running() const;
    double elapsed_sim_time_s() const;
};

} // namespace liteaerosim::runner
```

### Tests

- `Batch` mode with `duration_s = 1.0` and `dt_s = 0.01` calls `Aircraft::step()` exactly
  100 times and then stops.
- `RealTime` mode: elapsed wall time after 10 steps is within 10% of `10 * dt_s`.
- `stop()` called from outside (threaded test) terminates the run loop within one timestep.
- `elapsed_sim_time_s()` returns `n_steps * dt_s` after `n_steps` have been executed.

### CMake

Add `src/runner/SimRunner.cpp` to `liteaerosim`.
Add `test/SimRunner_test.cpp` to the test executable.

---

## 10. Remaining Sensor Models

Not blocking any higher-priority item. Stub headers exist in `include/sensor/`.
Implement when needed; order within this group follows dependency.

| Class | Depends on | Hardware modeled |
|---|---|---|
| `SensorMag` | `KinematicState` (done) | Triaxial magnetometer — body-frame field with hard-iron bias and soft-iron distortion |
| `SensorGnss` | `KinematicState` (done) | GNSS receiver — WGS84 position, NED velocity, SOG/COG, fix type, DOP |
| `SensorLaserAlt` | Terrain (done) | Laser altimeter — single-beam slant range and AGL altitude |
| `SensorRadAlt` | Terrain (done) | Radar altimeter — HAG from `Terrain::heightAboveGround_m` with noise and range saturation |
| `SensorInsSimulation` | `KinematicState` (done), `NavigationFilter` types | INS simulation replacement — truth-plus-error model producing `InsMeasurement` |
| `SensorForwardTerrainProfile` | Terrain (done), Guidance | Forward terrain profiling sensor (multi-beam LIDAR / line-scan radar) for terrain-following guidance |
| `SensorAA` | `KinematicState` (done) | Passive angle/angle sensor (imaging type) — measures two LOS angles, no range |
| `SensorAAR` | `KinematicState` (done) | Active angle/angle/range sensor (e.g. radar) — measures two LOS angles plus slant range |
| `SensorTrackEstimator` | `SensorAA` or `SensorAAR` | Kinematic track estimator for a moving object observed via angle or angle/range measurements |

---

## 11. Estimation Subsystem

Flight code estimation algorithms. Stub headers exist in `include/estimation/` (to be
created). Each derives from `DynamicElement` directly. Design authorities listed below.

| Class | Depends on | Design authority |
|---|---|---|
| `NavigationFilter` | `SensorGnss`, `SensorAirData`, `SensorMag` | [navigation_filter.md](../architecture/navigation_filter.md) |
| `WindEstimator` | `NavigationFilter` or `SensorInsSimulation`, `SensorAirData` | [wind_estimator.md](../architecture/wind_estimator.md) |
| `FlowAnglesEstimator` | `WindEstimator`, `SensorAirData` | [flow_angles_estimator.md](../architecture/flow_angles_estimator.md) |
