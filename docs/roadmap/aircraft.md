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
|-----------|------|--------|
| `KinematicState` | `include/KinematicState.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `LiftCurveModel` | `include/aerodynamics/LiftCurveModel.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `LoadFactorAllocator` | `include/aerodynamics/LoadFactorAllocator.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `WGS84_Datum` | `include/navigation/WGS84.hpp` | ✅ Implemented |
| `AeroPerformance` | `include/aerodynamics/AeroPerformance.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `AirframePerformance` | `include/airframe/AirframePerformance.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `Inertia` | `include/airframe/Inertia.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `V_Propulsion` | `include/propulsion/V_Propulsion.hpp` | ✅ Implemented — see [propulsion.md](../architecture/propulsion.md) |
| `PropulsionJet` | `include/propulsion/PropulsionJet.hpp` | ✅ Implemented + serialization (JSON + proto) — see [propulsion.md](../architecture/propulsion.md) |
| `PropulsionEDF` | `include/propulsion/PropulsionEDF.hpp` | ✅ Implemented + serialization (JSON + proto) — see [propulsion.md](../architecture/propulsion.md) |
| `PropellerAero` | `include/propulsion/PropellerAero.hpp` | ✅ Implemented — see [propulsion.md](../architecture/propulsion.md) |
| `V_Motor` | `include/propulsion/V_Motor.hpp` | ✅ Implemented — see [propulsion.md](../architecture/propulsion.md) |
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

---

## Delivered

Design authority for all delivered items: [`docs/architecture/aircraft.md`](../architecture/aircraft.md).

| # | Item | Tests |
|---|------|-------|
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

---

## 1. `AeroCoeffEstimator` — Geometry-to-Coefficient Derivation

Design authority: [`docs/algorithms/aerodynamics.md`](../algorithms/aerodynamics.md).

`AeroCoeffEstimator` is a stateless utility class that accepts a compact aircraft geometry
description and returns a fully populated `AeroPerformance` and `LiftCurveModel`
configuration. It implements the derivation chain in `aerodynamics.md` Parts 1–8. No I/O;
no units other than SI.

This item also adds the four proposed fields to `AeroPerformance`:
`cl_q_nd`, `mac_m`, `cy_r_nd`, `fin_arm_m`.

### Input (`AircraftGeometry`)

```cpp
// include/aerodynamics/AircraftGeometry.hpp
namespace liteaerosim::aerodynamics {

struct SurfaceGeometry {
    float span_m;               // tip-to-tip (or root-to-tip for vertical tail)
    float area_m2;              // reference area
    float le_sweep_rad;         // leading-edge sweep
    float taper_ratio_nd;       // c_tip / c_root
    float x_le_root_m;          // body x of root leading edge
};

struct AircraftGeometry {
    SurfaceGeometry wing;
    SurfaceGeometry h_tail;
    SurfaceGeometry v_tail;
    float fuselage_length_m;
    float fuselage_diameter_m;
    float thickness_ratio_nd;           // wing t/c
    float section_cl_alpha_rad;         // 2D lift slope (≈ 2π)
    float section_cl_max_2d_nd;         // 2D section Cl_max
    float x_cg_m;                       // body x of center of gravity
    float mach_nd           = 0.f;      // design Mach (0 → incompressible)
    float tail_efficiency_nd = 0.9f;    // η_t and η_v
    float cd_misc_nd        = 0.003f;   // miscellaneous drag increment
};

} // namespace liteaerosim::aerodynamics
```

### Output

`estimate()` returns a `std::pair<AeroPerformance, LiftCurveParams>` populated from the
derivation chain in `aerodynamics.md` §§1–8.

### Tests

- **AR**: computed `ar` matches $b^2 / S$ for the test wing.
- **MAC**: computed `mac_m` matches the closed-form $\bar{c} = \tfrac{2}{3} c_\text{root} (1 + \lambda + \lambda^2)/(1 + \lambda)$.
- **$C_{L_\alpha}$**: unswept wing at $M = 0$ matches $2\pi A\!\!R / (A\!\!R + 2)$ to within 0.5%.
- **$C_{L_\text{max}}$**: equals $C_{l_{\max,2D}} \cos\Lambda_{c/4}$ for a swept wing.
- **$e$**: Hoerner value for $A\!\!R = 8$, $\Lambda_{c/4} = 0$ matches $1/(1 + 0.007\pi \cdot 8)$ to within 0.1%.
- **$C_{D_0}$**: component-buildup result for a known configuration is within 10% of a published reference value.
- **$C_{Y_\beta}$**: negative (stabilizing); magnitude increases with $S_v / S$.
- **$C_{L_q}$**: tail-dominated configuration gives value in the range $[3, 12]$ rad⁻¹.
- **$C_{Y_r}$**: positive; increases with $l_v$.
- **Round-trip**: `estimate()` applied to a reference UAV geometry produces an `AeroPerformance` that passes `AeroPerformance`'s existing JSON round-trip test.

### CMake

Add `src/aerodynamics/AeroCoeffEstimator.cpp` to `liteaerosim`.
Add `test/AeroCoeffEstimator_test.cpp` to the test executable.

---

## 2. `Terrain` — Elevation Model

`Terrain` provides ground elevation (meters above mean sea level) at a given latitude and
longitude. The Domain Layer uses it to compute height above ground (HAG) and to detect
ground contact. An initial implementation may use a flat Earth at constant elevation.

### Interface sketch

```cpp
// include/environment/Terrain.hpp
namespace liteaerosim::environment {

class Terrain {
public:
    // Returns ground elevation (m ASL) at the given geodetic position.
    virtual float elevation_m(float latitude_rad, float longitude_rad) const = 0;

    // Returns height above ground (m) given aircraft altitude (m ASL).
    float heightAboveGround_m(float altitude_m, float latitude_rad, float longitude_rad) const;

    virtual ~Terrain() = default;
};

class FlatTerrain : public Terrain {
public:
    explicit FlatTerrain(float elevation_m = 0.f);
    float elevation_m(float latitude_rad, float longitude_rad) const override;
};

} // namespace liteaerosim::environment
```

### Tests

- `FlatTerrain(300)`: `elevation_m()` returns 300 regardless of lat/lon.
- `heightAboveGround_m(500, ...)` with `FlatTerrain(300)` returns 200.
- `heightAboveGround_m` returns 0 (not negative) when aircraft is at terrain elevation.

### CMake

Add `src/environment/Terrain.cpp` to `liteaerosim`.
Add `test/Terrain_test.cpp` to the test executable.

---

## 3. Air Data Sensors — `SensorAirData`, `SensorAA`, `SensorAAR`

Air data sensors derive indicated and calibrated quantities from the true atmospheric state
and aircraft kinematics. They model systematic bias and measurement noise. All sensors
consume the `KinematicState` and the `AtmosphericState` output of `Atmosphere`.

### `SensorAirData` — Pitot-Static System

Outputs: indicated airspeed (IAS), calibrated airspeed (CAS), equivalent airspeed (EAS),
true airspeed (TAS), barometric altitude, outside air temperature.

```cpp
// include/sensor/SensorAirData.hpp
struct AirDataMeasurement {
    float ias_mps;         // indicated airspeed (m/s)
    float cas_mps;         // calibrated airspeed (m/s)
    float eas_mps;         // equivalent airspeed (m/s)
    float tas_mps;         // true airspeed (m/s)
    float baro_altitude_m; // barometric altitude (m)
    float oat_k;           // outside air temperature (K)
};
```

### `SensorAA` — Angle-of-Attack Vane / `SensorAAR` — Sideslip Vane

Each outputs a scalar angle in radians. Modeled with a first-order lag and additive bias.

### Tests — `SensorAirData`

- At sea level ISA and `tas = 20 m/s`, IAS ≈ CAS ≈ EAS ≈ TAS to within 0.1 m/s.
- At altitude (3000 m) and same TAS, IAS < TAS.
- `baro_altitude_m` matches geometric altitude to within 10 m at ISA conditions.

### Tests — `SensorAA` / `SensorAAR`

- With zero lag (τ = 0), output equals the true angle immediately.
- With finite lag, output converges toward the true angle with the correct time constant.
- Bias shifts the mean output by the configured amount.

### CMake

Add sensor source files to `liteaerosim`.
Add `test/SensorAirData_test.cpp` and `test/SensorAngle_test.cpp` to the test executable.

---

## 4. `SensorRadAlt` — Radar / Laser Altimeter

`SensorRadAlt` outputs height above ground (HAG) derived from `Terrain::heightAboveGround_m`.
`SensorForwardTerrainProfile` returns a look-ahead terrain elevation vector along the
aircraft's projected track — used by terrain-following guidance.

### Tests — `SensorRadAlt`

- Over `FlatTerrain(0)` at 100 m altitude, output = 100 m (within noise bounds).
- Output is 0 when the aircraft is on the ground.
- When HAG exceeds the sensor's maximum range, output is clamped/saturated to `max_range_m`.

### Tests — `SensorForwardTerrainProfile`

- Over flat terrain, all profile samples equal the terrain elevation at the current position.
- Profile length and sample spacing match configuration parameters.

### CMake

Add `src/sensor/SensorRadAlt.cpp` and `src/sensor/SensorForwardTerrainProfile.cpp` to `liteaerosim`.
Add `test/SensorRadAlt_test.cpp` to the test executable.

---

## 5. Path Representation — `V_PathSegment`, `PathSegmentHelix`, `Path`

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

Guidance laws convert path and altitude errors into commanded load factors for `Aircraft::step()`.
They live in the Domain Layer and have no I/O. Each is a stateful element (contains filter
state) and implements `reset()`, `step()`, and JSON + proto serialization.

### `PathGuidance` — Lateral Path Tracking

Implements a nonlinear guidance law (L1 or similar) that commands lateral load factor `n_y`
to null cross-track error.

### `VerticalGuidance` — Altitude / Climb-Rate Hold

Commands normal load factor `n` to track a target altitude or climb rate profile.

### `ParkTracking` — Loiter / Station-Keep

Commands the aircraft to orbit a fixed ground point at a specified radius and altitude.

### Tests — `PathGuidance`

- With zero cross-track error and correct heading, commanded `n_y` is near zero.
- With a large cross-track error, `n_y` is bounded within `[-n_y_max, +n_y_max]`.

### Tests — `VerticalGuidance`

- With aircraft at target altitude, commanded `n` converges to `1.0 g`.
- With aircraft below target altitude, `n > 1.0 g`.

### Tests — Serialization

- JSON and proto round-trips preserve filter state; next `step()` output matches between
  original and restored instances.

### CMake

Add guidance source files to `liteaerosim`.
Add `test/Guidance_test.cpp` to the test executable.

---

## 7. Autopilot — Outer Loop Command Generation

`Autopilot` combines `PathGuidance`, `VerticalGuidance`, and `ParkTracking` into a single
class that consumes the `KinematicState` and `PathResponse` and produces an `AircraftCommand`
for `Aircraft::step()`. It also manages mode selection (path-following vs. loiter vs. manual
override).

### Interface sketch

```cpp
// include/control/Autopilot.hpp
namespace liteaerosim::control {

enum class AutopilotMode { PathFollow, Loiter, ManualOverride };

class Autopilot {
public:
    AircraftCommand step(const KinematicState& state,
                         const path::PathResponse& path_resp,
                         const Eigen::Vector3f& wind_NED_mps,
                         float rho_kgm3,
                         float throttle_nd);

    void setMode(AutopilotMode mode);
    AutopilotMode mode() const;

    void reset();

    nlohmann::json       serializeJson() const;
    void                 deserializeJson(const nlohmann::json& j);
    std::vector<uint8_t> serializeProto() const;
    void                 deserializeProto(const std::vector<uint8_t>& bytes);
};

} // namespace liteaerosim::control
```

### Tests

- In `PathFollow` mode with zero cross-track error and target altitude, `step()` produces
  `n ≈ 1.0` and `n_y ≈ 0.0`.
- Switching to `ManualOverride` causes `step()` to pass through the manual command unchanged.
- JSON and proto round-trips preserve inner guidance filter states.

### CMake

Add `src/control/Autopilot.cpp` to `liteaerosim`.
Add `test/Autopilot_test.cpp` to the test executable.

---

## 8. Plot Visualization — Python Post-Processing Tools

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

## 9. Manual Input — Joystick and Keyboard

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

## 10. Execution Modes — Real-Time, Scaled, and Batch Runners

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
