# Aircraft Class — Recommended Next Steps

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
| `AirframePerformance` | `include/airframe/AirframePerformance.hpp` | ⚠️ Plain struct — no serialization |
| `Inertia` | `include/airframe/Inertia.hpp` | ⚠️ Plain struct — no serialization |
| `V_Propulsion` | `include/propulsion/V_Propulsion.hpp` | ✅ Implemented — see [propulsion.md](../architecture/propulsion.md) |
| `PropulsionJet` | `include/propulsion/PropulsionJet.hpp` | ✅ Implemented + serialization (JSON + proto) — see [propulsion.md](../architecture/propulsion.md) |
| `PropulsionEDF` | `include/propulsion/PropulsionEDF.hpp` | ✅ Implemented + serialization (JSON + proto) — see [propulsion.md](../architecture/propulsion.md) |
| `PropellerAero` | `include/propulsion/PropellerAero.hpp` | ✅ Implemented — see [propulsion.md](../architecture/propulsion.md) |
| `V_Motor` | `include/propulsion/V_Motor.hpp` | ✅ Implemented — see [propulsion.md](../architecture/propulsion.md) |
| `MotorElectric` | `include/propulsion/MotorElectric.hpp` | ✅ Implemented — see [propulsion.md](../architecture/propulsion.md) |
| `MotorPiston` | `include/propulsion/MotorPiston.hpp` | ✅ Implemented — see [propulsion.md](../architecture/propulsion.md) |
| `PropulsionProp` | `include/propulsion/PropulsionProp.hpp` | ✅ Implemented + serialization (JSON + proto) — see [propulsion.md](../architecture/propulsion.md) |
| `Aircraft` | `include/Aircraft.hpp` | ❌ Stub comment only |

---

## 1. `AirframePerformance` — Field Cleanup, Serialization, and Envelope Integration

Design authority: [`docs/architecture/aircraft.md`](../architecture/aircraft.md).

`AirframePerformance` defines the structural and operational hard limits of the airframe.
It is a config-driven value struct owned by `Aircraft` as a value member. Its primary role
is **envelope protection**: `Aircraft::step()` clamps the commanded normal and lateral load
factors to `[g_min_nd, g_max_nd]` before calling `LoadFactorAllocator::solve()`, preventing
the physics loop from commanding structurally impossible maneuvers.

### Required changes

Current field names violate the naming convention (no unit suffixes). Renames required:

| Current name | Renamed to | Unit |
|---|---|---|
| `GMax` | `g_max_nd` | dimensionless (g) |
| `GMin` | `g_min_nd` | dimensionless (g) |
| `TASMax` | `tas_max_mps` | m/s |
| `MachMax` | `mach_max_nd` | dimensionless |

Also add `#pragma once` and move into `namespace liteaerosim`.

### Serialization

`AirframePerformance` must support full reconstitution from a frozen state — add
`serializeJson()` / `deserializeJson()` and `serializeProto()` / `deserializeProto()`
that capture all four config fields. These are included in the `Aircraft` state snapshot
so that `Aircraft::deserializeJson()` is self-contained and does not require a prior
`initialize()` call.

### Schema extension

Add an `"airframe"` section to `aircraft_config_v1`:

```json
"airframe": {
    "g_max_nd":    3.8,
    "g_min_nd":   -1.5,
    "tas_max_mps": 82.3,
    "mach_max_nd": 0.25
}
```

Update `python/tools/validate_aircraft_config.py` and its tests:
- `g_min_nd < 0` (structural negative limit must be negative)
- `g_max_nd > 0`
- `tas_max_mps > 0`
- `mach_max_nd > 0`

### Integration with `Aircraft::step()`

After computing dynamic pressure and before calling `LoadFactorAllocator::solve()`,
clamp the commanded load factors (new step 3 in the physics loop):

```
n_cmd   = clamp(cmd.n,   _airframe.g_min_nd, _airframe.g_max_nd)
n_y_cmd = clamp(cmd.n_y, _airframe.g_min_nd, _airframe.g_max_nd)
```

Pass `n_cmd` and `n_y_cmd` to `LoadFactorAllocator::solve()` instead of the raw commands.

### Tests

- JSON round-trip: `deserializeJson(serializeJson())` restores all four fields exactly.
- Proto round-trip: same.
- Schema version mismatch throws `std::runtime_error`.
- `g_min_nd >= 0` fails validator; `g_max_nd <= 0` fails validator.
- `tas_max_mps <= 0` fails validator.
- In `Aircraft::step()`: commanding `n = 10.0` with `g_max_nd = 3.8` clamps to `3.8`.

---

## 2. `Inertia` — Field Cleanup, Serialization, and `Aircraft` Integration

Design authority: [`docs/architecture/aircraft.md`](../architecture/aircraft.md).

`Inertia` collects all mass properties of the airframe into a single serializable struct.
Its `mass_kg` field replaces the bare `float _mass_kg` scalar planned on `Aircraft`,
keeping all mass properties co-located. The moment-of-inertia fields (`Ixx_kgm2`,
`Iyy_kgm2`, `Izz_kgm2`) are read from config and currently unused in the point-mass model
— they are reserved for the 6-DOF angular dynamics (moment equations) when those are added.

### Required changes

The existing field names already follow the convention. Add `#pragma once` and move into
`namespace liteaerosim`. Add a default constructor that zero-initializes all fields
(`mass_kg = 0` is invalid; `Aircraft::initialize()` throws if `mass_kg <= 0`).

### Serialization

`Inertia` must support full reconstitution from a frozen state — add `serializeJson()` /
`deserializeJson()` and `serializeProto()` / `deserializeProto()` that capture all four
fields. These are included in the `Aircraft` state snapshot so that
`Aircraft::deserializeJson()` is self-contained and does not require a prior
`initialize()` call.

### Schema extension

Add an `"inertia"` section to `aircraft_config_v1` and remove `aircraft.mass_kg`
(subsumed by `inertia.mass_kg`):

```json
"inertia": {
    "mass_kg":   1045.0,
    "Ixx_kgm2":  1285.0,
    "Iyy_kgm2":  1825.0,
    "Izz_kgm2":  2667.0
}
```

Update `python/tools/validate_aircraft_config.py` and its tests:
- `mass_kg > 0`
- `Ixx_kgm2 > 0`, `Iyy_kgm2 > 0`, `Izz_kgm2 > 0`
- Remove the existing `mass_kg` check from the `aircraft` section.

### Integration with `Aircraft`

Replace `float _mass_kg` with `Inertia _inertia` as a value member of `Aircraft`. All
references to `_mass_kg` inside `Aircraft::step()` and serialization methods become
`_inertia.mass_kg`.

### Tests

- JSON round-trip: all four fields restored exactly.
- Proto round-trip: same.
- Schema version mismatch throws `std::runtime_error`.
- `mass_kg <= 0` fails validator; any moment-of-inertia `<= 0` fails validator.
- `Aircraft::initialize()` with `mass_kg = 0` throws `std::invalid_argument`.

---

## 3. `Aircraft` Class Definition

Design authority: [`docs/architecture/aircraft.md`](../architecture/aircraft.md).

`Aircraft` owns and orchestrates the full physics update loop. It is not a `DynamicBlock`
(the interface is multi-input, multi-output), but it follows the project's lifecycle
convention (`initialize` → `reset` → `step` → `serialize` / `deserialize`).

### Ownership model

| Member | Type | Owned by |
|--------|------|----------|
| `_state` | `KinematicState` | Aircraft (value member) |
| `_liftCurve` | `LiftCurveModel` | Aircraft (value member) |
| `_allocator` | `LoadFactorAllocator` | Aircraft (value member) |
| `_aeroPerf` | `AeroPerformance` | Aircraft (value member) |
| `_propulsion` | `std::unique_ptr<V_Propulsion>` | Aircraft (owns, injected at construction) |
| `_mass_kg` | `float` | Aircraft |

### Inputs to `step()`

```cpp
struct AircraftCommand {
    float n;              // commanded normal load factor (g)
    float n_y;            // commanded lateral load factor (g)
    float n_dot;          // rate of change of n (1/s) — for alphaDot
    float n_y_dot;        // rate of change of n_y (1/s) — for betaDot
    float rollRate_Wind_rps; // commanded wind-frame roll rate (rad/s)
    float throttle_nd;    // normalized throttle [0, 1]
};
```

### Class interface

```cpp
// include/Aircraft.hpp
namespace liteaerosim {

class Aircraft {
public:
    explicit Aircraft(std::unique_ptr<propulsion::V_Propulsion> propulsion);

    void initialize(const nlohmann::json& config);
    void reset();

    // Advance the aircraft physics by one timestep.
    //   time_sec      — absolute simulation time (s)
    //   cmd           — autopilot-commanded inputs
    //   wind_NED_mps  — ambient wind vector in NED frame (m/s)
    //   rho_kgm3      — local air density (kg/m³)
    void step(double time_sec,
              const AircraftCommand& cmd,
              const Eigen::Vector3f& wind_NED_mps,
              float rho_kgm3);

    const KinematicState& state() const { return _state; }

    nlohmann::json serialize() const;
    void deserialize(const nlohmann::json& snapshot);

private:
    KinematicState                         _state;
    aerodynamics::LiftCurveModel           _liftCurve;
    aerodynamics::LoadFactorAllocator      _allocator;
    aerodynamics::AeroPerformance          _aeroPerf;
    std::unique_ptr<propulsion::V_Propulsion> _propulsion;
    float _mass_kg = 0.f;
    float _dt_s    = 0.f;
};

} // namespace liteaerosim
```

### Tests

- `Aircraft` constructs without throwing given a valid `PropulsionJet` and a representative config.
- `state()` returns the initial `KinematicState` before any `step()` call.

---

## 4. `Aircraft::step()` — Physics Integration Loop

Design authority: [`docs/architecture/aircraft.md — Physics Integration Loop`](../architecture/aircraft.md#physics-integration-loop).



The `step()` method is the closed-loop physics update. It must execute in this order:

```
1. Compute true airspeed from KinematicState and wind:
       V_air = (velocity_NED - wind_NED).norm()

2. Compute dynamic pressure:
       q_inf = 0.5 * rho * V_air²

3. Solve for α, β via LoadFactorAllocator:
       LoadFactorInputs in = { cmd.n, cmd.n_y, q_inf, thrust_n (previous step),
                               _mass_kg, cmd.n_dot, cmd.n_y_dot }
       LoadFactorOutputs out = _allocator.solve(in)

4. Evaluate lift coefficient:
       CL = _liftCurve.evaluate(out.alpha_rad)

5. Compute aerodynamic forces in Wind frame:
       AeroForces F = _aeroPerf.compute(out.alpha_rad, out.beta_rad, q_inf, CL)

6. Advance propulsion:
       float T = _propulsion->step(cmd.throttle_nd, V_air, rho_kgm3)

7. Compute net Wind-frame acceleration:
       // Thrust decomposition in Wind frame (see docs/algorithms/equations_of_motion.md)
       ax = (T * cosα * cosβ  + F.x_n) / m     // F.x_n is negative (drag)
       ay = (−T * cosα * sinβ + F.y_n) / m
       az = (−T * sinα        + F.z_n) / m     // F.z_n is negative (lift up)
       // Gravity is already embedded in the load factor — do not add separately

8. Advance KinematicState:
       _state.step(time_sec,
                   {ax, ay, az},           // acceleration_Wind_mps
                   cmd.rollRate_Wind_rps,
                   out.alpha_rad, out.beta_rad,
                   out.alphaDot_rps, out.betaDot_rps,
                   wind_NED_mps);
```

**Note on gravity:** The `LoadFactorAllocator` constraint `q·S·CL + T·sinα = n·m·g` already
encodes the gravitational load. The Wind-frame acceleration computed here is the kinematic
acceleration only — gravity must not be double-counted. Verify this against `KinematicState`
velocity integration before finalizing.

### Tests

- Straight-and-level at `n = 1`: speed is approximately constant over several steps.
- `throttle_nd = 0`, `n = 1`: aircraft decelerates (drag exceeds thrust).
- After `reset()`, position and velocity return to initial values.
- `step()` does not throw for any physically plausible command.

---

## 5. Serialization

Design authority: [`docs/architecture/aircraft.md — Serialization Design`](../architecture/aircraft.md#serialization-design).

`Aircraft` must implement both JSON and binary (protobuf) serialization, capturing the full
restart state of every owned subcomponent with warm-start state. The methods follow the
non-`DynamicBlock` pattern (plain public members, not NVI):

```cpp
nlohmann::json       serializeJson()                              const;
void                 deserializeJson(const nlohmann::json&        j);

std::vector<uint8_t> serializeProto()                            const;
void                 deserializeProto(const std::vector<uint8_t>& bytes);
```

### Serialization contract

Every simulation element serializes its **full configuration and internal state** sufficient
for exact reconstitution from a frozen snapshot. `Aircraft::deserializeJson()` must be
self-contained — a caller must not need to call `initialize()` before or after
`deserializeJson()` to obtain a fully operational `Aircraft`.

### JSON schema

```json
{
    "schema_version": 1,
    "type": "Aircraft",
    "params": { "dt_s": 0.01 },
    "kinematic_state":   { ... },    // KinematicState — position, velocity, attitude
    "allocator":         { ... },    // LoadFactorAllocator — config + warm-start α, β
    "lift_curve":        { ... },    // LiftCurveModel — config params (stateless)
    "aero_performance":  { ... },    // AeroPerformance — config params (stateless)
    "airframe":          { ... },    // AirframePerformance — config params (stateless)
    "inertia":           { ... },    // Inertia — config params (stateless)
    "propulsion":        { ... }     // V_Propulsion subclass — config + filter state
}
```

Stateless components (`LiftCurveModel`, `AeroPerformance`, `AirframePerformance`, `Inertia`)
serialize their config parameters only — they have no internal state beyond what is set at
construction. Stateful components additionally serialize their warm-start state.

### Proto message

Add an `Aircraft` message to `proto/liteaerosim.proto` that embeds the existing
`KinematicState` and `LoadFactorAllocatorState` messages, plus messages for each config
struct and the propulsion `oneof`.

### Tests

- JSON round-trip: `deserializeJson(serializeJson())` fully reconstitutes the `Aircraft`
  without a prior `initialize()` call; next `step()` output is identical to the original.
- Proto round-trip: same verification via `deserializeProto(serializeProto())`.
- Schema version mismatch on either deserialize throws `std::runtime_error`.
- Round-trip after 100 steps preserves all subsystem state exactly.

---

## 6. JSON Initialization

Design authority: [`docs/architecture/aircraft.md — Initialization`](../architecture/aircraft.md#initialization--json-config-mapping).

The JSON parameter schema is complete (see [`docs/schemas/aircraft_config_v1.md`](../schemas/aircraft_config_v1.md)).
`Aircraft::initialize(config)` must read from a validated config and construct all owned
subcomponents.

### Mapping from schema to Aircraft members

| JSON path | Aircraft member |
|-----------|-----------------|
| `aircraft.mass_kg` | `_mass_kg` |
| `aircraft.S_ref_m2` | `AeroPerformance`, `LoadFactorAllocator` |
| `aircraft.cl_y_beta` | `AeroPerformance`, `LoadFactorAllocator` |
| `lift_curve.*` | `LiftCurveParams` → `LiftCurveModel` |
| `initial_state.*` | `KinematicState` constructor |

`validate_aircraft_config.py` must pass before `initialize()` is called.  `initialize()`
may throw `std::invalid_argument` on malformed input but is not required to duplicate the
full Python-side validation.

### Tests

- `initialize()` with each of the three fixture files (`test/data/aircraft/general_aviation.json`,
  `test/data/aircraft/jet_trainer.json`, `test/data/aircraft/small_uas.json`) succeeds
  without throwing.
- `initialize()` with a missing required field throws `std::exception`.

---

## 7. `Logger` — Telemetry Serialization

`Logger` records simulation state at each timestep to a binary or structured-text file for
post-flight analysis. It lives in the Infrastructure layer and has no physics logic.

### Responsibilities

- Accept arbitrary key-value telemetry records (string key, scalar float value, timestep index).
- Buffer writes and flush to disk efficiently (not every timestep).
- Write a header that describes the channel layout so a reader can decode the file without
  prior knowledge of the recording configuration.
- Support at minimum one output format: line-delimited CSV with a header row.
- Provide a `Reader` counterpart (Python or C++) that loads a logged file back into a
  structured table indexed by channel name and timestep.

### Interface sketch

```cpp
// include/logger/Logger.hpp
namespace liteaerosim::logger {

class Logger {
public:
    // Open a new log file.  Throws std::runtime_error on I/O failure.
    void open(const std::filesystem::path& path, const std::vector<std::string>& channels);
    void log(double time_s, const std::vector<float>& values);  // values.size() == channels.size()
    void close();
    bool is_open() const;
};

} // namespace liteaerosim::logger
```

### Tests

- `open()` creates a file; `close()` flushes and closes it without throwing.
- After `close()`, re-opening and reading the file back recovers the exact values written
  by `log()` for all channels and all timesteps.
- `log()` called after `close()` throws `std::logic_error`.
- `open()` with mismatched `values.size()` on the first `log()` call throws.
- Logger survives 10 000 consecutive `log()` calls without memory growth (smoke test).

### CMake

Add `src/logger/Logger.cpp` to the `liteaerosim` target.
Add `test/Logger_test.cpp` to the test executable.

---

## 8. `Atmosphere` — International Standard Atmosphere Model

`Atmosphere` provides density, temperature, and pressure as functions of geopotential
altitude. It is a stateless value-type (no `step()`, no serialization). All dependent
subsystems receive density as a parameter rather than holding a reference to `Atmosphere`.

### Interface sketch

```cpp
// include/environment/Atmosphere.hpp
namespace liteaerosim::environment {

struct AtmosphericState {
    float temperature_k;    // static temperature (K)
    float pressure_pa;      // static pressure (Pa)
    float density_kgm3;     // air density (kg/m³)
    float speed_of_sound_mps;
};

class Atmosphere {
public:
    // Returns ISA state at the given geopotential altitude.
    static AtmosphericState isa(float altitude_m);

    // Returns density ratio σ = rho / rho_SL.
    static float densityRatio(float altitude_m);
};

} // namespace liteaerosim::environment
```

### Tests

- At 0 m: temperature = 288.15 K, pressure = 101 325 Pa, density = 1.225 kg/m³ (within 0.01%).
- At 11 000 m (tropopause): temperature = 216.65 K (within 0.1%).
- `densityRatio(0)` = 1.0.
- `densityRatio(5000)` matches the ISA table value to within 0.1%.
- Density is strictly monotonically decreasing up to 20 000 m.

### CMake

Add `src/environment/Atmosphere.cpp` to the `liteaerosim` target.
Add `test/Atmosphere_test.cpp` to the test executable.

---

## 9. `Wind` and `Gust` — Ambient Wind and Turbulence Models

`Wind` provides a spatially and temporally varying wind vector in NED coordinates. `Gust`
provides a transient velocity disturbance (discrete gust or Dryden turbulence). Both are
Domain Layer components that produce a `wind_NED_mps` vector consumed by `Aircraft::step()`.

### `Wind` interface sketch

```cpp
// include/environment/Wind.hpp
namespace liteaerosim::environment {

class Wind {
public:
    // Returns the ambient wind vector in NED frame at the given position and time.
    Eigen::Vector3f wind_NED_mps(const Eigen::Vector3f& position_NED_m, double time_s) const;

    void setConstant(const Eigen::Vector3f& wind_NED_mps);
    // Future: wind field, measured profile, etc.
};

} // namespace liteaerosim::environment
```

### `Gust` interface sketch

```cpp
// include/environment/Gust.hpp
namespace liteaerosim::environment {

// Discrete (1-cosine) gust model per MIL-SPEC-8785C.
class Gust {
public:
    // Configure a discrete gust starting at trigger_time_s.
    void set(float amplitude_mps, float gust_length_m, float airspeed_mps, double trigger_time_s);

    // Returns the instantaneous gust velocity contribution (m/s) in the body axis.
    float step(double time_s, float airspeed_mps);

    void reset();
};

} // namespace liteaerosim::environment
```

### Tests — `Wind`

- `setConstant({5, 0, 0})`: `wind_NED_mps(any_pos, any_time)` returns `{5, 0, 0}`.
- Default-constructed wind returns `{0, 0, 0}`.

### Tests — `Gust`

- Before trigger time, `step()` returns 0.
- At peak (half-gust length), output equals `amplitude_mps`.
- After gust has fully passed (time beyond end of gust), output returns to 0.
- Total impulse (integral of `step()` over gust duration) is consistent with `amplitude_mps * gust_length_m / airspeed_mps` analytically.

### CMake

Add `src/environment/Wind.cpp` and `src/environment/Gust.cpp` to `liteaerosim`.
Add `test/Wind_test.cpp` and `test/Gust_test.cpp` to the test executable.

---

## 10. `Terrain` — Elevation Model

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

## 11. Air Data Sensors — `SensorAirData`, `SensorAA`, `SensorAAR`

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

## 12. `SensorRadAlt` — Radar / Laser Altimeter

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

## 13. Path Representation — `V_PathSegment`, `PathSegmentHelix`, `Path`

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

## 14. Guidance — `PathGuidance`, `VerticalGuidance`, `ParkTracking`

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

## 15. Autopilot — Outer Loop Command Generation

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

## 16. Plot Visualization — Python Post-Processing Tools

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

## 17. Manual Input — Joystick and Keyboard

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

## 18. Execution Modes — Real-Time, Scaled, and Batch Runners

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
