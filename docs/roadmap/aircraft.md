# Aircraft Class — Roadmap

`Aircraft` is the top-level physics model for a single simulated aircraft. It owns the
`KinematicState`, the aerodynamics model, and the propulsion model, and exposes a single
`step()` method that advances all three by one timestep given commanded load factors and
throttle. It lives in the Domain Layer and has no I/O, no display logic, and no unit
conversions.

**Note on system scope.** LiteAero Sim is the simulation plant only. Autopilot, guidance,
path representation, navigation, and gain scheduling are LiteAero Flight components that are
architecturally separate. Their design and implementation items are in the LiteAero Flight
roadmap (`liteaero-flight/docs/roadmap/flight_code.md`; cross-reference at
[flight_code.md](flight_code.md)). Items in this document are LiteAero Sim items only.
The system architecture is complete — see [`docs/architecture/system/future/`](../architecture/system/future/) and the
project roadmap [README.md](README.md) for cross-cutting milestones.

**Item process.** All items follow a documentation-first process:

1. Architecture and design documents are produced and reviewed.
2. Authorization to begin implementation is granted.
3. Implementation follows TDD — a failing test is written before every production code change.
4. All implementation items include simulation scenarios and automated tests sufficient for
   integration testing, demonstration, and investigation of algorithmic alternatives.

---

## Current State

| Component | File | Status |
| ----------- | ------ | -------- |
| `KinematicState` | `include/KinematicState.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `LiftCurveModel` | `include/aerodynamics/LiftCurveModel.hpp` | ✅ Implemented + serialization (JSON + proto); `alphaSep()` / `alphaSepNeg()` accessors added |
| `LoadFactorAllocator` | `include/aerodynamics/LoadFactorAllocator.hpp` | ✅ Implemented + serialization (JSON + proto); alpha-ceiling guard corrected for positive thrust (item 20); branch-continuation predictor with domain guard (item 0a) |
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
| `DynamicElement` | `liteaero-flight/include/liteaero/control/DynamicElement.hpp` | → LiteAero Flight — see [dynamic_element.md](../architecture/dynamic_element.md) |
| `SisoElement` | `liteaero-flight/include/liteaero/control/SisoElement.hpp` | → LiteAero Flight — NVI SISO wrapper over `DynamicElement` |
| `SensorAirData` | `include/sensor/SensorAirData.hpp` | ✅ Implemented + serialization (JSON + proto) |
| `SensorGnss` | `include/sensor/SensorGnss.hpp` | 🔲 Planned — stub not yet created |
| `SensorLaserAlt` | `include/sensor/SensorLaserAlt.hpp` | 🔲 Planned — stub not yet created |
| `SensorMag` | `include/sensor/SensorMag.hpp` | 🔲 Planned — stub not yet created |
| `SensorINS` | `include/sensor/SensorINS.hpp` | 🔲 Stub only |
| `SensorAA` | `include/sensor/SensorAA.hpp` | 🔲 Stub only |
| `SensorAAR` | `include/sensor/SensorAAR.hpp` | 🔲 Stub only |
| `SensorRadAlt` | `include/sensor/SensorRadAlt.hpp` | 🔲 Stub only |
| `SensorForwardTerrainProfile` | `include/sensor/SensorForwardTerrainProfile.hpp` | 🔲 Stub only |
| `SensorTrackEstimator` | `include/sensor/SensorTrackEstimator.hpp` | 🔲 Stub only |
| `WheelUnit` | `include/landing_gear/WheelUnit.hpp` | 🔲 Planned — see [landing_gear.md](../architecture/landing_gear.md) |
| `StrutState` | `include/landing_gear/StrutState.hpp` | 🔲 Planned |
| `ContactForces` | `include/landing_gear/ContactForces.hpp` | 🔲 Planned |
| `V_SurfaceFriction` | `include/landing_gear/V_SurfaceFriction.hpp` | 🔲 Planned |
| `SurfaceFrictionUniform` | `include/landing_gear/SurfaceFrictionUniform.hpp` | 🔲 Planned |
| `LandingGear` | `include/landing_gear/LandingGear.hpp` | 🔲 Planned |

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
| 20 | `LoadFactorAllocator` alpha-ceiling fix — Newton overshoot and fold guards corrected for positive-thrust case; the achievable-Nz ceiling is at $\alpha^*$ (where $f'(\alpha) = qSC_L'(\alpha) + T\cos\alpha = 0$), not at `alphaPeak()`, when $T > 0$; overshoot guard now clamps the proposed Newton step to the CL parabolic domain using `LiftCurveModel::alphaSep()` / `alphaSepNeg()` before checking $f'$, preventing escape into the flat separated plateau where $f' = T\cos\alpha$ stays positive until $\alpha > \pi/2$; bisects to locate $\alpha^*$ when the guard fires; fold guard stays at current iterate rather than snapping to `alphaPeak()`; `LiftCurveModel::alphaSep()` and `alphaSepNeg()` added to public interface; design documentation updated in [`docs/implementation/equations_of_motion.md`](../implementation/equations_of_motion.md) and [`docs/algorithms/equations_of_motion.md`](../algorithms/equations_of_motion.md) | 4 new tests in `LoadFactorAllocator_test.cpp`; 19 tests total; 458 pass, 2 pre-existing `FilterTFTest` failures unchanged |
| 21 | **System architecture definition** — future-state system architecture model covering: originating requirements, use cases (UC-1 through UC-7), element registry (LiteAero Sim, LiteAero Flight, SimulationRunner, External Interface elements), data flow type and instance registries, interface control documents (ICD-8 through ICD-12), architectural decisions (30 recorded), open questions (all pre-design questions resolved; design-phase questions tracked); system boundary between LiteAero Sim simulation plant and LiteAero Flight established; Docker containerization model for SITL verification defined; `liteaero::` namespace structure and CMake target structure decided; repo split plan defined | No code tests — deliverable is the architecture document set under [`docs/architecture/system/future/`](../architecture/system/future/) |
| 22 | `LoadFactorAllocator` branch-continuation predictor — first-order warm-start $\alpha_0 = \alpha_\text{prev} + \delta n_z \cdot mg / f'(\alpha_\text{prev})$ and symmetric $\beta_0$ formula added to `solve()`; predictor is skipped at the stall ceiling ($f' \approx 0$) or when the raw prediction would fall outside $[\alpha_\text{sep\_neg}, \alpha_\text{sep}]$ (domain guard prevents cross-branch jumps on cold-start excess-demand calls); `_n_z_prev` and `_n_y_prev` added as serialized state fields in both JSON (`n_z_prev_nd`, `n_y_prev_nd`) and proto (`LoadFactorAllocatorState` fields 6–7); `iterations` (alpha solver iteration count) added to `LoadFactorOutputs`; `reset()` clears all four warm-start fields; [`docs/implementation/equations_of_motion.md`](../implementation/equations_of_motion.md) §Warm-Starting updated | 3 new tests in `LoadFactorAllocator_test.cpp`: `PredictorReducesIterationsOnLinearStep` (iterations == 1 for an exact linear-region prediction), `PredictorJsonRoundTrip_IncludesNzPrevAndNyPrev`, `PredictorProtoRoundTrip_IncludesNzPrev`; 22 tests total |
| 23 | `LoadFactorAllocator` test coverage extension — 8 new tests close continuity and domain-coverage gaps identified by code review. **White-box tests** (4): full positive and negative Nz sweeps through stall verifying the clamp value against `alphaPeak()`/`alphaTrough()`; fine-step sweep across the C¹ Linear→IncipientStall boundary confirming both segments are traversed; stall warm-start limitation test documenting that `reset()` is required after a discontinuous Nz jump. **Black-box tests** (4): uniform 500-step monotonicity sweeps from 0 to ±10 g for T = 0 (positive and negative) and T = `kLargeThrust` (positive); point-wise perturbation test at 37 grid points (T = 0, −9 g to +9 g) and 19 grid points (T > 0, 0 to +9 g) using fresh allocators. Stall warm-start limitation documented in [`docs/implementation/equations_of_motion.md`](../implementation/equations_of_motion.md) §Stall Warm-Start Limitation | 30 tests total in `LoadFactorAllocator_test.cpp` |
| 24 | **Aircraft command processing redesign** — all three command axes (`_nz_filter`, `_ny_filter`, `_roll_rate_filter`) converted to `setLowPassSecondIIR` (2nd-order LP); config params replaced: `cmd_deriv_tau_s`/`cmd_roll_rate_tau_s` → `nz_wn_rad_s`/`nz_zeta_nd`/`ny_wn_rad_s`/`ny_zeta_nd`/`roll_rate_wn_rad_s`/`roll_rate_zeta_nd`; `n_z_dot`/`n_y_dot` computed analytically from filter state after substep loop (no derivative filter lag); allocator receives shaped commands instead of raw clamped commands; Nyquist constraint enforced per axis (`wn * cmd_filter_dt_s < π`) at `initialize()`; proto `AircraftState` updated; `aircraft_config_v1` schema doc updated; fixture JSON files updated; design authority: [`docs/architecture/aircraft.md`](../architecture/aircraft.md) §Command Processing Architecture | 3 new Nyquist violation tests in `Aircraft_test.cpp` (`NyquistViolation_Nz_Throws`, `_Ny_Throws`, `_RollRate_Throws`); 345 pre-existing tests pass |
| 25 | **Landing gear model design** — full design authority document covering: use case decomposition; class hierarchy (`LandingGear`, `WheelUnit`, `StrutState`, `ContactForces`, `V_SurfaceFriction`, `SurfaceFrictionUniform`); physical models (suspension spring-damper, Pacejka magic formula, wheel friction, surface friction parameterization, ground plane interface); force assembly; step interface; JSON + proto serialization contract; computational resource estimate; test strategy (unit, integration, scenario, serialization); visualization notebook designs (`landing_gear_contact_forces.ipynb`, `crab_landing_dynamics.ipynb`, `takeoff_roll.ipynb`, `terrain_contact_animation.ipynb`); touchdown animation design (`touchdown_animation.py` — pybind11 driver, layout, visual encoding, coordinate mapping, data flow) | [`docs/architecture/landing_gear.md`](../architecture/landing_gear.md) |

---

## 1. Plot Visualization — Python Post-Processing Tools

Python scripts to load logger output and produce time-series plots for simulation
post-flight analysis. These are Application Layer tools and live under `python/tools/`.

### Deliverables — Plot Visualization

- `python/tools/plot_flight.py`: CLI script that reads a `.csv` log file produced by
  `Logger` and plots a configurable set of channels vs. time. Outputs a `.png` or
  displays interactively.
- Channel groups: kinematics (position, velocity, attitude), aerodynamics (α, β, CL, CD),
  propulsion (thrust, throttle), guidance (cross-track error, altitude error).

### Tests — Plot Visualization

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

## 2. Manual Input — Joystick and Keyboard

Manual input adapters translate human control inputs (joystick axes, keyboard state) into
an `AircraftCommand`. These live in the Interface Layer and have no physics logic.

### Deliverables — Manual Input

- `KeyboardInput`: maps configurable key bindings to throttle, roll, pitch, yaw increments.
  Operates at the simulation timestep rate.
- `JoystickInput`: reads a raw joystick device (SDL2 or platform API) and maps axes and
  buttons to the `AircraftCommand` fields. Applies a configurable dead zone and axis scaling.

### Interface Sketch — Manual Input

```cpp
// include/input/ManualInput.hpp
namespace liteaero::simulation {

class V_ManualInput {
public:
    virtual AircraftCommand read() = 0;   // non-blocking; returns latest command
    virtual ~V_ManualInput() = default;
};

} // namespace liteaero::simulation
```

### Tests — Manual Input

- `KeyboardInput` with no keys pressed returns zero lateral/directional commands and
  configured idle throttle.
- `JoystickInput` axis value at dead-zone boundary maps to zero output.
- Axis scaling: full-deflection axis value maps to the configured maximum command.

### CMake — Manual Input

Add `src/input/KeyboardInput.cpp` and `src/input/JoystickInput.cpp` to `liteaero-sim`.
Add a platform-conditional dependency on SDL2 for `JoystickInput`.

---

## 3. Execution Modes — Real-Time, Scaled, and Batch Runners

The simulation runner controls the wall-clock relationship to simulation time. Three modes
are required:

| Mode | Description |
| --- | --- |
| **Real-time** | Each `step()` is paced to its real elapsed wall time (`dt_s` per step). |
| **Scaled real-time** | Same as real-time but with a configurable speed multiplier (0.5×, 2×, etc.). |
| **Full-rate batch** | Steps run as fast as possible; no sleep; used for CI, Monte Carlo, and data generation. |

### Interface Sketch — SimRunner

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

### Tests — SimRunner

- `Batch` mode with `duration_s = 1.0` and `dt_s = 0.01` calls `Aircraft::step()` exactly
  100 times and then stops.
- `RealTime` mode: elapsed wall time after 10 steps is within 10% of `10 * dt_s`.
- `stop()` called from outside (threaded test) terminates the run loop within one timestep.
- `elapsed_sim_time_s()` returns `n_steps * dt_s` after `n_steps` have been executed.

### CMake — SimRunner

Add `src/runner/SimRunner.cpp` to `liteaero-sim`.
Add `test/SimRunner_test.cpp` to the test executable.

---

## 4. Remaining Sensor Models

Not blocking any higher-priority item. Stub headers in `include/sensor/` exist for
`SensorINS`, `SensorAA`, `SensorAAR`, `SensorRadAlt`, `SensorForwardTerrainProfile`, and
`SensorTrackEstimator`. Stubs for `SensorMag`, `SensorGnss`, and `SensorLaserAlt` have
not yet been created. Implement when needed; order within this group follows dependency.

| Class | Depends on | Hardware modeled |
| --- | --- | --- |
| `SensorMag` | `KinematicState` (done) | Triaxial magnetometer — body-frame field with hard-iron bias and soft-iron distortion |
| `SensorGnss` | `KinematicState` (done) | GNSS receiver — WGS84 position, NED velocity, SOG/COG, fix type, DOP |
| `SensorLaserAlt` | Terrain (done) | Laser altimeter — single-beam slant range and AGL altitude |
| `SensorRadAlt` | Terrain (done) | Radar altimeter — HAG from `Terrain::heightAboveGround_m` with noise and range saturation |
| `SensorINS` | `KinematicState` (done), `NavigationFilter` types | INS simulation replacement — truth-plus-error model producing `InsMeasurement` |
| `SensorForwardTerrainProfile` | Terrain (done), Guidance | Forward terrain profiling sensor (multi-beam LIDAR / line-scan radar) for terrain-following guidance |
| `SensorAA` | `KinematicState` (done) | Passive angle/angle sensor (imaging type) — measures two LOS angles, no range |
| `SensorAAR` | `KinematicState` (done) | Active angle/angle/range sensor (e.g. radar) — measures two LOS angles plus slant range |
| `SensorTrackEstimator` | `SensorAA` or `SensorAAR` | Kinematic track estimator for a moving object observed via angle or angle/range measurements |

---

## 5. Aerodynamic Coefficient Design Study

Prerequisite for item 7 (`Aircraft6DOF` and `BodyAxisCoeffModel`). Cannot be designed
without first defining how aerodynamic coefficients will be obtained and running the model
design process through several example aircraft configurations. This item produces the
design study that resolves OQ-16(c).

### Deliverables — Aerodynamic Coefficient Design Study

Design study document defining: aerodynamic coefficient data sources (wind tunnel, CFD,
DATCOM, flight test); data formats and axes conventions; coefficient model format for
`BodyAxisCoeffModel` across the range of anticipated fixed-wing configurations. Must be
complete before item 7 begins.

---

## 6. Aircraft6DOF — Design and Implementation

Full 6DOF aircraft dynamics model. Depends on item 5 (aero coefficient design study).
Architecture placeholders are defined in [`docs/architecture/system/future/element_registry.md`](../architecture/system/future/element_registry.md).

### Scope

- **`V_AeroModel`** — abstract aerodynamic model interface; produces forces and moments in
  body frame; decouples the 6DOF integrator from the coefficient axis convention.
- **`BodyAxisCoeffModel`** — implements `V_AeroModel` using body-axis stability derivatives
  (CX, CY, CZ, Cl, Cm, Cn) as functions of α, β, control surface deflections, and angular
  rates. Coefficient model format defined by item 5.
- **`Aircraft6DOF`** — full 6DOF dynamics; depends on `V_AeroModel` for forces and moments;
  accepts `SurfaceDeflectionCommand` (control surface deflection angles + per-motor
  throttle); produces `KinematicStateSnapshot`; used directly by ArduPilot and PX4
  simulations (no FBW bridge in that topology).
- **`FBWController`** — inner-loop FBW control law bridging the existing load-factor
  `AircraftCommand` interface to `SurfaceDeflectionCommand` for `Aircraft6DOF`; when paired
  with `Aircraft6DOF` forms a drop-in replacement for `Aircraft` for side-by-side
  comparison; not used in ArduPilot/PX4 topologies.
- **`SurfaceDeflectionCommand`** — plain value struct: control surface deflection angles
  (elevator, aileron, rudder) and per-motor throttle.

### Deliverables — Aircraft6DOF

Design authority document. Implementation follows TDD. All new elements include JSON +
proto serialization and round-trip tests. `Aircraft6DOF` and `Aircraft` must both produce
`KinematicStateSnapshot` to enable transparent substitution.

---

## 7. LandingGear — Implementation

Implement the design produced in delivered item 25. Architecture decisions are recorded in
[`docs/architecture/system/future/decisions.md`](../architecture/system/future/decisions.md) (LandingGear integration model and
fidelity target). Design authority: [`docs/architecture/landing_gear.md`](../architecture/landing_gear.md).

Implementation follows TDD — write a failing test before every production code change.

### Implementation Sequence

Each step must produce a green test suite before the next begins.

#### Step A — Data Types

Implement and serialize the value structs that all downstream code depends on. No physics
in this step.

| Class / Struct | File | Serialization |
| --- | --- | --- |
| `WheelUnit` | `include/landing_gear/WheelUnit.hpp` | JSON + proto |
| `StrutState` | `include/landing_gear/StrutState.hpp` | JSON + proto |
| `ContactForces` | `include/landing_gear/ContactForces.hpp` | JSON + proto |

Add `WheelUnit`, `StrutState`, and `ContactForces` messages to the proto schema.

**Tests — `test/landing_gear/LandingGearTypes_test.cpp`:**

- JSON and proto round-trips for all three structs pass with default-constructed values.
- `ContactForces` default-constructed has all force and moment components zero and
  `weight_on_wheels` false.

#### Step B — Surface Friction Interface and Uniform Model

| Class | File |
| --- | --- |
| `V_SurfaceFriction` | `include/landing_gear/V_SurfaceFriction.hpp` |
| `SurfaceFrictionUniform` | `include/landing_gear/SurfaceFrictionUniform.hpp` |

`V_SurfaceFriction::frictionCoefficients(position_m)` returns a `FrictionCoefficients`
struct (longitudinal peak, longitudinal sliding, lateral peak, lateral sliding). Surface
types: pavement, grass, dirt, gravel, wet pavement.

**Tests — `test/landing_gear/SurfaceFriction_test.cpp`:**

- `SurfaceFrictionUniform` returns configured coefficients at arbitrary world positions.
- JSON round-trip of `SurfaceFrictionUniform` config.
- Wet-pavement coefficients are strictly less than dry-pavement coefficients.

#### Step C — Core `LandingGear` Class (Normal Force, Flat Terrain)

Implement `LandingGear` as a `DynamicElement`. Tyre lateral and longitudinal forces are
deferred to Step D; this step establishes the full lifecycle and serialization contract.

- `initialize(config)` — parse `WheelUnit` list, suspension parameters, and tyre
  parameters; validate strut count.
- `reset()` — zero all `StrutState` fields.
- `step(state, terrain)` — for each wheel unit: compute penetration depth $h_i$ from
  `KinematicStateSnapshot` and `V_Terrain::heightAt()`; apply spring-damper normal force;
  set `weight_on_wheels`; sum forces and moments into `ContactForces`.
- `serializeJson()` / `deserializeJson()` — serialize `StrutState` array.
- `serializeProto()` / `deserializeProto()`.

Aerodynamic coefficients for the animation scenario are read from the JSON output of
`AeroCoeffEstimator` — see §[Aerodynamics Parameterization](#aerodynamics-parameterization).

**Tests — `test/landing_gear/LandingGear_test.cpp`:**

| Test | Pass criterion |
| --- | --- |
| `NoContact_ZeroForces` | All `ContactForces` components zero when aircraft is airborne |
| `Compressed_PositiveNormalForce` | $F_z > 0$ when wheel penetrates flat ground |
| `SymmetricGear_NoLateralMoment` | Symmetric main gear produces zero roll moment |
| `NoseGear_PitchMoment` | Nose gear contact produces nose-down pitch moment |
| `WeightOnWheels_SetWhenContact` | `weight_on_wheels` true iff $\exists\, i : h_i > 0$ |
| `StrutLimit_Compressed_Clamped` | Strut force does not exceed fully-compressed bound |
| `JsonRoundTrip_StrutState` | `serializeJson` / `deserializeJson` recovers `StrutState` |
| `ProtoRoundTrip_LandingGearState` | proto round-trip of `LandingGearState` |

#### Step D — Tyre Forces (Pacejka Magic Formula)

Add slip ratio and slip angle computation to `step()`. Implement the Pacejka magic formula

$$F(s) = D\sin\!\bigl(C\arctan(Bs - E(Bs - \arctan(Bs)))\bigr)$$

for both longitudinal ($F_x$, function of slip ratio $\kappa$) and lateral ($F_y$,
function of slip angle $\alpha_t$). Add nose wheel steering angle and differential brake
demand inputs to the step interface.

**Tests — additions to `LandingGear_test.cpp`:**

| Test | Pass criterion |
| --- | --- |
| `TyreForce_ZeroSlip_ZeroLateral` | $F_y = 0$ at zero slip angle |
| `TyreForce_PeakSlip_NearPeak` | $F_x$ peaks near $\kappa \approx 0.10$ |
| `TyreForce_Saturates_BeyondPeak` | $F_x$ does not increase monotonically with $\kappa$ |
| `NoseWheelSteering_ProducesYawMoment` | Non-zero steering angle produces body-axis yaw moment |
| `DifferentialBrake_ProducesRollMoment` | Asymmetric brake demand produces roll moment |

#### Step E — Terrain Normal Query

Replace the flat-ground assumption with a query to `V_Terrain::surfaceNormalAt()`. The
contact force assembly rotates from the local terrain frame to the body frame using the
terrain surface normal.

**Tests — `test/landing_gear/LandingGearTerrain_test.cpp`:**

- Contact force on a tilted flat surface matches the analytical projection.
- Wheel-ground normal tracks a known terrain slope angle.

#### Step F — `Aircraft` Integration

Wire `LandingGear` into `Aircraft::step()` on the disturbance force path. `ContactForces`
enters the 9-step physics loop between propulsion and aerodynamics. `weight_on_wheels` is
exposed on `AircraftState`.

**Tests — additions to `Aircraft_test.cpp`:**

- `LandingGear_GroundContact_PositiveNz` — aircraft on ground at zero velocity produces
  $n_z > 0$ from gear contact.
- `LandingGear_Airborne_ZeroContact` — aircraft at altitude produces `ContactForces` with
  all components zero.

#### Step G — Python Bindings (pybind11)

Expose `LandingGear`, `WheelUnit`, `StrutState`, and `ContactForces` to Python via
pybind11. Required by `python/scripts/touchdown_animation.py`.

Build target: `liteaero_sim_py` — a pybind11 extension module. Add to CMake as an
optional target gated on `LITEAERO_SIM_BUILD_PYTHON_BINDINGS=ON`.

Rewrite `python/scripts/touchdown_animation.py` to call `LandingGear::step()` via the
pybind11 module. The Python-side contact physics are removed; the Python-side
aerodynamics model (parameterized from `AeroCoeffEstimator` JSON output — see
§[Aerodynamics Parameterization](#aerodynamics-parameterization)) remains until a C++
aerodynamics class with bindings exists.

#### Step H — Scenario Tests and Visualization

Implement the four scenario tests from the design authority document as pytest fixtures
that drive `LandingGear` via the pybind11 module:

| Scenario | File | Pass criterion |
| --- | --- | --- |
| Landing rollout | `python/test/test_landing_rollout.py` | Aircraft decelerates to rest; max $F_z$ within 10% of analytical impulse estimate |
| Crab landing | `python/test/test_crab_landing.py` | Lateral drift eliminated within 3 s; no diverging yaw oscillation |
| Takeoff roll | `python/test/test_takeoff_roll.py` | All wheels leave ground at rotation speed; `weight_on_wheels` → false |
| Bounce (light contact) | `python/test/test_bounce.py` | Contact duration < 0.3 s; $F_z$ peak < 1.5× static weight |

---

### Aerodynamics Parameterization

The touchdown animation and Python scenario tests use a simplified aerodynamics model
(linear lift curve, parabolic drag polar, quasi-static pitch moment) that must be
consistent with the aircraft geometry under test. Coefficients are derived using the
existing `AeroCoeffEstimator` pipeline:

1. Define aircraft geometry in a JSON config (`AircraftGeometry` + `SurfaceGeometry`
   structs — already serializable).
2. Run `AeroCoeffEstimator` to derive `AeroPerformance` coefficients from geometry.
3. Write `AeroPerformance` to JSON (already implemented).
4. The Python animation and test scripts read the `AeroPerformance` JSON and extract
   $C_{L_\alpha}$, $C_{D_0}$, Oswald $e$, $AR$, $C_{m_0}$, $C_{m_\alpha}$, $\hat{C}_{m_q}$,
   $\bar{c}$, and $S_\text{ref}$.

This ensures the aerodynamics model in the animation matches the geometry of the aircraft
being simulated, with no hardcoded coefficients in the script.

---

### Deliverables

| Deliverable | Location |
| --- | --- |
| `V_SurfaceFriction`, `SurfaceFrictionUniform` | `include/landing_gear/`, `src/landing_gear/` |
| `WheelUnit`, `StrutState`, `ContactForces` | `include/landing_gear/` |
| `LandingGear` | `include/landing_gear/LandingGear.hpp`, `src/landing_gear/LandingGear.cpp` |
| Proto messages | `proto/` (new `LandingGearState` message) |
| Unit and integration tests | `test/landing_gear/` |
| pybind11 module | `python/liteaero_sim_py/` |
| `touchdown_animation.py` (rewritten) | `python/scripts/touchdown_animation.py` |
| Scenario test fixtures | `python/test/test_landing_*.py` |

### CMake

Add `liteaero::landing_gear` static library target. Dependencies: `liteaero::terrain`,
`liteaero-flight` (for `DynamicElement`). Add unit tests to `test/CMakeLists.txt`. Add
optional `liteaero_sim_py` pybind11 extension target controlled by
`LITEAERO_SIM_BUILD_PYTHON_BINDINGS`.

---

## 8. External Interface Elements

Adapters that connect LiteAero Sim to external systems. All live in the Interface Layer.

| # | Element | Protocol | Depends on |
| --- | --- | --- | --- |
| LAS-ext-1 | `ArduPilotInterface` | ArduPilot SITL protocol | SimRunner (item 3); `Aircraft` or `Aircraft6DOF` |
| LAS-ext-2 | `PX4Interface` | PX4 SITL bridge | SimRunner (item 3); `Aircraft6DOF` |
| LAS-ext-3 | `QGroundControlLink` | MAVLink over UDP | SimRunner (item 3); `NavigationState` (LiteAero Flight) |
| LAS-ext-4 | `VisualizationLink` | UDP to Godot 4 GDExtension plugin at simulation rate | SimRunner (item 3); `SimulationFrame` (done) |

Each element requires a design document before implementation. `VisualizationLink` transport
and axis convention are decided (see [`docs/architecture/terrain.md`](../architecture/terrain.md) §Game Engine Integration
and [`docs/architecture/system/future/decisions.md`](../architecture/system/future/decisions.md) §Game engine for real-time visualization).

---

## 9. Synthetic Perception Sensors — Proposed

The following sensor elements are proposed and not yet designed. They depend on `V_Terrain`
for geometry queries. Design items will be scheduled when prerequisite sensor and terrain
models are stable.

| Element | Responsibility |
| --- | --- |
| `SensorCamera` | Synthetic image sensor; generates imagery from terrain and scene model against `V_Terrain` |
| `SensorLidar` | Synthetic lidar; generates 3D point cloud by ray-casting against `V_Terrain` |
| `SensorLaserAGL` | Synthetic laser altimeter; computes AGL range by ray-casting against `V_Terrain` |
| `SensorLineOfSight` | Computes RF link quality and terrain occlusion by ray-casting against `V_Terrain` |
