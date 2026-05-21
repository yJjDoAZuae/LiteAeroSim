# LiteAero Sim — Documentation

## Document Taxonomy

LiteAero Sim uses six document types organized in a defined folder hierarchy. The type
determines the canonical format, the maintenance skill, and the inter-referencing rules.
Using the correct type and following the referencing rules is a project requirement
(CLAUDE.md §Rules 15–21).

| Type | Folder | Role | Skill |
| --- | --- | --- | --- |
| **Architecture** | [`docs/architecture/`](architecture/overview.md) | System topology, layer model, subsystem registry, cross-subsystem coordination contracts | [`/arch`](../.claude/commands/arch.md) |
| **Design** | [`docs/design/`](design/) | Per-subsystem authority: class hierarchy, models, interface, OQs, serialization, test strategy | [`/design`](../.claude/commands/design.md) |
| **Algorithm** | [`docs/algorithms/`](algorithms/) | Mathematical derivations, discretization methods, numerical properties — no code, no OQs | [`/algo`](../.claude/commands/algo.md) |
| **Schema** | [`docs/schemas/`](schemas/) | Serialization format specs: JSON field tables, proto messages, constraints, validation examples | [`/schema`](../.claude/commands/schema.md) |
| **Roadmap** | [`docs/roadmap/`](roadmap/) | What capabilities to build and why; delivered vs. pending; item-level blocking dependencies | [`/roadmap`](../.claude/commands/roadmap.md) |
| **Implementation plan** | [`docs/implementation/`](implementation/) | How to build it: atomic code-level work items with dependency order and status tracking | [`/impl`](../.claude/commands/impl.md) |

Open questions (`/oq`) are not a document type — they live inside architecture and design
documents using the format defined in [`.claude/commands/oq.md`](../.claude/commands/oq.md).

---

## How the types inter-reference each other

```text
Architecture (docs/architecture/)
  │ subsystem registry → links to
  ▼
Design (docs/design/)                     ◄── Roadmap "Design authority:" field
  │ model sections → cites                    (docs/roadmap/)
  ▼                                               │ spawns
Algorithm (docs/algorithms/)              Implementation plan (docs/implementation/)
  (derivations; no back-reference)                │ work item "Design refs:" → cites
  │                                               │         design doc sections
Design serialization section → cites      ◄───────┘
  ▼                                       │ blocked (OQ-XX-N) → references
Schema (docs/schemas/)                    │         OQs in design/architecture docs
  (field specs; links back to design)
```

**The key rule:** information flows from design authority toward planning documents, not
the reverse. A design document does not list which implementation plans reference it.

---

## Document Index

### Architecture

| Document | Contents |
| --- | --- |
| [`architecture/overview.md`](architecture/overview.md) | Layer model, subsystem map and registry, coordinate frames, data flow, component lifecycle |
| [`architecture/system/present/`](architecture/system/present/) | Current baseline state registry: requirements, use cases, element registry, dataflow registry, diagrams, ICDs |
| [`architecture/system/future/`](architecture/system/future/) | Roadmap-target state registry: same six documents for the planned architecture |

### Design

Design documents currently live in `docs/architecture/` pending migration to
`docs/design/` (see [`docs/design/README.md`](design/README.md)). They are valid
design authority documents regardless of their current location.

| Document | Subsystem |
| --- | --- |
| [`architecture/dynamic_element.md`](architecture/dynamic_element.md) | `DynamicElement` / `SisoElement` lifecycle contract; Filter and Propulsion hierarchies |
| [`architecture/aircraft.md`](architecture/aircraft.md) | `Aircraft` — integration loop, use cases, serialization |
| [`architecture/landing_gear.md`](architecture/landing_gear.md) | `LandingGear` — contact forces, wheel dynamics, serialization |
| [`architecture/propulsion.md`](architecture/propulsion.md) | `V_Propulsion`, `PropulsionJet`, `PropulsionProp`, `PropulsionEDF` |
| [`architecture/aero_coeff_estimator.md`](architecture/aero_coeff_estimator.md) | `AeroCoeffEstimator`, `AeroPerformance` |
| [`architecture/aero_coefficient_model.md`](architecture/aero_coefficient_model.md) | Aerodynamic coefficient model |
| [`architecture/sensor.md`](architecture/sensor.md) | Sensor base interface and hierarchy |
| [`architecture/sensor_air_data.md`](architecture/sensor_air_data.md) | `SensorAirData` |
| [`architecture/sensor_ins_sim.md`](architecture/sensor_ins_sim.md) | `SensorINS` (design doc; unreviewed) |
| [`architecture/sensor_laser_alt.md`](architecture/sensor_laser_alt.md) | `SensorRadAlt` (design doc; unreviewed) |
| [`architecture/sensor_gnss.md`](architecture/sensor_gnss.md) | `SensorGnss` |
| [`architecture/sensor_mag.md`](architecture/sensor_mag.md) | `SensorMag` |
| [`architecture/sim_runner.md`](architecture/sim_runner.md) | `SimRunner`, `RunnerConfig`, `ExecutionMode` |
| [`architecture/terrain.md`](architecture/terrain.md) | `V_Terrain`, `TerrainMesh`, height query interface |
| [`architecture/terrain_build.md`](architecture/terrain_build.md) | Python terrain ingestion pipeline |
| [`architecture/godot_plugin.md`](architecture/godot_plugin.md) | GDExtension C++ plugin — `SimulationReceiver`, build system |
| [`architecture/live_sim_view.md`](architecture/live_sim_view.md) | Live simulation viewer — UDP broadcast path, SimSession, Godot scene |
| [`architecture/python_bindings.md`](architecture/python_bindings.md) | pybind11 module — binding strategy, exposed classes |
| [`architecture/post_processing.md`](architecture/post_processing.md) | Python post-processing tools — `FlightLogReader`, `TimeHistoryFigure`, etc. |
| [`architecture/ring_buffer.md`](architecture/ring_buffer.md) | Ring buffer — `ChannelRegistry`, `ChannelSubscriber` |
| [`architecture/logger.md`](architecture/logger.md) | Logger subsystem |
| [`architecture/environment.md`](architecture/environment.md) | `Atmosphere`, `Wind`, `Turbulence`, `Gust` |
| [`architecture/manual_input.md`](architecture/manual_input.md) | `ManualInput`, `JoystickInput`, `KeyboardInput` |
| [`architecture/navigation_filter.md`](architecture/navigation_filter.md) | Navigation filter |
| [`architecture/flow_angles_estimator.md`](architecture/flow_angles_estimator.md) | `FlowAnglesEstimator` |
| [`architecture/wind_estimator.md`](architecture/wind_estimator.md) | `WindEstimator` |
| [`architecture/antiwindup.md`](architecture/antiwindup.md) | Anti-windup strategy |
| [`architecture/propulsion_coeff_estimator.md`](architecture/propulsion_coeff_estimator.md) | `PropulsionCoeffEstimator` |

### Algorithms

| Document | Contents |
| --- | --- |
| [`algorithms/filters.md`](algorithms/filters.md) | Tustin discretization with frequency prewarping; first-order and second-order filter coefficients |
| [`algorithms/aerodynamics.md`](algorithms/aerodynamics.md) | Aerodynamic coefficient models |
| [`algorithms/equations_of_motion.md`](algorithms/equations_of_motion.md) | RK4 equations-of-motion integration |
| [`algorithms/integration.md`](algorithms/integration.md) | Numerical integration methods |
| [`algorithms/air_data.md`](algorithms/air_data.md) | Air data computation — indicated airspeed, Mach, AGL |

### Schemas

| Document | Contents |
| --- | --- |
| [`schemas/aircraft_config_v1.md`](schemas/aircraft_config_v1.md) | `aircraft_config_v1` JSON schema — all sections and field constraints; maps to `Aircraft::initialize()` |

### Roadmap

| Document | Scope |
| --- | --- |
| [`roadmap/aircraft.md`](roadmap/aircraft.md) | Aircraft simulation subsystem: landing gear, sensors, propulsion, post-processing, architecture migration |
| [`roadmap/flight_code.md`](roadmap/flight_code.md) | Flight code (liteaero-flight) roadmap |

### Implementation Plans

| Document | Scope | Status |
| --- | --- | --- |
| [`implementation/PLANS.md`](implementation/PLANS.md) | Master index of all implementation plans | Index |
| [`implementation/landing_gear_dynamics.md`](implementation/landing_gear_dynamics.md) | Wheel kappa fix, rolling-condition clamp, Tustin ODE, airborne bearing drag | Active |

### Guidelines and Standards

| Document | Contents |
| --- | --- |
| [`guidelines/general.md`](guidelines/general.md) | TDD, naming standards, SI units, serialization, architecture |
| [`guidelines/cpp.md`](guidelines/cpp.md) | C++ conventions, tooling, testing with gtest, CMake |
| [`guidelines/python.md`](guidelines/python.md) | Python conventions, type hints, testing with pytest, tooling |
| [`testing/strategy.md`](testing/strategy.md) | TDD strategy, required test categories, coverage, known failures |
| [`examples/siso_elements.md`](examples/siso_elements.md) | Usage examples for filters, integrators, PID, serialization, logging |
| [`installation/README.md`](installation/README.md) | Build from source, toolchain setup, first run |
| [`dependencies/README.md`](dependencies/README.md) | License policy, dependency registry, Conan + FetchContent patterns |
