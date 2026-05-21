# Data Flow Registry — Future State

## Data Flow Types

| ID | Type | Producer | Consumers | Description |
| --- | --- | --- | --- | --- |
| DFT-1 | `AircraftCommand` | `Autopilot` (or manual input) | `Aircraft` | Commanded normal load factor (g), lateral load factor (g), load factor rates (1/s), wind-frame roll rate (rad/s), normalized throttle [0, 1] |
| DFT-2 | `AtmosphericState` | `Atmosphere` | `Aircraft`, `SensorAirData`, `Propulsion` | Ambient temperature (K), static pressure (Pa), density (kg/m³), speed of sound (m/s), relative humidity (nd), density altitude (m) |
| DFT-3 | `EnvironmentState` | Assembled by `SimRunner` from environment components | `Aircraft` | Aggregate: `AtmosphericState` + wind NED (m/s) + `TurbulenceVelocity` (body, m/s + rad/s) + gust velocity body (m/s) |
| DFT-4 | `KinematicState` | `Aircraft` | Sensors, Logger, `Autopilot` | Position (WGS84), NED velocity (m/s), attitude (quaternion + Euler angles), body angular rates (rad/s), body acceleration (m/s²), alpha (rad), beta (rad), airspeed (m/s) |
| DFT-5 | `AirDataMeasurement` | `SensorAirData` | Logger, `NavigationFilter` | IAS, CAS, EAS, TAS (m/s), Mach (nd), barometric altitude (m), OAT (K) |
| DFT-6 | `TerrainQuery` / `TerrainResponse` | `SimRunner`, sensors, guidance | `V_Terrain` implementation | Input: NED position (m), observer range (m). Output: height_m (m), surface normal (unit vector NED) |
| DFT-7 | Log channel values | All components (via `LogSource`) | `Logger` | Named scalar or vector quantity at each timestep |
| DFT-8 | `TurbulenceVelocity` | `Turbulence` | `EnvironmentState` | Body-frame turbulence: u\_t, v\_t, w\_t (m/s); p\_t, q\_t, r\_t (rad/s) |
| DFT-9 | `NavigationState` | `NavigationFilter` | `Autopilot`, `WindEstimator`, Logger | Estimated position (NED or WGS84), NED velocity, attitude, angular rates, covariance |
| DFT-10 | `SetPoint` | Guidance components | `Autopilot` | Target altitude (m), vertical speed (m/s), heading (rad), roll attitude (rad) |
| DFT-11 | `PathQuery` / `PathResponse` | `PathGuidance` | `Path` | Query: position NED, heading. Response: crosstrack error (m), along-track (m), desired heading (rad), desired altitude (m), segment-complete flag |
| DFT-12 | `GroundContactForces` | `LandingGear` | `Aircraft` | Body-frame forces (N) and moments (N·m) from all active wheel contact points |
| DFT-13 | `SimulationState` | `SimRunner` (aggregated) | External interfaces, Logger | Full simulation snapshot: `KinematicState` + sensor measurements + flight code state; used for HITL streaming, visualization, and logging |
| DFT-14 | Telemetry / command stream | `QGroundControlLink`, `ArduPilotInterface`, `PX4Interface` | External ground stations / autopilot platforms | MAVLink messages (attitude, position, airspeed, mode, mission items) |

---

## Data Flow Instance Registry

| ID | Type | Producer | Consumers | Description |
| --- | --- | --- | --- | --- |
| DFT-9 | `NavigationState` | `NavigationFilter` | `Autopilot`, `WindEstimator`, Logger | Estimated position (NED or WGS84), NED velocity, attitude, angular rates, covariance |
| DFT-10 | `SetPoint` | Guidance components | `Autopilot` | Target altitude (m), vertical speed (m/s), heading (rad), roll attitude (rad) |
| DFT-11 | `PathQuery` / `PathResponse` | `PathGuidance` | `Path` | Query: position NED, heading. Response: crosstrack error (m), along-track (m), desired heading (rad), desired altitude (m), segment-complete flag |
| DFT-12 | `GroundContactForces` | `LandingGear` | `Aircraft` | Body-frame forces (N) and moments (N·m) from all active wheel contact points |
| DFT-13 | `SimulationState` | `SimRunner` (aggregated) | External interfaces, Logger | Full simulation snapshot: `KinematicState` + sensor measurements + flight code state; used for HITL streaming, visualization, and logging |
| DFT-14 | Telemetry / command stream | `QGroundControlLink`, `ArduPilotInterface`, `PX4Interface` | External ground stations / autopilot platforms | MAVLink messages (attitude, position, airspeed, mode, mission items) |

---

## Data Flow Instance Registry

| Instance | Type | From | To | Notes |
| --- | --- | --- | --- | --- |
| aircraft-command | DFT-1 | `Autopilot` (or manual input) | `Aircraft::step()` | In closed-loop mode, produced by Autopilot; in open-loop, by test harness or `ManualInput` |
| navigation-state | DFT-9 | `NavigationFilter` | `Autopilot`, `WindEstimator`, Logger | In simulation, may be replaced by `SensorInsSimulation` output |
| set-point | DFT-10 | Guidance (`PathGuidance`, `VerticalGuidance`, `ParkTracking`) | `Autopilot` | Outer loop → inner loop |
| path-query | DFT-11 | `PathGuidance` | `Path` | Path geometry query each step |
| ground-contact | DFT-12 | `LandingGear` | `Aircraft::step()` | Active only when any wheel is in contact; zero force/moment when airborne |
| simulation-state | DFT-13 | `SimRunner` | Visualization, HITL link | Streamed at simulation update rate |
| sensor-measurements | DFT-5 (+ others) | All sensors | `NavigationFilter` (or `SensorInsSimulation`) | In SITL/HITL, also streamed to flight hardware |
| telemetry-up | DFT-14 | `SimRunner` → interface adapters | QGroundControl, ArduPilot/PX4 | State encoded as MAVLink or custom |
| command-down | DFT-14 | QGroundControl, ArduPilot/PX4 | Interface adapters → Autopilot or `ManualInput` | Mode changes, mission items, override commands |
