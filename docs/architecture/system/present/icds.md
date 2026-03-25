# Interface Control Documents — Present State

At the present stage, ICDs identify each interface, its data content, and the
architectural constraints that govern it. Field-level schema definitions are in the C++
headers and `proto/liteaerosim.proto`; they are not reproduced here.

ICD-1, ICD-4, ICD-5, ICD-6, and ICD-7 are owned by liteaero-flight and defined in
`liteaero-flight/docs/interfaces/icds.md`. The entries below are cross-references only.
ICD-2 and ICD-3 are owned by LiteAero Sim and defined here.

---

## ICD-1 — AircraftCommand

**Owner:** liteaero-flight (ICD-F1). See `liteaero-flight/docs/interfaces/icds.md`.

---

## ICD-2 — AtmosphericState

**Producer:** `Atmosphere::step()`

**Consumers:** `EnvironmentState` assembly; `SensorAirData::step()`; `Propulsion` subclasses.

**Transport:** Value struct passed by value or const reference.

**Content:**

| Field | Unit | Description |
| --- | --- | --- |
| `temperature_k` | K | Static (ambient) temperature |
| `pressure_pa` | Pa | Static pressure |
| `density_kgm3` | kg/m³ | Moist air density |
| `speed_of_sound_mps` | m/s | Speed of sound |
| `relative_humidity_nd` | nd | Relative humidity [0, 1] |
| `density_altitude_m` | m | ISA altitude at which ρ\_ISA = ρ\_actual |

**Constraints:**

- Queried once per step at the current geometric altitude of the aircraft.
- The struct is immutable after production; no consumer modifies it.

---

## ICD-3 — EnvironmentState

**Producer:** Simulation loop (assembles from Atmosphere, Wind, Turbulence, Gust).

**Consumer:** `Aircraft::step()`

**Transport:** Value struct.

**Content:**

| Field | Type/Unit | Description |
| --- | --- | --- |
| `atmosphere` | `AtmosphericState` | See ICD-2 |
| `wind_NED_mps` | m/s (NED vector) | Steady ambient wind velocity |
| `turbulence` | `TurbulenceVelocity` | Body-frame continuous turbulence (m/s + rad/s) |
| `gust_body_mps` | m/s (body vector) | Discrete gust velocity |

**Constraints:**

- All velocities SI.
- Wind is expressed in NED frame; turbulence and gust in body frame.

---

## ICD-4 — KinematicStateSnapshot

**Owner:** liteaero-flight (ICD-F2). See `liteaero-flight/docs/interfaces/icds.md`.

Note: the liteaero-sim wrapper `KinematicState` holds a `KinematicStateSnapshot` and
exposes it via `snapshot()`.

---

## ICD-5 — AirDataMeasurement

**Owner:** liteaero-flight (ICD-F4). See `liteaero-flight/docs/interfaces/icds.md`.

---

## ICD-6 — Terrain Query Interface (`V_Terrain`)

**Owner:** liteaero-flight (ICD-F5). See `liteaero-flight/docs/interfaces/icds.md`.

---

## ICD-7 — Logger Write Interface

**Owner:** liteaero-flight (ICD-F6). See `liteaero-flight/docs/interfaces/icds.md`.
