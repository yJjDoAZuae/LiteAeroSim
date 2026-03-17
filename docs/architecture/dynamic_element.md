# DynamicElement — Unified Base Class for Dynamic Components

This document is the design authority for `DynamicElement`, the unified abstract base class
for all stateful, time-evolving components in LiteAeroSim. It defines the lifecycle
contract, the NVI pattern, the logging interface, and the migration plan for consolidating
the currently inconsistent family of base classes (`DynamicBlock`, `V_Sensor`,
`V_Propulsion`).

This document also describes how `SisoElement` and `Filter` sit alongside the composable
`SISOBlock` hierarchy.

---

## Motivation

LiteAeroSim contains several families of components that share an identical behavioral
contract: they maintain internal state, receive time-stepped inputs, and produce
time-correlated outputs whose characteristics are determined by their dynamic properties.
This includes sensors, estimators, control filters, propulsion models, and environment
models such as Dryden turbulence filters.

Each of these families currently defines its own base class independently. Those bases are
mutually inconsistent:

| Base | `initialize()` | NVI | Serialization naming | Logging |
| --- | --- | --- | --- | --- |
| `DynamicBlock` | `initialize(json)` | Yes | `serialize()` / `deserialize()` | `attachLogger` / `onLog` |
| `V_Sensor` | `initialize(json)` | Yes | `serializeJson()` / `deserializeJson()` | None |
| `V_Propulsion` | None | No | `serializeJson()` / `deserializeJson()` | None |

Consequences of this inconsistency:

- Any cross-cutting concern (logging, telemetry, watchdog, replay) must be added to every
  base class separately.
- Generic tooling (scenario replay, Monte-Carlo harness, snapshot comparison) cannot hold
  a heterogeneous collection of dynamic components through a common base pointer.
- The `V_` naming convention is non-standard and not recognized outside this codebase.

A unified root base class eliminates all three problems.

---

## Naming

The root base class is named **`DynamicElement`**.

The name is drawn from the same usage as "circuit element," "finite element," and "boundary
element" — a fundamental building block in a larger interconnected system. "Dynamic"
distinguishes it from static configuration structs and data-only value objects.

The alternative term "block" was ruled out due to its connotation with concurrent /
multithreaded execution. "Component" and "module" were ruled out as too generic in a
mixed-language codebase.

Categorical identity (sensor, estimator, propulsion) is expressed through namespace
(`liteaerosim::sensor`, `liteaerosim::estimation`, `liteaerosim::propulsion`) rather than
through thin base classes. Thin categorical base classes add a level of inheritance depth
with no behavioral content and no meaningful interface; namespaces communicate the same
category information with less overhead.

---

## `SISOBlock`, `SisoElement`, and `Filter`

### Tier 1 — Composable SISO interface (no lifecycle)

`SISOBlock` is the minimal composable SISO interface: `step(float u)`, `in()`, `out()`.
It carries no lifecycle and is designed to be embedded in any context — simulation loop,
flight code, or unit test — without requiring initialization. `FilterSS` derives from
`SISOBlock` and provides a complete, self-contained first-order state-space filter that
works correctly from construction without any lifecycle call.

`SISOBlock` is not part of the `DynamicElement` tree. It represents a separate, minimal
interface tier that can be used anywhere lifecycle machinery is unnecessary or undesirable.

### Tier 2 — `SisoElement` (merges `DynamicBlock` and `SISOBlock`)

`SisoElement` is the fully-capable SISO dynamic element. It derives from both
`DynamicElement` (lifecycle) and `SISOBlock` (SISO step interface):

```cpp
class SisoElement : public liteaerosim::DynamicElement,
                    public liteaerosim::SISOBlock { ... };
```

Merging `DynamicBlock` into `SisoElement` means every SISO component in the tree
automatically carries the full lifecycle — useful for control elements embedded in larger
`DynamicElement` owners (`Autopilot`, `NavigationFilter`, `PropulsionProp`) that serialize
their own state. Embedded uses that do not need lifecycle simply do not call `initialize()`
or `serializeJson()`.

Both parents are pure abstract interfaces with no shared state; the multiple inheritance
introduces no ambiguity. `SisoElement` is the only class in the hierarchy that inherits
from both.

### `Filter` (under `SisoElement`)

`Filter` extends `SisoElement` with filter-specific query and control methods: `order()`,
`dcGain()`, `resetToInput()`, `resetToOutput()`, `errorCode()`. `Filter` is the abstract
base for all filter implementations (`FilterSS2`, `FilterSS2Clip`, `FilterTF`, `FilterTF2`,
`FilterFIR`).

Once `Filter` derives from `SisoElement` (and therefore from `DynamicElement`), the
existing `DynamicFilterBlock` class becomes redundant — it adds the same filter-specific API
on top of `DynamicBlock` that `Filter` adds on top of `SisoElement`. `DynamicFilterBlock`
is retired and its subclasses move under `Filter`.

One naming inconsistency is resolved in this migration: `Filter` currently uses
`resetInput()`/`resetOutput()`, while `DynamicFilterBlock` uses `resetToInput()`/
`resetToOutput()`. The `resetTo*` form is more explicit and is adopted as the canonical
name; `Filter` is updated accordingly.

### `LimitElement` (under `SisoElement`)

`LimitElement` replaces `DynamicLimitBlock`. It extends `SisoElement` with limit-control
methods (`enableLower()`, `setUpper()`, `isLimited()`, etc.) and is the parent of `Limit`,
`RateLimit`, and related elements. No interface change beyond the rename.

---

## Why No Common `step()` Signature

`SisoElement` can define a common `step(float u) -> float` because all SISO control
elements share the same I/O type. No other category shares a common I/O type:

- A sensor `step()` takes hardware-specific inputs (atmospheric state, kinematic state,
  terrain query) and returns a sensor-specific measurement struct.
- An estimator `step()` takes a heterogeneous mix of measurement structs and returns an
  estimate struct.
- A propulsion `step()` takes throttle, airspeed, and density and returns thrust.

`DynamicElement` therefore provides only the lifecycle and logging contract. The simulation
loop holds concrete types (or category-namespace base pointers) and calls typed `step()`
directly.

---

## Class Interface

```cpp
// include/DynamicElement.hpp
#pragma once
#include "ILogger.hpp"
#include <nlohmann/json.hpp>

namespace liteaerosim {

/// Abstract base for all stateful, time-evolving simulation components.
///
/// Lifecycle (in order):
///   initialize(config) — one-time setup from JSON config
///   reset()            — return to post-initialize state; may be called
///                        between runs without re-reading config
///   step(...)  × N     — advance one timestep; signature defined by each
///                        concrete class; not declared on this base
///   serializeJson() / deserializeJson() — checkpoint at any point after initialize()
///
/// initialize() and deserializeJson() validate "schema_version" in the JSON
/// object before forwarding to the protected hook; throw std::runtime_error on
/// mismatch.
///
/// Proto serialization is not declared on this base because proto message types
/// are component-specific. Each concrete class declares serializeProto() /
/// deserializeProto() with the appropriate message type.
class DynamicElement {
public:
    virtual ~DynamicElement() = default;

    void initialize(const nlohmann::json& config);
    void reset();
    [[nodiscard]] nlohmann::json serializeJson() const;
    void deserializeJson(const nlohmann::json& state);

    void attachLogger(ILogger* logger) noexcept;

protected:
    virtual void           onInitialize(const nlohmann::json& config) = 0;
    virtual void           onReset()                                   = 0;
    virtual nlohmann::json onSerializeJson()                   const   = 0;
    virtual void           onDeserializeJson(const nlohmann::json&)    = 0;
    virtual void           onLog(ILogger& /*logger*/)          const   {}
    virtual int            schemaVersion()                     const   = 0;
    virtual const char*    typeName()                          const   = 0;

private:
    ILogger* logger_ = nullptr;
};

} // namespace liteaerosim
```

`schemaVersion()` and `typeName()` are pure-virtual on the base so that every concrete
class is required to declare them explicitly. The base injects both into every serialized
snapshot so that diagnostics tools can identify and version-check any snapshot without
knowing the concrete type.

---

## Complete Class Hierarchy

### DynamicElement tree

```text
liteaerosim::DynamicElement
│
├── liteaerosim::SisoElement          (derives from both DynamicElement and SISOBlock)
│   ├── liteaerosim::control::Filter  (adds filter API; replaces DynamicFilterBlock)
│   │   ├── FilterSS2
│   │   ├── FilterSS2Clip
│   │   ├── FilterTF
│   │   ├── FilterTF2
│   │   └── FilterFIR
│   ├── liteaerosim::control::LimitElement  (adds limit API; replaces DynamicLimitBlock)
│   │   ├── Limit
│   │   └── RateLimit
│   ├── Integrator
│   ├── Gain
│   ├── Derivative
│   ├── Unwrap
│   ├── SISOPIDFF
│   └── (other SISO control elements)
│
├── liteaerosim::sensor::SensorAirData
├── liteaerosim::sensor::SensorGnss
├── liteaerosim::sensor::SensorLaserAlt
├── liteaerosim::sensor::SensorMag
├── liteaerosim::sensor::SensorRadAlt
├── liteaerosim::sensor::SensorAA
├── liteaerosim::sensor::SensorAAR
├── liteaerosim::sensor::SensorTrackEstimator
├── liteaerosim::sensor::SensorForwardTerrainProfile
├── liteaerosim::sensor::SensorInsSimulation
│
├── liteaerosim::estimation::NavigationFilter
├── liteaerosim::estimation::WindEstimator
├── liteaerosim::estimation::FlowAnglesEstimator
│
├── liteaerosim::propulsion::PropulsionJet
├── liteaerosim::propulsion::PropulsionEDF
├── liteaerosim::propulsion::PropulsionProp
├── liteaerosim::propulsion::MotorElectric
└── liteaerosim::propulsion::MotorPiston
```

Categorical identity is expressed through namespace only. There are no thin categorical
base classes for sensors, estimators, or propulsion models.

### SISOBlock / FilterSS tree (not under DynamicElement)

```text
liteaerosim::SISOBlock
├── liteaerosim::SisoElement     (also derives from DynamicElement — see above)
└── liteaerosim::control::FilterSS
```

`SisoElement` appears in both trees because it inherits from both roots. `FilterSS` is
Tier 1 only — it has no lifecycle and is not a `DynamicElement`.

**Subsystems not yet evaluated** — the following subsystems contain classes that may be
dynamic elements and should be reviewed against this hierarchy before implementation:
`include/guidance/`, `include/aerodynamics/` (`AeroCoeffEstimator`),
`include/environment/` (`Turbulence`, `Gust`), `include/path/` (`V_PathSegment`).

---

## Schema Version Convention

Schema version validation moves from each individual base class into
`DynamicElement::initialize()` and `DynamicElement::deserializeJson()`. Each concrete class
continues to supply its own `kSchemaVersion_` constant and implements `schemaVersion()` to
return it. Behavior is unchanged; the check fires once in the root base rather than in each
family's base.

---

## Migration Plan

The migration is purely mechanical — no behavioral change at any step. Each step is
independently buildable and testable.

### Step 1 — Create `DynamicElement`

- Create `include/DynamicElement.hpp` and `src/DynamicElement.cpp`
- Implement `initialize()`, `reset()`, `serializeJson()`, `deserializeJson()`,
  `attachLogger()` — logic taken verbatim from `DynamicBlock`'s existing implementations
- No test file: `DynamicElement` is abstract; its contract is verified by every concrete
  subclass's existing tests

### Step 2 — Rename `DynamicBlock` → `SisoElement`; merge `SISOBlock`

- Rename `DynamicBlock` to `SisoElement`; update all references
- Change declaration to `class SisoElement : public DynamicElement, public SISOBlock`
- Remove duplicated lifecycle logic from `SisoElement` (now inherited from `DynamicElement`)
- Remove `attachLogger` / `onLog` from `SisoElement` (now inherited from `DynamicElement`)
- Rename `serialize()` → `serializeJson()`, `deserialize()` → `deserializeJson()` on
  `SisoElement` and all subclasses
- Update all call sites in test files
- `SISOBlock` header (`include/SISOBlock.hpp`) is retained as the interface type; its
  former role as a standalone composable element is unchanged

### Step 3 — Migrate `Filter`; retire `DynamicFilterBlock`

- Change `Filter` to derive from `SisoElement` instead of `SISOBlock` directly
- Rename `resetInput()`/`resetOutput()` → `resetToInput()`/`resetToOutput()` on `Filter`
- Move `FilterSS2`, `FilterSS2Clip`, `FilterTF`, `FilterTF2`, `FilterFIR` from
  `DynamicFilterBlock` to `Filter` directly (parent change only; no interface change)
- Delete `include/control/DynamicFilterBlock.hpp` and `src/control/DynamicFilterBlock.cpp`
  (if it exists as a separate translation unit)
- Update all `#include "control/DynamicFilterBlock.hpp"` directives

### Step 4 — Rename `DynamicLimitBlock` → `LimitElement`

- Rename `DynamicLimitBlock` to `LimitElement`; update all references and `#include`s

### Step 5 — Migrate `V_Sensor`; delete `V_Sensor.hpp`

- Change all sensor classes (`SensorAirData`, `SensorGnss`, etc.) to derive from
  `DynamicElement` directly (namespace `liteaerosim::sensor`)
- Remove `include "sensor/V_Sensor.hpp"` from all sensor headers; add `include "DynamicElement.hpp"`
- Delete `include/sensor/V_Sensor.hpp`

### Step 6 — Migrate `V_Propulsion`; delete `V_Propulsion.hpp` and `V_Motor.hpp`

- Change all propulsion models to derive from `DynamicElement` directly
  (namespace `liteaerosim::propulsion`)
- Add `initialize(json)` to all propulsion models; move construction-time config into
  `onInitialize()`; switch from direct virtual methods to the NVI pattern
- Delete `include/propulsion/V_Propulsion.hpp`, `include/propulsion/V_Motor.hpp`
- Update `docs/architecture/propulsion.md` base class names and lifecycle section

### Step 7 — Evaluate remaining subsystems

Review `include/guidance/`, `include/aerodynamics/`, `include/environment/`, and
`include/path/` against the `DynamicElement` contract. For each class that qualifies,
derive it from `DynamicElement` directly under the appropriate namespace.

---

## Files Created / Modified

| File | Action |
| --- | --- |
| `include/DynamicElement.hpp` | **Create** |
| `src/DynamicElement.cpp` | **Create** |
| `include/DynamicBlock.hpp` | **Rename** → `include/SisoElement.hpp` |
| `src/DynamicBlock.cpp` | **Rename** → `src/SisoElement.cpp` |
| `include/control/DynamicFilterBlock.hpp` | **Delete** (Filter takes its role) |
| `include/control/DynamicLimitBlock.hpp` | **Rename** → `include/control/LimitElement.hpp` |
| `include/control/Filter.hpp` | **Modify** (derive from SisoElement; rename reset methods) |
| All `SisoElement` subclass headers and sources | **Modify** (rename serialize methods) |
| All sensor class headers | **Modify** (drop V_Sensor; derive from DynamicElement directly) |
| All propulsion class headers and sources | **Modify** (add initialize, NVI, drop V_Propulsion) |
| `include/sensor/V_Sensor.hpp` | **Delete** |
| `include/propulsion/V_Propulsion.hpp` | **Delete** |
| `include/propulsion/V_Motor.hpp` | **Delete** |
| `docs/architecture/sensor.md` | **Modify** (update base class name) |
| `docs/architecture/propulsion.md` | **Modify** (update base class names, lifecycle) |
