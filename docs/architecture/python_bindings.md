# Python Bindings — Architecture and Interface Design

This document is the design authority for the pybind11 extension module that exposes
LiteAero Sim C++ types to Python. It covers the module structure, CMake integration,
exported surface, and threading contract.

---

## Purpose

Python scenario scripts, Jupyter notebooks, and test fixtures require direct access to
selected C++ simulation types — primarily to drive simulation inputs from Python and to
enumerate hardware devices. All simulation logic remains in C++; the binding layer
provides access, not reimplementation.

---

## Module

| Property | Value |
| --- | --- |
| Python import name | `liteaero_sim_py` |
| Extension source entry point | `src/python/bindings.cpp` |
| CMake target | `liteaero_sim_py` (pybind11 extension module) |
| Build flag | `LITEAERO_SIM_BUILD_PYTHON_BINDINGS` (default `OFF`) |
| Conan dependency | `pybind11/2.11.1` (or latest stable in ConanCenter) |

The module is an optional build target. It is not built by default and is not required
for C++ unit tests or batch simulation. It must be enabled explicitly when building for
Python integration.

---

## CMake Integration

```cmake
# Root CMakeLists.txt (conditional on LITEAERO_SIM_BUILD_PYTHON_BINDINGS)
option(LITEAERO_SIM_BUILD_PYTHON_BINDINGS "Build pybind11 Python extension module" OFF)

if(LITEAERO_SIM_BUILD_PYTHON_BINDINGS)
    find_package(pybind11 REQUIRED)
    pybind11_add_module(liteaero_sim_py src/python/bindings.cpp)
    target_link_libraries(liteaero_sim_py PRIVATE liteaero-sim)
endif()
```

`pybind11` is declared in `conanfile.txt`. It is a project-wide dependency; it must
appear in `conanfile.txt` and the dependency registry
([`docs/dependencies/README.md`](../dependencies/README.md)) before any binding code
is built.

---

## Source Organization

`src/python/bindings.cpp` is the single entry point: it defines the `PYBIND11_MODULE`
block and calls one registration function per subsystem. Each subsystem's registration
function is declared in a header and defined in a dedicated source file co-located with
the C++ it wraps:

```text
src/python/
    bindings.cpp              — PYBIND11_MODULE entry point; calls bind_*() functions
    bind_manual_input.cpp     — AircraftCommand, ScriptedInput, JoystickInput
    bind_landing_gear.cpp     — LandingGear, WheelUnit, StrutState, ContactForces
```

Each `bind_*()` function signature follows the pattern:

```cpp
void bind_manual_input(py::module_& m);
void bind_landing_gear(py::module_& m);
```

New subsystems add a new `bind_*.cpp` file and a call in `bindings.cpp`. No existing
files are modified when adding a new subsystem binding.

---

## Exported Surface

### Manual Input

Design authority: [`manual_input.md`](manual_input.md)

| C++ type / function | Python exposure | Purpose |
| --- | --- | --- |
| `AircraftCommand` | Class with named constructor arguments (`n_z`, `n_y`, `roll_rate_wind_rps`, `throttle_nd`) and attribute access | Passed to `ScriptedInput.push()` from Python scenario scripts |
| `ScriptedInput::push(const AircraftCommand&)` | Instance method | Drive simulation inputs from Python |
| `JoystickInput::enumerateDevices()` | Static method returning a list of dicts (`device_index`, `name`, `num_axes`) | Enumerate connected joystick devices from Python setup code or notebooks |

### Landing Gear

Design authority: [`landing_gear.md`](landing_gear.md)

| C++ type / function | Python exposure | Purpose |
| --- | --- | --- |
| `LandingGear` | Class with `initialize()`, `reset()`, `step()` | Call contact physics from Python scenario and animation scripts |
| `WheelUnit` | Class | Per-wheel state access |
| `StrutState` | Class | Strut deflection state |
| `ContactForces` | Class | Ground contact force output |

---

## Threading Contract

pybind11 releases the GIL when calling into C++ functions that do not touch Python
objects. The following rules apply to all binding code in this module:

- `ScriptedInput::push()` acquires `mutex_` internally (C++ mutex, not the GIL). It
  may be called from the Python thread while `SimRunner` calls `read()` from the
  simulation thread. No additional locking is required in the binding.
- `JoystickInput::enumerateDevices()` calls SDL2 functions. SDL2 must have been
  initialized (`SDL_Init`) before this call. Callers are responsible for SDL lifecycle.
- `LandingGear::step()` is not thread-safe with respect to concurrent Python access.
  Callers must not call it from Python while a `SimRunner` run is in progress with the
  same `LandingGear` instance.

---

## Open Questions

None.
