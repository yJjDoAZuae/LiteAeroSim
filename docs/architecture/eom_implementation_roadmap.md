# Equations of Motion — Implementation Roadmap

This document tracks the remaining implementation work for the full equations-of-motion subsystem.
Each phase must pass its tests (green) before the next phase starts.

---

## Status Summary

| Phase | Description | Status |
|-------|-------------|--------|
| A | LiftCurveModel — 5-region piecewise lift curve | **Complete** (16 tests) |
| B | KinematicState — store `_q_nw`, complete `step()` | Not started |
| C | KinematicState — derived-quantity implementations | Not started |
| D | KinematicState — position integration (WGS84) | Not started |
| E | LoadFactorAllocator — implicit α/β Newton solver | Not started |

---

## Phase A — LiftCurveModel ✓ Complete

**Files:** `include/aerodynamics/LiftCurveModel.hpp`, `src/aerodynamics/LiftCurveModel.cpp`,
`test/LiftCurveModel_test.cpp`

**Interface:**

```cpp
struct LiftCurveParams {
    float cl_alpha;              // pre-stall lift-curve slope (rad⁻¹)
    float cl_max;                // peak CL (positive stall vertex)
    float cl_min;                // minimum CL (negative stall vertex, < 0)
    float delta_alpha_stall;     // angular distance: positive linear join → positive vertex (rad)
    float delta_alpha_stall_neg; // angular distance: negative linear join → negative vertex (rad)
    float cl_sep;                // positive post-stall plateau (< cl_max)
    float cl_sep_neg;            // negative post-stall plateau (> cl_min)
};

class LiftCurveModel {
    float evaluate(float alpha_rad) const;   // C_L(α)
    float derivative(float alpha_rad) const; // dC_L/dα
    float alphaPeak() const;                 // α at cl_max
    float alphaTrough() const;               // α at cl_min
};
```

**5-region model:**

| Region | Condition | C_L(α) |
|--------|-----------|--------|
| 1 — neg flat | α < α_sep_neg | cl_sep_neg |
| 2 — neg quadratic | α_sep_neg ≤ α < α_*_neg | a2n·α²+a1n·α+a0n (upward-opening) |
| 3 — linear | α_*_neg ≤ α ≤ α_* | cl_alpha·α |
| 4 — pos quadratic | α_* < α ≤ α_sep | a2·α²+a1·α+a0 (downward-opening) |
| 5 — pos flat | α > α_sep | cl_sep |

---

## Phase B — KinematicState: Store `_q_nw`

### Goal

`q_nw()` is currently **undefined** (no function body exists — any call is an ODR violation).
`step()` computes a corrected `local_q_nw` but immediately discards it.
This phase makes `_q_nw` a stored member and completes the three outstanding TODOs in `step()`.

### Files to modify

- `include/KinematicState.hpp`
- `src/KinematicState.cpp`
- `test/KinematicState_test.cpp` (new — write first, TDD)

### Header changes (`include/KinematicState.hpp`)

**Add member** in the `protected` block (after `_q_nb`):
```cpp
Eigen::Quaternionf _q_nw;   // Wind-to-NED rotation (stored state)
```

**Replace** the two wind scalars:
```cpp
// Remove:
float _windVelNorth;
float _windVelEast;

// Add:
Eigen::Vector3f _wind_NED_mps;
```

**Add getter** (inline, since `_q_nw` is now a stored member):
```cpp
Eigen::Quaternionf q_nw() const { return _q_nw; }
```
Remove the existing `q_nw() const;` declaration (currently unimplemented).

**Constructor 2** (takes `q_nb` directly, lines 53–65 of `.hpp`) does not currently receive
a `q_nw` parameter. Two options:
- Add `const Eigen::Quaternionf& q_nw` parameter and initialize `_q_nw(q_nw)`.
- Initialize `_q_nw(Eigen::Quaternionf::Identity())` (acceptable if this constructor is only
  used for already-NED states where Wind ≡ NED).

Choose based on actual call sites. If no call site needs non-Identity `q_nw` via this
constructor, default to `Identity()`.

### Implementation changes (`src/KinematicState.cpp`)

**Constructor 1** — add `_q_nw(q_nw)` to member-initializer list:
```cpp
: _time_sec(time_sec),
  _positionDatum(position_datum),
  _velocity_NED_mps(velocity_NED_mps),
  _acceleration_NED_mps(q_nw.toRotationMatrix() * acceleration_Wind_mps),
  _q_nw(q_nw),                      // add this
  _q_nb(Eigen::Quaternionf::Identity()),
  _wind_NED_mps(-windSpeed_mps * Eigen::Vector3f(cos(windDirFrom_rad),
                                                  sin(windDirFrom_rad), 0.0f)),
  _rates_Body_rps(Eigen::Vector3f::Zero())
```

**Replace all four `_windVelNorth`/`_windVelEast` sites** with `_wind_NED_mps`:

| Location | Old code | New code |
|----------|----------|----------|
| Constructor body | `_windVelNorth = ...; _windVelEast = ...;` | removed (handled in initializer) |
| `crab()` line 149 | `wind_NED_mps << _windVelNorth, _windVelEast, 0;` | `Eigen::Vector3f wind_NED_mps = _wind_NED_mps;` |
| `crabRate()` line 162 | same | same |
| `step()` line 240 | `Eigen::Vector3f windVel_NED = Eigen::Vector3f(_windVelNorth, _windVelEast, 0);` | `Eigen::Vector3f windVel_NED = _wind_NED_mps;` |

Also update `step()`: when `windSpeed_mps` / `windDirFrom_rad` arrive as parameters,
update the stored member:
```cpp
_wind_NED_mps = -windSpeed_mps * Eigen::Vector3f(cos(windDirFrom_rad),
                                                   sin(windDirFrom_rad), 0.0f);
```

**Complete the three TODOs in `step()`:**

Replace lines 267–278 with:
```cpp
// Step 1: path-curvature rotation in NED (already computed above)
Eigen::Quaternionf diff_rot_n;
diff_rot_n.setFromTwoVectors(velocity_NED_mps_prev, _velocity_NED_mps);

// Step 2: roll rotation around the Wind X axis
Eigen::Quaternionf roll_delta(
    Eigen::AngleAxisf(rollRate_Wind_rps * dt, Eigen::Vector3f::UnitX()));

// Step 3: propagate _q_nw
_q_nw = (diff_rot_n * _q_nw * roll_delta).normalized();
```

Lines 284–286 (`q_wb` and `_q_nb = q_nw() * q_wb`) need **no textual change** — they
already call `q_nw()`, which will now return the stored `_q_nw`.

Line 309 (`_rates_Body_rps = ...q_nw().toRotationMatrix()...`) similarly needs no change.

**Remove the now-unreachable `q_nw()` definition** in `.cpp` (currently the function body
is missing — there is nothing to remove, but confirm no orphan definition exists).

### Tests (`test/KinematicState_test.cpp`) — write first

```
Test 1 — q_nw becomes non-Identity after roll
    Construct state with Identity q_nw, nonzero velocity, zero wind.
    Call step() with rollRate_Wind_rps = 0.1 rad/s, alpha=0, beta=0, nonzero acceleration.
    Assert q_nw() != Identity (coefficient deviation > 1e-6).

Test 2 — q_nb = q_nw * q_wb
    After one step with known alpha and beta, verify:
    q_nb() ≈ q_nw() * q_wb(alpha, beta)
    where q_wb = AngleAxisf(alpha, UnitY) * AngleAxisf(-beta, UnitZ).

Test 3 — rates_Body_rps has correct magnitude
    Zero alpha/beta/alphaDot/betaDot, zero rollRate, constant acceleration.
    Verify _rates_Body_rps is consistent with path curvature: |ω| ≈ |a_perp| / |V|.
```

**Note:** `setFromTwoVectors` is degenerate when the two vectors are identical (zero-acceleration
straight-line step). Test 1 must supply nonzero acceleration so `diff_rot_n ≠ Identity`,
or rely only on `roll_delta` (nonzero `rollRate_Wind_rps`) for the non-Identity assertion.
The latter is simpler and avoids the degenerate case.

---

## Phase C — KinematicState: Derived Quantity Implementations

### Prerequisite

Phase B complete (stored `_q_nw` and `_wind_NED_mps` available).

### Implementations (replace stubs in `src/KinematicState.cpp`)

```cpp
Eigen::Vector3f KinematicState::velocity_Wind_mps() const {
    return _q_nw.toRotationMatrix().transpose() * (_velocity_NED_mps - _wind_NED_mps);
}

Eigen::Vector3f KinematicState::velocity_Body_mps() const {
    return _q_nb.toRotationMatrix().transpose() * _velocity_NED_mps;
}

Eigen::Vector3f KinematicState::acceleration_Wind_mps() const {
    return _q_nw.toRotationMatrix().transpose() * _acceleration_NED_mps;
}

Eigen::Vector3f KinematicState::acceleration_Body_mps() const {
    return _q_nb.toRotationMatrix().transpose() * _acceleration_NED_mps;
}
```

`alpha()`, `beta()`, `rollRate_Wind_rps()`, `alphaDot()`, `betaDot()`, `q_nl()`, `q_ns()`,
`rollRate_rps()`, `pitchRate_rps()`, `headingRate_rps()`, `velocity_Stab_mps()`, `POM()`,
`turnCircle()` are declared but not specified in the current plan. Do not implement these
in Phase C unless they are needed by a test; flag them for a separate design decision.

### Tests

```
Test — velocity_Wind_mps X-component equals airspeed
    Zero wind, straight-and-level, V = [50, 0, 0] NED, Identity q_nw.
    velocity_Wind_mps().x() ≈ 50.0 m/s.

Test — velocity_Body_mps at zero α, β
    q_nw = Identity, q_nb = Identity, V = [50, 0, 0] NED.
    velocity_Body_mps().x() ≈ 50.0 m/s.

Test — acceleration_Wind_mps resolves correctly
    Known q_nw (e.g., 90° rotation), known NED acceleration.
    Verify acceleration_Wind_mps() equals manually rotated vector.
```

---

## Phase D — KinematicState: Position Integration

### WGS84 API (actual, verified against `include/navigation/WGS84.hpp`)

`WGS84_Datum` already provides:

| Method | Purpose |
|--------|---------|
| `latitudeGeodetic_rad()` | getter (not `latitude_rad`) |
| `setLatitudeGeodetic_rad(double)` | setter |
| `longitude_rad()` | getter |
| `setLongitude_rad(double)` | setter |
| `height_WGS84_m()` | getter (not `altitude_m`) |
| `setHeight_WGS84_m(float)` | setter |
| `latitudeRate(double Vnorth)` | returns `dφ/dt` in rad/s |
| `longitudeRate(double Veast)` | returns `dλ/dt` in rad/s |

### Implement `latitudeRate_rps()` and `longitudeRate_rps()` in `KinematicState`

Delegate to `WGS84_Datum`:
```cpp
double KinematicState::latitudeRate_rps() const {
    return _positionDatum.latitudeRate(_velocity_NED_mps(0));
}

double KinematicState::longitudeRate_rps() const {
    return _positionDatum.longitudeRate(_velocity_NED_mps(1));
}
```

### Position integration in `step()` (add after velocity update)

```cpp
const float height_prev = _positionDatum.height_WGS84_m();

_positionDatum.setLatitudeGeodetic_rad(
    _positionDatum.latitudeGeodetic_rad() + latitudeRate_rps() * dt);
_positionDatum.setLongitude_rad(
    _positionDatum.longitude_rad() + longitudeRate_rps() * dt);
// NED convention: positive D is down, so altitude increases when V_D < 0
_positionDatum.setHeight_WGS84_m(
    height_prev - 0.5f * (_velocity_NED_mps(2) + prev_vel_D) * dt);
```

Where `prev_vel_D` is `_velocity_NED_mps(2)` saved before the velocity update.

### Tests

```
Test — northward velocity increases latitude
    V = [50, 0, 0] NED, dt = 1 s.
    New latitude > original latitude; magnitude matches latitudeRate * dt.

Test — climbing reduces height (V_D < 0 → altitude increases)
    V = [0, 0, -10] NED (climbing), dt = 1 s.
    height_WGS84_m increases by ≈ 10 m.

Test — zero velocity produces no position change
    V = 0, any dt. Position unchanged.
```

---

## Phase E — LoadFactorAllocator

### New files

- `include/aerodynamics/LoadFactorAllocator.hpp`
- `src/aerodynamics/LoadFactorAllocator.cpp`
- `test/LoadFactorAllocator_test.cpp`

### Interface

```cpp
struct LoadFactorInputs {
    float n;         // commanded normal load factor (g)
    float n_y;       // commanded lateral load factor (g)
    float q_inf;     // dynamic pressure (Pa)
    float thrust_n;  // thrust magnitude (N)
    float mass_kg;   // aircraft mass (kg)
};

struct LoadFactorOutputs {
    float alpha_rad;
    float beta_rad;
    bool  stall;     // true when α demand exceeds C_L ceiling
};

class LoadFactorAllocator {
public:
    explicit LoadFactorAllocator(const LiftCurveModel& liftCurve,
                                 float S_ref_m2,
                                 float cl_y_beta);   // C_Yβ < 0
    LoadFactorOutputs solve(const LoadFactorInputs& in);
    void reset(float alpha0_rad = 0.0f, float beta0_rad = 0.0f);

private:
    const LiftCurveModel& _lift;
    float _S;
    float _cl_y_beta;
    float _alpha_prev;
    float _beta_prev;
    static constexpr int   kMaxIter = 20;
    static constexpr float kTol     = 1e-6f;
};
```

### Implicit equations

**Normal (α):** `f(α) = q·S·C_L(α) + T·sin(α) − n·m·g = 0`
`f'(α) = q·S·C_L'(α) + T·cos(α)`

**Lateral (β):** `g(β) = q·S·C_Yβ·β − T·cos(α)·sin(β) − n_y·m·g = 0`
`g'(β) = q·S·C_Yβ − T·cos(α)·cos(β)` (always negative → unique root)

### α solver algorithm

1. Predictor: `α₀ = α_prev + δn·m·g / f'(α_prev)`
2. Newton iteration: `α_{k+1} = α_k − f(α_k)/f'(α_k)` (up to `kMaxIter`)
3. Fold guard: if `f'(α) → 0`, set `stall = true`, clamp `α = α_sep`
4. Store `α_prev = α`

### β solver algorithm

1. Predictor: `β₀ = β_prev + δn_y·m·g / g'(β_prev)`
2. Newton: `β_{k+1} = β_k − g(β_k)/g'(β_k)` (uses `T·cos(α)` from solved α)
3. Store `β_prev = β`

### Tests

```
Test — zero demand produces zero angles
    n=0, n_y=0, T=0 → α=0, β=0.

Test — small n, linear region, T=0
    Analytic: α ≈ n·m·g / (q·S·C_Lα) for small α in linear region.

Test — stall flag raised when n exceeds ceiling
    n > (C_L,sep·q·S + T) / (m·g) → stall = true.

Test — monotonically increasing n stays on pre-stall branch
    Step n up from 0 in small increments; verify α_prev tracking avoids jump to post-stall branch.

Test — lateral, T=0
    Analytic: β = n_y·m·g / (q·S·C_Yβ).
```

**Note:** The `LiftCurveModel` interface used in Phase E (`alphaPeak()`, `alphaTrough()`,
`evaluate()`, `derivative()`) matches the completed Phase A implementation.

---

## Build Notes

To build and run tests (DLL PATH must include ucrt64):
```bash
PATH="/c/msys64/ucrt64/bin:/c/Program Files/CMake/bin:$PATH" cmake --build build
PATH="/c/msys64/ucrt64/bin:$PATH" ./build/test/liteaerosim_test.exe
```

New `.cpp` source files are picked up automatically by `GLOB_RECURSE` in
`src/CMakeLists.txt`. New `_test.cpp` files are picked up by `test/CMakeLists.txt`.
No CMake edits required when adding files under `src/` or `test/`.
