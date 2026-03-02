# Equations of Motion — Implementation Notes

Implementation decisions for the EOM subsystem. For algorithm design and math, see
[docs/algorithms/equations_of_motion.md](../algorithms/equations_of_motion.md).

---

## Files

| File | Role |
|------|------|
| `include/KinematicState.hpp` | Kinematic state interface |
| `src/KinematicState.cpp` | KinematicState implementation |
| `include/aerodynamics/LiftCurveModel.hpp` | 5-region lift curve interface |
| `src/aerodynamics/LiftCurveModel.cpp` | LiftCurveModel implementation |
| `include/aerodynamics/LoadFactorAllocator.hpp` | α/β Newton solver interface |
| `src/aerodynamics/LoadFactorAllocator.cpp` | LoadFactorAllocator implementation |

---

## KinematicState

### Wind Frame Quaternion (`_q_nw`)

`_q_nw` (Wind-to-NED rotation) is stored as `Eigen::Quaternionf`.  It is propagated each
`step()` by composing two rotations applied in order:

1. **Path curvature** (`diff_rot_n`): differential rotation in NED from the velocity
   direction change, computed via `setFromTwoVectors(v_prev, v_new)`.
2. **Roll** (`roll_delta`): rotation about Wind X by `rollRate_Wind_rps * dt`.

```cpp
_q_nw = (diff_rot_n * _q_nw * roll_delta).normalized();
```

Constructor 1 accepts an initial `q_nw` and stores it directly.  Constructor 2 (which
takes `q_nb` directly) derives both α and β exactly from the body-frame velocity
projection, then computes `_q_nw`:

```
// body-frame velocity components (u forward, v right, w down)
v_body = C_BN * v_NED

// exact inversion of  u = V cosα cosβ,  v = V cosα sinβ,  w = V sinα
alpha = atan2(w, sqrt(u^2 + v^2))
beta  = atan2(v, u)

// q_wb = Ry(alpha) * Rz(-beta)  →  q_wb^{-1} = Rz(beta) * Ry(-alpha)
_q_nw = (q_nb * Rz(beta) * Ry(-alpha)).normalized()
```

Below the small-V threshold (0.1 m/s), α and β default to 0 and `_q_nw = q_nb`.

### Wind Representation

The 2D wind input (`windSpeed_mps`, `windDirFrom_rad`) is converted to a 3D NED vector at
the top of `step()` (and in Constructor 1).  `windDirFrom_rad` is the direction the wind
blows FROM: a north wind (`dir = 0`) gives `_wind_NED_mps = [-windSpeed, 0, 0]`.

```cpp
_wind_NED_mps = -windSpeed_mps
              * Vector3f(cos(windDirFrom_rad), sin(windDirFrom_rad), 0.f);
```

The vertical component is always zero in the current model; 3D wind requires a separate
`_wind_NED_mps` input (see roadmap).

### Position Integration

Position integration runs inside `step()` after the velocity update:

- **Latitude / longitude** — forward Euler using the post-step velocity and `WGS84_Datum`
  rate helpers.
- **Altitude** — trapezoidal rule averaging the pre- and post-step vertical velocity,
  negated because NED `V_D > 0` is descent.

```cpp
const float height_prev_m = _positionDatum.height_WGS84_m();
_positionDatum.setLatitudeGeodetic_rad(
    _positionDatum.latitudeGeodetic_rad() + latitudeRate_rps() * dt);
_positionDatum.setLongitude_rad(
    _positionDatum.longitude_rad() + longitudeRate_rps() * dt);
_positionDatum.setHeight_WGS84_m(
    height_prev_m - 0.5f * (velocity_NED_mps_prev(2) + _velocity_NED_mps(2)) * dt);
```

#### WGS84_Datum API

| Method | Purpose |
|--------|---------|
| `latitudeGeodetic_rad()` / `setLatitudeGeodetic_rad(double)` | Geodetic latitude |
| `longitude_rad()` / `setLongitude_rad(double)` | Longitude |
| `height_WGS84_m()` / `setHeight_WGS84_m(float)` | WGS84 ellipsoidal altitude |
| `latitudeRate(double Vnorth)` | Returns dφ/dt (rad/s) |
| `longitudeRate(double Veast)` | Returns dλ/dt (rad/s) |

### Derived Quantities

All implemented as rotation-matrix products on stored state:

```
velocity_Wind_mps()      = C_WN * (v_NED − wind_NED)
velocity_Body_mps()      = C_BN * v_NED
acceleration_Wind_mps()  = C_WN * a_NED
acceleration_Body_mps()  = C_BN * a_NED
latitudeRate_rps()       = _positionDatum.latitudeRate(v_NED[0])
longitudeRate_rps()      = _positionDatum.longitudeRate(v_NED[1])
```

---

## LoadFactorAllocator

### Newton Solver Guards

Two guards prevent divergence when the demanded load factor exceeds the pre-stall ceiling:

**Overshoot guard** (inside the Newton loop): if a step would move α past `alphaPeak()`,
break immediately, set `stall = true`, `alpha = alphaPeak()`.  This prevents oscillation
across the CL peak when no pre-stall solution exists — without this guard the solver
alternates between the linear and post-peak quadratic slopes indefinitely.

**Fold guard**: if `|f'(α)| < kTol`, the derivative has vanished (CL' = 0 at the stall
vertex, T ≈ 0).  Same action: `stall = true`, `alpha = alphaPeak()`.

The β solver's derivative `g'(β) = q·S·C_Yβ − T·cos(α)·cos(β)` is always ≤ 0 for
C_Yβ < 0, guaranteeing a unique root and no fold condition.

### Warm-Starting

`_alpha_prev` and `_beta_prev` persist between `solve()` calls.  Starting Newton from
the previous solution keeps the solver on the same branch across small load-factor
increments and typically converges in 2–4 iterations.  Call `reset()` before a
discontinuous change in demand.

---

## Test Coverage

| Suite | Tests | File |
|-------|-------|------|
| KinematicState | 15 | `test/KinematicState_test.cpp` |
| LiftCurveModel | 16 | `test/LiftCurveModel_test.cpp` |
| LoadFactorAllocator | 5 | `test/LoadFactorAllocator_test.cpp` |

---

## Build

New `.cpp` files under `src/` and new `*_test.cpp` files under `test/` are picked up
automatically by `GLOB_RECURSE`; no CMake edits are needed when adding files in those
directories.

On Windows with the ucrt64 toolchain, the ucrt64 DLLs must be in PATH:

```bash
PATH="/c/msys64/ucrt64/bin:/c/Program Files/CMake/bin:$PATH" cmake --build build
PATH="/c/msys64/ucrt64/bin:$PATH" ./build/test/liteaerosim_test.exe
```
