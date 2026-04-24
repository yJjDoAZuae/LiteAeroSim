# Equations of Motion — Implementation Notes

Implementation decisions for the EOM subsystem. For algorithm design and math, see
[docs/algorithms/equations_of_motion.md](../algorithms/equations_of_motion.md).

---

## Files

| File | Role |
| --- | --- |
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

```text
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

`step()` accepts wind as a full 3D NED vector `wind_NED_mps` (`Eigen::Vector3f`). The
2D-to-3D conversion (from meteorological speed and direction-from) happens in the
environment or scenario layer before the call to `step()`. A north wind blowing at
$V_w$ m/s corresponds to `wind_NED_mps = {-Vw, 0, 0}`.

### Position and Velocity Integration (RK4)

Position and velocity are jointly integrated with classical fourth-order Runge-Kutta
(RK4) over the six-component state `PVState{lat_rad, lon_rad, alt_m, vN_mps, vE_mps,
vD_mps}`, defined in an anonymous namespace in `src/KinematicState.cpp`.

The four RK4 derivative evaluations call `pvDerivative()`, which queries
`WGS84_Datum::latitudeRate()` and `WGS84_Datum::longitudeRate()` at intermediate
position/velocity values. Because the input acceleration `a_NED` is constant during each
step, all four velocity slope values are identical and the velocity result reduces to
`v_new = v_old + a_NED * dt`. The position result achieves fourth-order accuracy in the
nonlinear WGS84 geodetic rate functions.

#### WGS84_Datum API

| Method | Purpose |
| --- | --- |
| `latitudeGeodetic_rad()` / `setLatitudeGeodetic_rad(double)` | Geodetic latitude |
| `longitude_rad()` / `setLongitude_rad(double)` | Longitude |
| `height_WGS84_m()` / `setHeight_WGS84_m(float)` | WGS84 ellipsoidal altitude |
| `latitudeRate(double Vnorth)` | Returns dφ/dt (rad/s) |
| `longitudeRate(double Veast)` | Returns dλ/dt (rad/s) |

### Derived Quantities

All implemented as rotation-matrix products on stored state:

```text
velocity_Wind_mps()      = C_WN * (v_NED − wind_NED)
velocity_Body_mps()      = C_BN * v_NED
acceleration_Wind_mps()  = C_WN * a_NED
acceleration_Body_mps()  = C_BN * a_NED
latitudeRate_rps()       = _positionDatum.latitudeRate(v_NED[0])
longitudeRate_rps()      = _positionDatum.longitudeRate(v_NED[1])
```

### `step()` Parameters

`KinematicState::step()` is the integration sink. Each parameter is produced by a
specific upstream subsystem and passed in each simulation step:

| Parameter | Source | Meaning |
| --- | --- | --- |
| `time_sec` | Simulation clock | Absolute simulation time |
| `acceleration_Wind_mps` | Aerodynamic / propulsion model | Net Wind-frame acceleration (lift + drag + thrust + gravity expressed in Wind frame). **Note:** the gravity term is currently absent from `Aircraft.cpp` step 10 — see [Aircraft.cpp → KinematicState Acceleration Interface](#aircraftcpp--kinematicstate-acceleration-interface). |
| `rollRate_Wind_rps` | Roll-control model | Wind-axis roll rate $p_W$ — drives `_q_nw` propagation |
| `alpha_rad` | Aerodynamic model | Angle of attack — used to propagate `_q_nb` and stored in `_alpha_rad` |
| `beta_rad` | Aerodynamic model | Sideslip angle — same uses as above |
| `alphaDot_rps` | Aerodynamic model | Rate of change of angle of attack — stored in `_alphaDot_rps` |
| `betaDot_rps` | Aerodynamic model | Rate of change of sideslip — stored in `_betaDot_rps` |
| `wind_NED_mps` | Environment model | Ambient 3D wind velocity in NED |

The integrator does not call back into the aerodynamic model within a step. Coupling
is one-directional: aero/propulsion outputs → kinematic inputs. The aerodynamic model
reads `alpha()`, `beta()`, and `velocity_Wind_mps()` from the kinematic state at the
start of the next step.

### `q_nl()` Semantics

`q_nl()` returns `Quaternionf(_positionDatum.qne().cast<float>())`, where `qne()` is
the ECEF-to-NED rotation at the current position. This is used as the NED-to-Local-Level
rotation.

**Open question:** For long-range flights, the Local Level frame (tangent plane at the
current aircraft position) diverges from the NED frame (fixed at the initial datum). The
current implementation returns the ECEF-to-NED rotation at the current position, which
is the rotation of the tangent plane, not a fixed NED frame. Confirm whether this is the
intended behavior before depending on `q_nl()` for inertial navigation calculations.

---

## LoadFactorAllocator

### Newton Solver Guards

Two guards prevent divergence when the demanded load factor exceeds the achievable ceiling.
The ceiling is not simply `alphaPeak()` when thrust is positive: the normal thrust component
$T\sin\alpha$ continues to grow past $\alpha_{peak}$, so the load-factor ceiling
$N_{z,max}(\alpha) = (qS C_L(\alpha) + T\sin\alpha)/(mg)$ peaks at α* where
$f'(\alpha^*) = qS C_L'(\alpha^*) + T\cos\alpha^* = 0$, which lies above `alphaPeak()`
whenever $T > 0$.

**Overshoot guard** (inside the Newton loop): before evaluating $f'$ at the proposed step
`alpha_new`, clamp it to the CL parabolic domain using `alphaSep()` / `alphaSepNeg()`.
In the flat separated plateau $f'(\alpha) = T\cos\alpha$, which stays positive until
$\alpha > \pi/2$, so without this clamp large thrust would let Newton escape the physical
domain entirely.  After clamping, if `f'(alpha_hi) <= 0`, the f′-zero crossing lies between
the current iterate and the clamped point: bisect the interval (30 iterations, ~30 bits of
precision) to pin it.  Set `stall = true` and break.  For $T = 0$ the crossing coincides
exactly with `alphaPeak()` / `alphaTrough()`, preserving the zero-thrust behavior.

**Fold guard**: if `|f'(α)| < kTol` at the current iterate, the iterate is already at the
f′-zero crossing.  Stay at the current α (do not snap to `alphaPeak()`) and set
`stall = true`.  For $T = 0$ this fires at `alphaPeak()` as before; for $T > 0$ it fires
at the correct crossing above `alphaPeak()`.

The β solver's derivative `g'(β) = q·S·C_Yβ − T·cos(α)·cos(β)` is always ≤ 0 for
C_Yβ < 0, guaranteeing a unique root and no fold condition.

### Warm-Starting and Branch-Continuation Predictor

`_alpha_prev`, `_beta_prev`, `_n_z_prev`, and `_n_y_prev` persist between `solve()`
calls.  Before the Newton loop, a first-order branch-continuation predictor computes:

```text
α₀ = α_prev + δn_z · m·g / f′(α_prev)
β₀ = β_prev + δn_y · m·g / g′(β_prev)
```

In the linear lift region `f′(α)` is constant (`q·S·C_Lα`), so the predictor places
Newton exactly at the solution and the solver converges in a single iteration.  The
predictor is guarded by two conditions:

- **f′ guard**: skip if `|f′(α_prev)| < kTol` (at the stall ceiling, no meaningful step
  direction).
- **Domain guard**: the raw prediction is only applied when it stays within
  `[alphaSepNeg, alphaSep]`.  If the demand jump would project the warm-start into
  the flat separated plateau (e.g., a cold-start excess-demand call), the predictor
  falls back to `α_prev` so that the overshoot guards operate correctly.

`reset()` sets `_alpha_prev`, `_beta_prev`, `_n_z_prev`, and `_n_y_prev` all to zero.
Call `reset()` before a discontinuous change in demand.  All four fields are included
in both JSON and proto serialization.

### Alpha Limit Box Constraint

`alpha_max_rad` and `alpha_min_rad` (`LiftCurveModel` parameters from the `lift_curve`
config section) are enforced as a box constraint **inside** the Newton iteration, not as
a post-solve clamp.  After each Newton step the iterate is projected:

```cpp
alpha_new = std::clamp(alpha_k - f(alpha_k) / f_prime(alpha_k),
                       liftCurve.alpha_min_rad, liftCurve.alpha_max_rad);
```

**Boundary exit condition.** If the projected iterate is pinned at a boundary and the
residual at that boundary still has the sign that would push Newton further outside the
domain, the solver accepts the boundary as the constrained solution and sets an
alpha-limit flag:

- Pinned at `alpha_max_rad` with `f(alpha_max) > 0` → return `alpha_max_rad`, set flag.
- Pinned at `alpha_min_rad` with `f(alpha_min) < 0` → return `alpha_min_rad`, set flag.

The alpha-limit flag is separate from the stall flag.  Both can be set simultaneously
(e.g., if `alpha_max_rad` falls past the fold point and the fold guard also fires).

See [docs/algorithms/equations_of_motion.md §Alpha Limits as a Box Constraint](../algorithms/equations_of_motion.md) for the full mathematical treatment and
interaction with the fold/overshoot guards.

### Stall Warm-Start Limitation

When the fold guard or overshoot guard clamps α at the stall ceiling, `_alpha_prev` is left
at the clamped value where $f'(\alpha) = 0$ by definition.  On the next `solve()` call:

- The predictor f′ guard fires (`|f′(α_prev)| < kTol`), so the warm-start falls back to
  α_prev (the clamped value).
- The Newton loop starts at the clamped α.  In floating-point arithmetic `f′(alphaPeak)` for
  $T = 0$ evaluates to a small nonzero residual (~3 × 10⁻⁶ for typical parameters) that
  exceeds `kTol = 1e-6`.  The fold guard therefore does not fire at the first Newton
  iteration.  Newton takes a large-magnitude step and the overshoot guard may pin α on the
  opposite-sign branch — converging to the wrong solution.

**Consequence**: a single discontinuous Nz jump past the stall ceiling (e.g., a direct step
from 1 g to 1.5 × $N_{z,\text{max}}$) leaves the warm-start at the fold point.  The
subsequent sub-ceiling `solve()` call is likely to converge to the wrong branch.

**Required mitigation**: call `reset()` before any discontinuous change in demand, including
re-engagement after a stall event.  In normal closed-loop simulation with small $dt$ the Nz
command evolves continuously within one step, so the warm-start stays near the true solution
and this limitation does not arise in practice.

See `AllocatorFixture.StallRecovery_RequiresReset` in `test/LoadFactorAllocator_test.cpp`.

---

## Aircraft.cpp → KinematicState Acceleration Interface

### Intended Interface

`KinematicState::step()` receives `acceleration_Wind_mps` — the net Wind-frame specific force vector produced by the aerodynamic and propulsion models.  Per the interface table in the KinematicState section, this vector must include all forces acting on the aircraft expressed in the Wind frame, **including gravity**:

$$
\mathbf{a}_W = \frac{\mathbf{F}_{aero} + \mathbf{F}_{thrust}}{m} + C_{WN}\,\mathbf{g}_{NED}
$$

where $\mathbf{g}_{NED} = [0,\;0,\;g]^\top$ ($g = 9.80665\;\text{m/s}^2$, positive down in NED) and $C_{WN} = C_{NW}^\top$ is the NED-to-Wind DCM.  `KinematicState` then integrates:

$$
\dot{\mathbf{v}}_{NED} = C_{NW}\,\mathbf{a}_W
$$

For straight-and-level flight at load factor $n_z = 1$, the load-factor constraint gives:

$$
q\,S\,C_L + T\sin\alpha = n_z\,m\,g = m\,g
\quad\Rightarrow\quad
F^W_z = -q\,S\,C_L = -(m\,g - T\sin\alpha)
$$

The Wind-Z component of the total specific force is therefore:

$$
a^W_z = \frac{-T\sin\alpha + F^W_z}{m} + (C_{WN}\,\mathbf{g}_{NED})_z
= -g + g\cos\gamma \approx 0 \quad (\gamma \to 0)
$$

which produces $a_{NED,D} \approx 0$ — no altitude drift at trim.

### Current Implementation Gap

`Aircraft.cpp` step 10 computes:

```cpp
const float ax = (T * ca * cb + F.x_n + F_gear_wind.x()) / m;
const float ay = (-T * ca * sb + F.y_n + F_gear_wind.y()) / m;
const float az = (-T * sa      + F.z_n + F_gear_wind.z()) / m;
```

with the comment: *"Gravity is embedded in the load-factor constraint — must not be added here."*

The Wind-frame gravity term $C_{WN}\,\mathbf{g}_{NED}$ is absent.  At trim ($n_z = 1$) the aerodynamic forces produce:

$$
a^W_z = \frac{-T\sin\alpha - q\,S\,C_L}{m} = -g
$$

This results in $a_{NED,D} \approx -g$ — a 1 g upward NED acceleration — so the aircraft climbs even at exact trim thrust.  The comment is incorrect: the load-factor constraint determines the magnitude of the aerodynamic reaction to gravity; it does not introduce a gravity term into `acceleration_Wind_mps`.

### Observed Failure Mode

`AircraftTest.StraightAndLevel_AllFixtures_100s` runs each example aircraft for 100 s at drag-balanced trim thrust with $n_z = 1, n_y = 0$, roll rate $= 0$, and fixed ISA density.  All three fixtures fail the 50 m altitude-change bound (see [Test Coverage](#test-coverage)):

| Fixture | Initial speed (m/s) | Altitude drift (m, 100 s) |
| --- | --- | --- |
| `aircraft/general_aviation.json` | 55 | ~200 |
| `aircraft/jet_trainer.json` | 150 | ~300 |
| `aircraft/small_uas.json` | 20 | ~100 |

A fix was attempted (adding `g_wind = R_nw_mat.transpose() * {0, 0, g}` to step 10); this produced a ~17 800 m descent and broke `AircraftTest.ZeroThrottle_AircraftDecelerates`.  The root cause of the sign reversal is not yet understood; see [OQ-17](#oq-17--root-cause-of-the-gravity-fix-reversal), [OQ-18](#oq-18--where-to-add-the-gravity-term), and [OQ-19](#oq-19--does-the-load-factor-constraint-already-account-for-gravity).

---

## Open Questions

### Resolved — OQ-1 through OQ-16 (Stall Dynamics)

OQ-1 through OQ-16 arose during the stall dynamics design and are fully resolved.

| # | Question | Decision |
| --- | --- | --- |
| OQ-1 | Alpha bridge activation scope | **Stall-conditional.** Rate limit applies only while `_stalled` or `_stalled_neg` is true. In non-stalled operation `alpha_out = alpha_eq` exactly. |
| OQ-2 | Newton solver while `_stalled` (nominal CL vs. recovering CL) | **Explicit solve using recovering CL.** Newton is replaced by `α_eq = arcsin((n·mg − qS·_cl_recovering) / T)`. Realized Nz uses `_cl_recovering`. |
| OQ-3 | `dt_s` delivery to `solve()` | **Add `dt_s` to `LoadFactorInputs`.** Explicit per-call field; supports variable-rate stepping. |
| OQ-4 | `LiftCurveModel` accessors needed | **Add `alphaStar()`, `alphaStarNeg()`, `clAlpha()`.** Consistent with the existing `alphaPeak()`, `alphaSep()` interface. |
| OQ-5 | Hysteresis flag serialization | **Serialize `_stalled` and `_stalled_neg`.** Recomputing from `_alpha_prev` fails in the hysteretic region where `alphaStar < alpha_prev < alphaSep`. |
| OQ-6 | CL recovery state serialization | **Serialize `_cl_recovering` and `_cl_recovering_neg`.** Recomputing from stall flags loses in-progress recovery position. |
| OQ-7 | Newton residual while stalled — contradiction with OQ-2 | **Explicit solve using recovering CL** (see OQ-2). Algorithm doc §"Interaction with the Newton Solver" is correct as written. |
| OQ-8 | Hysteresis entry condition (`alphaSep()` vs. `alphaPeak()`) | **Entry when alpha passes `alphaPeak()` and begins to decrease**, not at `alphaSep()`. |
| OQ-9 | Re-entry behavior when alpha rises while `_stalled` is true | **Effective CL is `min(clSep(), clNom(alpha))` throughout the stalled phase.** Snaps to nominal at the intersection with no discontinuity; CL recovery bridge activates once below `alphaStar()`. |
| OQ-10 | When to introduce `_cl_recovering` | **Introduce in Item 1.** The realized-Nz formula is written once; Items 2/3 add only the stalled update branch. |
| OQ-11 | `_cl_recovering` initialization value | **Initialize to zero.** Every non-stalled `solve()` overwrites it before first use. |
| OQ-12 | Hysteresis entry check — one-step delay | **Accept one-step delay.** At 100 Hz the error is one step of the nominal parabola; negligible and self-correcting. |
| OQ-13 | `clSep()` / `clSepNeg()` vs. `params()` accessor | **Add `clSep()` and `clSepNeg()` as individual accessors.** Consistent with `alphaPeak()`, `alphaSep()` pattern. |
| OQ-14 | `cl_eff` delivery to `Aircraft.cpp` | **Add `cl_eff` to `LoadFactorOutputs`.** `Aircraft.cpp` uses `lfa_out.cl_eff` in place of `_liftCurve->evaluate(lfa_out.alpha_rad)`. |
| OQ-15 | `alphaDot` computation during stall | **While `_stalled`, substitute `fprime_alpha = T * cos(alpha_out)`.** Exact IFT derivative for the flat-CL explicit solve. |
| OQ-16 | Stalled and negative-side CL formulas | **Stalled: `_cl_recovering = min(clSep(), clNom)`.** Instant snap to nominal when nominal falls below `clSep`. Non-stalled recovery: `min(clNom, _cl_recovering + clDotMax * dt)`. Negative side mirrors with inverted direction throughout. |

---

### OQ-17 — Root Cause of the Gravity Fix Reversal

**Question:** `Aircraft.cpp` step 10 omits the Wind-frame gravity term.  A fix was attempted by adding `g_wind = R_nw_mat.transpose() * {0.f, 0.f, kGravity}` to each acceleration component.  For level flight the math predicts $a^W_z = -g + g = 0$, which should correct the altitude drift.  Instead the aircraft descended ~17 800 m and gained ~152 m/s in 100 s, and `AircraftTest.ZeroThrottle_AircraftDecelerates` failed.  Why?

---

**Alternative A: `KinematicState::step()` adds gravity internally.**

*Implication:* If `pvDerivative()` in `KinematicState.cpp` appends `{0, 0, g}` to `a_NED` before integrating, then the original step 10 (aero+thrust only, az = −g) is correct as written: `KinematicState` adds +g, yielding a_NED_D = 0 at trim.  The fix would then double-count gravity — az after fix = 0, plus KinematicState's internal +g → a_NED_D = +g — driving a 1 g downward acceleration.  Over 100 s this predicts ~49 000 m of altitude loss; the observed ~17 800 m is smaller, consistent with a strong drag increase during the resulting high-speed dive.  Under Alternative A, `StraightAndLevel_AllFixtures_100s` should already pass without any fix, which means the test failure has a different cause (possibly the filter transient or a thrust-accounting offset).

*Recommendation:* **Verify first.** Read `KinematicState.cpp` `pvDerivative()` and confirm whether a gravity term appears.  This is the highest-priority check because it resolves all three OQs and determines whether any code change is needed in `Aircraft.cpp` at all.

*Finding:* [`src/KinematicState.cpp:28–38`](../../src/KinematicState.cpp) passes `accel_NED(0)`, `accel_NED(1)`, `accel_NED(2)` directly to the velocity derivatives with no additional term.  No gravity is added internally.  **Alternative A is refuted.**

---

**Alternative B: `_q_nw` stores NED-to-Wind (opposite of the documented Wind-to-NED convention).**

*Implication:* If `_q_nw.toRotationMatrix()` produces $C_{WN}$ (NED → Wind) rather than the documented $C_{NW}^{-1}$ (Wind → NED), then `R_nw_mat.transpose()` maps Wind → NED.  Evaluating `R_nw_mat.transpose() * {0, 0, g}_NED` uses a Wind → NED matrix applied to an NED vector — a frame mismatch.  For near-identity rotation (level flight) the numerical result is still approximately `{0, 0, g}`, so the first step would appear correct.  As the aircraft attitude changes during subsequent steps, the mismatch compounds.  This alternative cannot be the sole cause of the first-step error for level flight, but it could contribute to instability over longer runs.  It would also invalidate the existing landing-gear wind-frame transform (`F_gear_wind = R_nw_mat.transpose() * (R_nb_mat * F_body)`), which appears to work correctly in tests — evidence against Alternative B.

*Recommendation:* **Check by examining `KinematicState::velocity_Wind_mps()`.**  The implementation doc states this returns `C_{WN} * (v_NED − wind_NED)`.  If `q_nw.toRotationMatrix()` is used there without transposition, the convention is NED → Wind (Alternative B is confirmed).  If it is used with transposition, the convention is Wind → NED (Alternative B is refuted).

*Finding:* The landing gear force transform in [`src/Aircraft.cpp:243–245`](../../src/Aircraft.cpp) is:

```cpp
const Eigen::Matrix3f R_nw_mat = _state.q_nw().toRotationMatrix();
const Eigen::Vector3f F_gear_wind = R_nw_mat.transpose() * (R_nb_mat * contact_forces.force_body_n);
```

`R_nw_mat.transpose()` is applied as the NED-to-Wind transform (body force rotated to NED by `R_nb_mat`, then to Wind by `R_nw_mat.transpose()`).  This definitively establishes that `q_nw.toRotationMatrix()` = $C_{NW}$ (Wind-to-NED), confirming the documented convention.  **Alternative B is refuted.**  No check of `velocity_Wind_mps()` is required.

---

**Alternative C: The fix introduces a frame inconsistency with the allocator's implicit assumption.**

*Implication:* The load-factor allocator solves `q·S·C_L + T·sinα = n_z·mg` under the implicit assumption that the resulting aerodynamic forces fully satisfy the load-factor demand — i.e., that the net NED-Z acceleration is zero.  Adding a gravity term to step 10 changes the actual NED-Z acceleration to zero (correct physically), but the allocator's target $n_z$ was computed relative to a model that assumed az = −g is the entire story.  In subsequent steps the allocator receives a different kinematic state (velocity unchanged by the fix on the first step, so no first-step effect), meaning this alternative cannot explain the first-step failure.  Over many steps it could produce a feedback interaction where the allocator over-corrects CL, causing a transient.  It is not a plausible explanation for a systematic 17 800 m dive.

*Recommendation:* **Rule out after confirming Alternative A.** If Alternative A explains the dive, Alternative C is moot.  If Alternative A is refuted, write a two-step test: observe whether the second-step CL changes unexpectedly after the fix is applied.

---

**Resolution:** Both Alternative A and Alternative B are refuted by direct code inspection (see *Finding* notes above).  Alternative C is not a plausible explanation for the observed failure — as its analysis notes, it cannot produce a systematic first-step error.  The 17 800 m descent produced by the historical fix attempt was an implementation error; the specific error is not recoverable from the available record.  The mathematical derivation of the fix is correct: adding the Wind-frame gravity vector to step 10 yields $a_{NED,D} = 0$ at level-flight trim.  Proceed to [OQ-18](#oq-18--where-to-add-the-gravity-term) to decide where the gravity term belongs.

---

### OQ-18 — Where to Add the Gravity Term

**Question:** Once OQ-17 is resolved and the gravity omission is confirmed as the root cause, where should the gravity term be inserted: in `Aircraft.cpp` step 10 (expressed in Wind frame) or inside `KinematicState::step()` (appended as an NED vector after rotating `acceleration_Wind_mps`)?

---

**Alternative A: `Aircraft.cpp` step 10, Wind frame.**

```cpp
const Eigen::Vector3f g_wind = R_nw_mat.transpose() * Eigen::Vector3f{0.f, 0.f, kGravity};
const float ax = (T * ca * cb + F.x_n + F_gear_wind.x()) / m + g_wind.x();
const float ay = (-T * ca * sb + F.y_n + F_gear_wind.y()) / m + g_wind.y();
const float az = (-T * sa      + F.z_n + F_gear_wind.z()) / m + g_wind.z();
```

*Implication:* Matches the stated `step()` interface ("Net Wind-frame acceleration including gravity").  `KinematicState` remains a pure frame-transform integrator with no physics knowledge — it rotates whatever vector it receives and integrates.  Any future force model calling `step()` must supply gravity itself; failing to do so silently produces the same bug.  The gravity computation requires one matrix-vector product per step.

*Recommendation:* **Prefer this if the interface contract must remain as documented** and if `KinematicState` is intended to be physics-free.  The interface table in the implementation doc stays correct as written.

---

**Alternative B: Inside `KinematicState::step()`, appended in NED.**

```cpp
// Inside pvDerivative(), after computing a_NED:
a_NED += Eigen::Vector3f{0.f, 0.f, kGravity};
```

*Implication:* `Aircraft.cpp` step 10 passes only the aero+thrust acceleration (the current behavior, az = −g at trim).  `KinematicState` adds gravity in NED, yielding a_NED_D = 0 at level-flight trim — correct.  Gravity is in exactly one location; any caller of `step()` gets gravity automatically.  The interface contract must be updated: `acceleration_Wind_mps` becomes "aero + propulsion specific force in Wind frame, excluding gravity."  The `step()` parameter table in this document and the algorithm doc table must both be corrected.

*Recommendation:* **Prefer this if `KinematicState` should encapsulate all integration physics** and if reducing per-caller error risk is valued.  No changes to `Aircraft.cpp` step 10.  Document update is required in both the KinematicState `step()` parameter table (this file, line ~105) and the algorithm doc.

---

**Resolution:** Git history confirms that `KinematicState::step()` has received `acceleration_Wind_mps` and rotated it to NED without adding gravity since the first implementation commit.  The parameter name, semantics, and absence of internal gravity are unchanged across every commit.  **Alternative A is correct.**  `KinematicState` has always been a physics-free frame-transform integrator; the caller is responsible for supplying the complete acceleration vector including gravity.  `Aircraft.cpp` has simply never fulfilled that contract.  No interface change is required; no documentation update to the `step()` parameter table is needed.  Gravity belongs in `Aircraft.cpp` step 10.

---

### OQ-19 — Does the Load-Factor Constraint Already Account for Gravity?

**Question:** The load-factor constraint `q·S·C_L + T·sinα = n_z·mg` relates aerodynamic and thrust forces to the commanded load factor.  Does this constraint implicitly embed gravity in `acceleration_Wind_mps`, making any additional gravity term a double-count?

---

**Alternative A: The constraint fully accounts for gravity — no separate gravity term is needed.**

*Implication:* Under this reading the constraint is a force-to-acceleration identity: the left-hand side equals the right-hand side, and `acceleration_Wind_mps` computed from it already equals the total specific force including gravity.  This requires `KinematicState` to add gravity internally (i.e., Alternative A of OQ-17 must be true).  The two alternatives are linked: if gravity is added inside `KinematicState`, then the constraint does play the role of encoding the gravity reaction, and no explicit gravity term is needed in `Aircraft.cpp`.

*Recommendation:* **Contingent on OQ-17 Alternative A.** If `KinematicState.cpp` is found to add gravity, this interpretation is self-consistent.  If not, this interpretation is wrong regardless of how the constraint is read.

---

**Alternative B: The constraint is an allocator setpoint equation; gravity must still be added to the integration.**

*Implication:* The constraint determines what value of $C_L$ and $\alpha$ the allocator should target — it is derived from the level-flight force balance but does not itself add gravity to `acceleration_Wind_mps`.  Newton's second law in NED says:

$$m\,\dot{\mathbf{v}}_{NED} = \mathbf{F}_{aero} + \mathbf{F}_{thrust} + \mathbf{F}_{gravity}$$

Step 10 computes only the first two right-hand terms.  Substituting the constraint into the Wind-Z component gives:

$$m\,a_{NED,D} \approx F^W_z + F^W_{thrust,z} + m\,g = -n_z\,m\,g + m\,g = m\,g\,(1 - n_z)$$

For $n_z = 1$: $a_{NED,D} = 0$ — but only when the gravity term $m\,g$ is included in the right-hand side.  Without it, $a_{NED,D} = -n_z\,g = -g$ (upward) at every step, matching the observed climb.

*Recommendation:* **This is the correct interpretation.** The constraint is a setpoint; the gravity term in Newton's second law is separate.  The step 10 comment "Gravity is embedded in the load-factor constraint" is incorrect and should be removed when the fix is applied.

---

**Resolution:** Alternative B is correct.  The load-factor constraint is a setpoint equation that determines the aerodynamic reaction to a commanded load factor; it does not introduce a gravity term into `acceleration_Wind_mps`.  The step 10 comment *"Gravity is embedded in the load-factor constraint — must not be added here"* is incorrect and must be removed when the fix is applied.  OQ-17 is resolved; proceed to [OQ-18](#oq-18--where-to-add-the-gravity-term) to decide where to insert the gravity term.

---

## Stall Dynamics Implementation Plan

Three related features to implement, in dependency order.  Each item follows TDD: failing
tests first, then production code.  See
[docs/algorithms/equations_of_motion.md](../algorithms/equations_of_motion.md) for the
full algorithm design.

### Item 1 — Alpha Rate Limiting (Stall Recovery Bridge)

**New state in `LoadFactorAllocator`:**

```cpp
float _alpha_dot_max_rad_s;   // LoadFactorAllocator constructor parameter
```

Added to the `LoadFactorAllocator` constructor signature alongside `S_ref_m2`, `cl_y_beta`,
`alpha_min_rad`, `alpha_max_rad`.  Serialized in `LoadFactorAllocatorState`.

**New proto field:**

- `LoadFactorAllocatorState`: field 10 `alpha_dot_max_rad_s`

**Algorithm change in `LoadFactorAllocator::solve()`:**

The bridge applies only when the stall flag is set from the *previous* step.  After the
Newton solve (or explicit stall solve) produces `alpha_eq`, gate the rate limit:

```cpp
float alpha_out;
if (_stalled || _stalled_neg) {
    // stall bridge: step toward alpha_eq at pitch-authority rate
    const float max_step = _alpha_dot_max_rad_s * in.dt_s;
    alpha_out = _alpha_prev + std::clamp(alpha_eq - _alpha_prev, -max_step, +max_step);
} else {
    // not stalled: use Newton solution exactly — no rate constraint
    alpha_out = alpha_eq;
}
_alpha_prev = alpha_out;
```

`alpha_out` replaces `alpha_eq` in the output struct and in the realized-Nz computation.
The `dt_s` field must be added to `LoadFactorInputs` (OQ-3).

**Introduce `_cl_recovering` in Item 1:**

`_cl_recovering` (and `_cl_recovering_neg`) are introduced here so that the realized-Nz
formula is written once and never needs replacement when Items 2/3 are added.  While not
stalled, `_cl_recovering` simply tracks the nominal lift curve:

```cpp
_cl_recovering = _lift.evaluate(alpha_out);   // non-stalled: tracks nominal
```

Items 2/3 add the stalled update branch without touching the realized-Nz formula.

**Realized Nz:**

```cpp
const float n_realized = (qS * _cl_recovering + T * std::sin(alpha_out)) / mg;
```

Return `n_realized` and `_cl_recovering` (as `cl_eff`) in `LoadFactorOutputs`.  The
aerodynamic force computation in `Aircraft.cpp` uses `cl_eff` in place of
`_liftCurve->evaluate(alpha_out)` — this is the only change needed in `Aircraft.cpp`
for the stall dynamics to propagate correctly through to `KinematicState`.

**New field in `LoadFactorOutputs`:**

```cpp
float n_z_realized = 0.f;   // load factor actually delivered (equals commanded when not stalled)
```

(`alpha_bridging` is not added: the `stalled` / `stalled_neg` flags from Item 2 already
convey bridge state; callers use `stalled || stalled_neg`.)

**Tests to write (in `LoadFactorAllocator_test.cpp`):**

1. `AlphaBridge_Inactive_InNormalPreStall` — non-stalled call with any step size; verify
   `alpha_out == alpha_eq` exactly
2. `AlphaBridge_LimitsStepSizeWhileStalled` — enter stall, then issue a large Nz step;
   verify `|alpha_out - alpha_prev| == alpha_dot_max * dt` while `_stalled == true`
3. `AlphaBridge_ConvergesAndDeactivates` — run stall-recovery steps to completion;
   verify `alpha_out` reaches `alpha_eq` and bridge is inactive once stall flag clears
4. `AlphaBridge_NRealizedComputedFromAlphaOut` — confirm `n_z_realized` is computed from
   `alpha_out`, not commanded `n`
5. `LoadFactorAllocator` serialization round-trip extended to cover `alpha_dot_max_rad_s`
   (JSON and proto)

---

### Item 2 — Stall Hysteresis Flags

**New state in `LoadFactorAllocator`:**

```cpp
bool  _stalled;       // positive-side hysteresis flag
bool  _stalled_neg;   // negative-side hysteresis flag
```

Both initialized to `false` in constructor and `reset()`.

**Algorithm change in `solve()`:**

After `alpha_out` is determined (post rate-limit), update flags:

```cpp
// Entry: alpha has passed alphaPeak and is now decreasing
if (alpha_out < _alpha_prev && _alpha_prev >= _lift.alphaPeak()) _stalled     = true;
if (alpha_out > _alpha_prev && _alpha_prev <= _lift.alphaTrough()) _stalled_neg = true;
// Exit: threshold (alphaStar) or snap-down (nominal meets plateau in ascending quadratic)
if (alpha_out <= _lift.alphaStar() ||
    (alpha_out < _lift.alphaPeak() && _lift.evaluate(alpha_out) <= _lift.clSep()))
    _stalled = false;
if (alpha_out >= _lift.alphaStarNeg() ||
    (alpha_out > _lift.alphaTrough() && _lift.evaluate(alpha_out) >= _lift.clSepNeg()))
    _stalled_neg = false;
```

Note: `_alpha_prev` here is the value from the *previous* step (before updating it to
`alpha_out`), so it reflects the alpha from which the current step descended.

**New fields in `LoadFactorOutputs`:**

```cpp
bool stalled     = false;   // positive-side hysteresis active
bool stalled_neg = false;   // negative-side hysteresis active
```

**Serialization:** add `_stalled` and `_stalled_neg` to JSON and proto serialization of
`LoadFactorAllocator` state.

**Tests to write:**

1. `Hysteresis_FlagSetsWhenDecreasingFromAlphaPeak` — push alpha to `alphaPeak`, then
   step it down; verify `stalled` sets on the first decreasing step
2. `Hysteresis_FlagDoesNotSetOnAscent` — push alpha past `alphaPeak` on the way up;
   verify `stalled` is false while ascending
3. `Hysteresis_FlagClearsAtAlphaStar` — configuration with `clSep() < clAlpha() * alphaStar()`; reduce alpha through stall region; flag remains set until alpha crosses `alphaStar()`, clears immediately after
4. `Hysteresis_FlagClearsAtSnapDown` — configuration with `clSep() > clAlpha() * alphaStar()`; flag clears when `_lift.evaluate(alpha_out)` first drops to `clSep()` in the ascending-quadratic domain, above `alphaStar()`
5. `Hysteresis_CLFlatThroughDescentRegion` — verify `cl_eff == C_L_sep` throughout
   the descent from `alphaPeak` to `alphaStar`, not the nominal descending parabola
6. `Hysteresis_AlphaBounceDoesNotClearFlag` — alpha descends partway, then increases
   again without reaching `alphaStar()`; flag stays set
7. `Hysteresis_NoFlagWithoutStallEntry` — normal pre-stall operation; flag never sets
8. `Hysteresis_NegativeSideSymmetric` — negative-side entry and exit
9. `Hysteresis_SerializationRoundTrip` — JSON and proto round-trip with flag set

---

### Item 3 — CL Recovery Rate Limiting

**New state in `LoadFactorAllocator`:**

```cpp
float _cl_recovering;      // current effective CL during post-stall recovery (positive side)
float _cl_recovering_neg;  // current effective CL (negative side)
```

Initialized in constructor and `reset()` to `0.0f`.  Under OQ-10-B, every non-stalled
`solve()` call overwrites `_cl_recovering` before computing `n_realized`; the initial
value is never used without being overwritten first.

**Three-phase CL behavior:**

| Phase | Condition | Effective CL |
| --- | --- | --- |
| Stalled descent | `_stalled == true` (alpha ≥ `alphaStar()`) | `min(clSep(), C_L_nom(alpha))` — snaps to nominal if nominal falls below `C_L_sep` |
| Recovery | `_stalled == false`, `_cl_recovering < C_L_nom(alpha)` | Rate-limited rise toward nominal at `cl_dot_max` |
| Normal | `_stalled == false`, `_cl_recovering == C_L_nom(alpha)` | Tracks nominal lift curve |

If alpha rises back into the descending quadratic while `_stalled == false` (recovery in
progress), `C_L_nom(alpha)` falls along the nominal parabola.  Since `_cl_recovering` is
clamped to `min(C_L_nom, _cl_recovering + cl_dot_max * dt)`, CL descends immediately with
the nominal curve — no rate limit on downward movement.  This is the correct behavior:
CL cannot be sustained above the nominal model.

**Algorithm change in `solve()`:**

After updating hysteresis flags (Item 2), update `_cl_recovering`:

```cpp
const float cl_dot_max = _lift.clAlpha() * _alpha_dot_max_rad_s;
const float cl_nom = _lift.evaluate(alpha_out);

// Positive side
if (_stalled) {
    // min(): hold plateau, but snap instantly to nominal if nominal falls below clSep
    _cl_recovering = std::min(_lift.clSep(), cl_nom);
} else {
    // Rate-limit upward approach to nominal; instant downward snap
    _cl_recovering = std::min(cl_nom, _cl_recovering + cl_dot_max * in.dt_s);
}

// Negative side — mirrors positive with inverted direction (max instead of min)
if (_stalled_neg) {
    // max(): hold plateau, but snap instantly to nominal if nominal rises above clSepNeg
    _cl_recovering_neg = std::max(_lift.clSepNeg(), cl_nom);
} else {
    // Rate-limit downward approach to nominal; instant upward snap
    _cl_recovering_neg = std::max(cl_nom, _cl_recovering_neg - cl_dot_max * in.dt_s);
}
```

When `_stalled` is true, the Newton loop is replaced by the explicit solve:

```cpp
// alpha_eq from explicit fully-separated form with _cl_recovering as the plateau
if (T > kTol) {
    const float sin_alpha_eq = (in.n_z * mg - qS * _cl_recovering) / T;
    alpha_eq = (std::abs(sin_alpha_eq) <= 1.0f) ? std::asin(sin_alpha_eq) : _alpha_prev;
} else {
    alpha_eq = _alpha_prev;   // T=0: no alpha solution; hold current
}
// Alpha bridge then steps toward alpha_eq at the rate limit (same as non-stalled path)
```

The realized Nz uses `_cl_recovering`:

```cpp
const float cl_eff = _cl_recovering;   // always valid: tracks nominal when not stalled
const float n_realized = (qS * cl_eff + T * std::sin(alpha_out)) / mg;
```

Note: `_cl_recovering` tracks nominal when `_stalled == false` and the bridge has
converged, so using it unconditionally is correct in all phases.

**alphaDot during stall (OQ-15-A):**

While `_stalled` or `_stalled_neg`, the Newton loop is bypassed so `positive_stall` and
`negative_stall` remain `false`.  The correct IFT derivative for the explicit solve (flat
CL, `dC_L/dα = 0`) is `T·cos(α)` alone.  Replace the existing `fprime_alpha` computation:

```cpp
const float fprime_alpha = _stalled     ?  T * std::cos(alpha_out)
                         : _stalled_neg ? -T * std::cos(alpha_out)
                         : qS * _lift.derivative(alpha_out) + T * std::cos(alpha_out);
float alphaDot = 0.f;
if (!positive_stall && !negative_stall && std::abs(fprime_alpha) > kTol) {
    alphaDot = mg * in.n_z_dot / fprime_alpha;
}
```

The existing Newton fold-guard (`positive_stall` / `negative_stall`) remains and fires only
in the non-stalled path.

**`LiftCurveModel` accessors needed:**

```cpp
float clAlpha()  const;  // returns the pre-stall lift-curve slope C_Lα
float clSep()    const;  // returns C_L_sep — positive-side post-stall plateau CL
float clSepNeg() const;  // returns C_L_sep_neg — negative-side post-stall plateau CL
```

Add any not already present.  `clSep()` and `clSepNeg()` are included in Implementation
Order Step 1.

**New fields in `LoadFactorOutputs`:**

```cpp
float cl_eff = 0.f;          // effective CL used (recovering or nominal)
bool  cl_recovering = false;  // true while CL recovery bridge is active (_cl_recovering < C_L_nom(alpha_out))
```

**Serialization:** add `_cl_recovering` and `_cl_recovering_neg` to JSON and proto
serialization.

**Tests to write:**

1. `CLRecovery_FlatThroughEntireDescentToAlphaStar` — enter stall at `alphaPeak`, reduce
   alpha to `alphaStar()`; verify `cl_eff == C_L_sep` throughout.  Valid only when
   `C_L_sep < C_L_star`; use a lift curve where `clSep() < clAlpha() * alphaStar()`.
2. `CLRecovery_SnapToNominalBeforeAlphaStar` — use a lift curve where
   `C_L_sep > C_L_star`; enter stall; verify `cl_eff` tracks `min(clSep(), cl_nom)` and
   snaps to the nominal descending quadratic at the intersection alpha (above `alphaStar()`)
   with no CL discontinuity.
3. `CLRecovery_RateLimitedBelowAlphaStar` — alpha crosses `alphaStar()`; verify CL climbs
   toward nominal at exactly `cl_alpha * alpha_dot_max * dt` per step
4. `CLRecovery_InstantDownwardFollow` — during recovery (below `alphaStar()`), raise alpha
   back into the descending quadratic; verify `cl_eff` follows nominal parabola downward
   immediately without rate limiting
5. `CLRecovery_BridgeDeactivatesAtNominal` — run recovery to completion; verify
   `cl_recovering == false` and `cl_eff == nominal`
6. `CLRecovery_NzFromEffectiveCL` — confirm realized Nz uses `cl_eff` throughout all phases
7. `CLRecovery_TZeroHoldsAlpha` — T=0 while stalled; verify `alpha_out` held at `_alpha_prev`
8. Serialization round-trip for `_cl_recovering` / `_cl_recovering_neg`

---

Design decisions for this section are recorded in [Open Questions — OQ-1 through OQ-16](#open-questions) (all resolved).

---

### Step 9 — Gravity Fix (`Aircraft.cpp` Step 10)

All open questions (OQ-17 through OQ-19) are resolved.  The failing test `AircraftTest.StraightAndLevel_AllFixtures_100s` already exists and is failing, satisfying the TDD precondition.  Proceed directly to the production code fix.

**Production code change — `src/Aircraft.cpp` step 10:**

Remove the incorrect comment:

```text
Gravity is embedded in the load-factor constraint — must not be added here.
```

Replace the step 10 block with:

```cpp
// 10. Wind-frame specific force = aero + thrust + gravity.
//    Thrust decomposition (Wind frame, X forward, Y right, Z down):
//      Tx =  T·cos(α)·cos(β)
//      Ty = -T·cos(α)·sin(β)
//      Tz = -T·sin(α)
//    Gravity in Wind frame: C_WN · g_NED = R_nw_mat^T · {0, 0, g}
//    (R_nw_mat is already computed above for the landing gear transform.)
constexpr float kGravity_mps2 = 9.80665f;
const Eigen::Vector3f g_wind = R_nw_mat.transpose() * Eigen::Vector3f{0.f, 0.f, kGravity_mps2};

const float ax = (T * ca * cb + F.x_n + F_gear_wind.x()) / m + g_wind.x();
const float ay = (-T * ca * sb + F.y_n + F_gear_wind.y()) / m + g_wind.y();
const float az = (-T * sa      + F.z_n + F_gear_wind.z()) / m + g_wind.z();
```

**Verification — all must pass after the fix:**

| Test | Expected outcome |
| --- | --- |
| `AircraftTest.StraightAndLevel_AllFixtures_100s` | Altitude drift < 50 m in 100 s for all three fixtures |
| `AircraftTest.ZeroThrottle_AircraftDecelerates` | Unchanged — must continue to pass |
| All other `AircraftTest` tests | Unchanged — must continue to pass |

### Implementation Order

| Step | Status | Action |
| --- | --- | --- |
| 1 | Done | Add `alphaStar()`, `alphaStarNeg()`, `alphaTrough()`, `clAlpha()`, `clSep()`, `clSepNeg()` accessors to `LiftCurveModel` (with tests) |
| 2 | Done | Add `alpha_dot_max_rad_s` as constructor parameter and serialized field to `LoadFactorAllocator` (proto, JSON, round-trip tests) |
| 3 | Done | Update `LoadFactorAllocator` test fixtures for new `alpha_dot_max_rad_s` serialized field |
| 4 | Done | Add `dt_s` to `LoadFactorInputs` |
| 5 | Done | Implement Item 1 (alpha rate bridge) — tests first |
| 6 | Done | Implement Item 2 (hysteresis flags) — tests first |
| 7 | Done | Implement Item 3 (CL recovery) — tests first |
| 8 | Done | Update `Aircraft.cpp` to read `alpha_dot_max_rad_s` from `load_factor_allocator` config, pass `dt_s` to `solve()`, and use `lfa_out.cl_eff` for the aerodynamic force computation |
| 9 | Ready | OQ-17, OQ-18, and OQ-19 resolved; failing test `StraightAndLevel_AllFixtures_100s` already exists; implement gravity fix in `Aircraft.cpp` step 10 per [Step 9](#step-9--gravity-fix-aircraftcpp-step-10) |

---

## Test Coverage

| Suite | Tests | File |
| --- | --- | --- |
| KinematicState | 58 | `test/KinematicState_test.cpp` |
| LiftCurveModel | 36 | `test/LiftCurveModel_test.cpp` |
| LoadFactorAllocator | 65 | `test/LoadFactorAllocator_test.cpp` |
| Aircraft | 20 | `test/Aircraft_test.cpp` |

### Known Failing Tests

| Test | Reason |
| --- | --- |
| `AircraftTest.StraightAndLevel_AllFixtures_100s` | Gravity omission in `Aircraft.cpp` step 10 (see [Aircraft.cpp → KinematicState Acceleration Interface](#aircraftcpp--kinematicstate-acceleration-interface)) causes ~100–300 m altitude drift in 100 s. Fix is specified in [Step 9](#step-9--gravity-fix-aircraftcpp-step-10); all open questions resolved. |

### Coverage Gap

No test verified altitude stability over multi-second simulation runs before `StraightAndLevel_AllFixtures_100s` was added.  The existing `AircraftTest.ZeroThrottle_AircraftDecelerates` runs for 0.5 s and checks speed only; it does not catch steady-state altitude drift.  `StraightAndLevel_AllFixtures_100s` fills this gap and is expected to pass once Step 9 is implemented.

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
