# Defect Report — KinematicState Attitude Model: Trim Aero vs. 6DOF Confusion

**Status:** Open  
**Affects:** `KinematicState` (liteaero-sim), `equations_of_motion.md` (liteaero-sim)  
**Discovered during:** Ground contact dynamics investigation, 2026-05-09

---

## Background: Two Distinct Attitude Models

The project uses a **trim aerodynamic (trim aero) point-mass model**, not a 6DOF rigid-body model.
These two models share some translational kinematics (position and velocity integration via RK4 on
the WGS84 state) but differ fundamentally in how attitude is defined and propagated.

### 6DOF Attitude Model

In a 6DOF model, attitude is the PRIMARY integrated state. The body quaternion `q_nb` is evolved
via the quaternion ODE driven by body angular rates `[p, q, r]`:

$$
\dot{q}_{nb} = \tfrac{1}{2}\,q_{nb} \otimes [0,\,p,\,q,\,r]^T
$$

Body rates are either measured (sensor path) or commanded (inner-loop control path). Attitude is
initialized from known conditions and propagates forward in time independently of the force field.
Aerodynamic angles α and β are then DERIVED from the resulting body attitude and the velocity
vector.

### Trim Aero Attitude Model

In a trim aero model, the force balance is the PRIMARY constraint. The aerodynamic angles α and β
are INPUTS (solved by the load factor allocator each step) and are not independently integrated.
Attitude is therefore DERIVED FROM the velocity vector and the aerodynamic angles — it is not an
independent integrated state.

The attitude chain for the trim aero model is:

1. **Velocity vector direction → VW frame (algebraic).**  
   The Velocity-Wind frame (VW) is defined algebraically from the airmass-relative velocity:

   $$C_{VW,N} = C_y(-\gamma_a)\,C_z(\chi_a)$$

   where $\chi_a$ is the aerodynamic track angle and $\gamma_a$ is the aerodynamic flight path angle.
   This is a deterministic function of the current velocity vector; it contains no accumulated error.

2. **Roll about the velocity axis → Wind frame (accumulated).**  
   The Wind frame (W) is the VW frame rolled by the wind-axis bank angle $\mu$:

   $$C_{VW,W} = C_x(\mu) \quad\Longrightarrow\quad q_{nw} = q_{nVW} \otimes R_x(\mu)$$

   $\mu$ is the accumulated wind-axis bank angle, driven by the commanded wind-frame roll rate
   $p_W$. In the implementation, $\mu$ is not stored or integrated as a separate scalar; it is
   implicitly encoded in the full quaternion $q_{nw}$ and updated incrementally via
   `commitAttitude`. Extracting $\mu$ as a scalar would introduce a gimbal singularity at
   vertical flight path angles (see OQ-2).

3. **Body frame → derived from Wind frame and α, β (algebraic).**

   $$q_{nb} = q_{nw} \otimes q_{wb}, \qquad C_{BW} = C_y(\alpha)\,C_z(-\beta)$$

   Body rates `[p, q, r]` are derived as a function of path-curvature rates, roll rate, and
   α̇, β̇ — they are NOT integrated.

---

## Current Implementation

### Primary State

`KinematicStateSnapshot` (liteaero-flight, `include/liteaero/nav/KinematicStateSnapshot.hpp`,
line 33) correctly stores `q_nw` as the primary orientation quaternion.  `q_nb` is documented as a
derived quantity:

> "q_nb is not stored because it is exactly derivable from q_nw, alpha_rad, and beta_rad."

The snapshot stores `roll_rate_wind_rad_s` as the instantaneous wind-frame roll rate; it does NOT
store the accumulated wind-axis bank angle `μ` as a separate scalar.

### `stepQnw` — Velocity-Tracking Attitude Update

`KinematicState::stepQnw` (`src/KinematicState.cpp`, lines 235–257):

```cpp
Eigen::Quaternionf diff_rot_n;
diff_rot_n.setFromTwoVectors(velocity_prev_NED_mps, velocity_NED_mps);
q_nw = (diff_rot_n * q_nw * roll_delta).normalized();
```

**What it does:**

- Computes the minimal rotation in NED that maps `v_prev` to `v_new`. The rotation axis is
  `v_prev × v_new`, perpendicular to both velocity vectors and therefore never about the
  velocity (roll) axis. Roll is preserved automatically.
- Pre-multiplies `q_nw` by this rotation — tracking the velocity direction change exactly.
- Right-multiplies by `roll_delta = Rx(p_W · dt)` to accumulate commanded wind-axis roll.

**Invariant:** `q_nw.x = v.normalized()` is established by the constructor and propagated by
every `stepQnw` call, provided that `v_prev` is the same velocity that was current when the
previous `stepQnw` call established `q_nw.x`. This invariant holds as long as `stepQnw` is
called with the truly final velocity for each step — after all modifications are complete.

### `stepQnv` — Algebraic Velocity Frame (Not Called)

`KinematicState::stepQnv` (`src/KinematicState.cpp`, lines 259–281) constructs `q_nv`
algebraically from the velocity vector each step. This is the correct approach for the velocity-
direction component of a trim aero frame.

However, it is **never called** from within `KinematicState::step()`. The public accessor
`q_nv()` (`include/KinematicState.hpp`, line 82) returns `Eigen::Quaternionf::Identity()`
unconditionally:

```cpp
Eigen::Quaternionf q_nv() const { return Eigen::Quaternionf::Identity(); } // not implemented
```

`stepQnv` is dead code.

---

## Defects

### D-1 — `stepQnw` Is Called Before All Velocity Modifications Are Complete

**File:** `src/Aircraft.cpp`, lines 277–296; `src/KinematicState.cpp`, lines 165–231  
**Severity:** High — root cause of the ground contact dynamics corruption bug

`stepQnw` is correct: `setFromTwoVectors(v_prev, v_new)` maps `v_prev.hat → v_new.hat`, and
since `q_nw.x = v_prev.hat` holds at step entry (established by the constructor, propagated by
each prior `stepQnw` call), the postcondition `q_nw.x = v_new.hat` follows exactly. Roll is
preserved because the rotation axis `v_prev × v_new` is perpendicular to both velocity vectors.

**The defect is timing.** `Aircraft::step()` calls `_state.step()` — which runs `stepQnw`
internally — and then applies the terrain hard constraint:

```cpp
_state.step(time_sec, ...);   // line 277: RK4 → v_post_RK4; stepQnw → q_nw.x = v_post_RK4.hat

if (_has_body_collider && _terrain != nullptr) {
    const float pen = _body_collider.maxCornerPenetration_m(...);
    if (pen > 0.f)
        _state.applyTerrainHardConstraint(pen);   // line 296: v_final = v_constrained ≠ v_post_RK4
}
```

`stepQnw` runs with `v_new = v_post_RK4`. The constraint then changes velocity to
`v_constrained`. `q_nw.x` now points at `v_post_RK4.hat` while the stored velocity is
`v_constrained`. On the next step, `v_prev = v_constrained` — but `q_nw.x = v_post_RK4.hat`
— so the precondition `q_nw.x = v_prev.hat` no longer holds. `stepQnw` applies the delta
from `v_constrained` to the next velocity to a `q_nw.x` that was not at `v_constrained` to
begin with. The misalignment (≈ flight path angle at contact, typically 15–30°) persists and
accumulates across gear contact events.

**What the calling sequence must guarantee:** `v_prev` is captured at the start of the step,
aligned with `q_nw.x` by construction. `stepQnw(v_prev, v_final)` must be called only after
ALL velocity modifications for that step — RK4, gear spring forces, terrain hard constraint —
are complete. Then `v_final` captures every modification and `q_nw.x = v_final.hat` holds
exactly. It does not matter who or what changed velocity in between; the only requirement is
that `stepQnw` sees the true final velocity as `v_new`.

**Consequence of the bug:** After the first terrain contact, `q_nw.x` points in the
pre-constraint descent direction while the stored velocity is level. The misalignment is never
recovered because subsequent `stepQnw` calls apply correct increments from the wrong starting
point. After the second contact the gap widens. `g_wind = R_nw^T · {0,0,g}` accumulates error
with each contact, producing a phantom acceleration that grows with each gear bounce.

### D-2 — `stepQnv` Is Defined but Never Called; `q_nv()` Returns Identity

**File:** `include/KinematicState.hpp` line 82; `src/KinematicState.cpp` lines 259–281  
**Severity:** Low (dead code)

`stepQnv` constructs `q_nVW` algebraically from the velocity vector. It is not required for the
`q_nw` update path — `setFromTwoVectors` preserves roll without reference to `q_nVW`. However,
`q_nVW` is a useful derived quantity for display and monitoring (aerodynamic track, flight path
angle). `stepQnv` is currently unreachable via the public interface and its accessor always returns
identity, making any caller silently incorrect.

### D-3 — `equations_of_motion.md` Integration Scheme Summary Describes 6DOF Attitude, Not Trim Aero

**File:** `docs/algorithms/equations_of_motion.md`, lines 1112–1126 (Integration Scheme Summary)  
**Severity:** Medium — documentation defect that obscures the correct trim aero algorithm

The Integration Scheme Summary shows:

```
ATT["Propagate quaternion
     q_{nb,k+1} = q_{nb,k} + Δt/2 · q ⊗ ω
     then renormalize"]
```

This is the 6DOF quaternion ODE. The trim aero model does not propagate `q_nb` via a quaternion
ODE. `q_nb` is a derived quantity in this model; it is never directly integrated.

The same section's "Implementation Notes" (line 1129) states `_q_nb` is the primary attitude
quaternion. This is incorrect — the primary stored quaternion is `q_nw` (see `KinematicStateSnapshot`,
line 33).

### D-5 — `equations_of_motion.md` Does Not Clearly Separate Trim Aero and 6DOF Models

**File:** `docs/algorithms/equations_of_motion.md`  
**Severity:** Medium — caused D-4 and contributed to D-1 being introduced

The document contains correct derivations for both models interleaved without explicit labeling of
which applies to this simulator:

| Section | Model described | Applicable? |
| --- | --- | --- |
| Attitude Kinematics — Quaternion ODE | 6DOF (q_nb integrated) | No |
| Body Rates from Wind Frame Rates | Trim aero | Yes |
| Integration Scheme Summary | 6DOF (q_nb propagated) | No |
| Implementation Notes | Incorrectly claims q_nb primary | No |
| Velocity and Velocity-Wind Frames | Trim aero | Yes |
| Path Angle Rates | Trim aero | Yes |

A reader cannot tell from the document which integration scheme is actually implemented.

---

## Observable Symptom Linking to D-1

The ground contact bug observed in the Godot simulation is a direct consequence of D-1:

**With the bug (current code):**

1. Aircraft descends and makes first gear contact. `stepQnw` correctly tracks velocity direction
   changes throughout the gear spring interval — no error here.
2. The terrain hard constraint fires: `applyTerrainHardConstraint` zeroes the downward velocity
   component. `KinematicState::step()` has already returned; `stepQnw` has already run for this
   cycle using `v_post_RK4` as `v_new`. The constraint changes velocity to `v_constrained` but
   `q_nw.x` remains at `v_post_RK4.hat`. The step-start velocity for the next cycle will be
   `v_constrained`, but `q_nw.x ≠ v_constrained.hat`.
3. Next cycle: `v_prev = v_constrained`. `stepQnw(v_constrained, v_next)` applies the correct
   delta from `v_constrained` to `v_next`, but adds it to a `q_nw.x` that was already 15–30°
   off from `v_constrained`. The full gap is never closed. `q_nw.x` remains pointing in the
   pre-constraint descent direction indefinitely.
4. Aircraft bounces and becomes airborne. The misaligned `q_nw` causes `g_wind = R_nw^T · {0,0,g}`
   to have a component in the wrong direction. Each subsequent step accumulates this phantom
   acceleration.
5. Second gear contact: constraint fires again, adding another increment of misalignment to
   the already-incorrect `q_nw`. By the second or third contact, `g_wind` is significantly
   wrong, producing the observed upward buoyancy that grows with each gear bounce.

**With the fix (stepPV/commitAttitude):** `commitAttitude` is called after both the RK4
integration and the terrain constraint. It receives `v_prev` captured at the start of the
step and `snapshot_.velocity_ned_mps` after the constraint. `setFromTwoVectors(v_prev, v_final)`
maps across the full velocity change — gear spring, RK4, and constraint all included.
`q_nw.x = v_final.hat` holds exactly. No misalignment is ever introduced.

---

## Open Questions

**OQ-1 — Resolved: no separate initialization path required.**  
This question was wrongly framed around scalar μ. There is no μ state to initialize.

At simulation start, `q_nw` is initialized through the existing KinematicState Constructor 2
path: `Aircraft::initialize()` reads optional `heading_rad`, `pitch_rad`, and `roll_rad` fields
from `initial_state` (all defaulting to 0), constructs `q_nb`, and Constructor 2 derives `q_nw`
from `q_nb`, `alpha`, and `beta`. Roll attitude at sim start is therefore fully covered by the
existing `roll_rad` config parameter.

At snapshot restore, `q_nw` is stored directly in `KinematicStateSnapshot` and is immediately
available. `stepPV` captures `v_prev` from `snapshot_.velocity_ned_mps` at the start of the
first step after restore — the same velocity that was stored when the snapshot was saved. No
additional state fields are required.

**OQ-2 — Resolved: compute the velocity-direction rotation directly; do not route through q_nVW.**  
A scalar encoding of μ has the same gimbal singularity as Euler angles: the roll axis (the velocity
vector) becomes degenerate at γ = ±90°, and the scalar wraps at ±π, producing orientation
discontinuities on any trajectory that passes through vertical flight path angles. Scalar μ must NOT
be used.

An earlier formulation of this answer used a five-step q_nVW residual algorithm. That approach
routes the velocity-direction update through `q_nVW_new ⊗ q_nVW_prev⁻¹`. When the atan2 azimuth
in `q_nVW` flips 180° as the horizontal velocity component changes sign through vertical (γ = ±90°),
this difference is a 180° yaw rotation, not a tiny one — and it propagates directly into `q_nw`.
Making `stepQnv` stateful to suppress the azimuth flip is fragile and surprising; the underlying
structural problem is that `q_nw` continuity should not depend on `q_nVW` azimuth continuity at all.

**Decision:** Update `q_nw` directly from the velocity direction change, never through `q_nVW`.

The rotation that takes v_hat_prev to v_hat_new is computed as
`setFromTwoVectors(v_hat_prev, v_hat_new)`. Its rotation axis is always `v_hat_prev × v_hat_new`,
which is perpendicular to both velocity vectors and therefore **never about the velocity vector
itself**. It does not touch the roll component, preserving roll automatically. It makes no reference
to azimuth and is not singular at vertical — it is only singular when v_hat_prev ≈ −v_hat_new
(antiparallel velocities), which is a genuine near-180° reversal requiring a separate fallback.

The update algorithm at each step is:

1. Compute `R_vel = setFromTwoVectors(v_hat_prev, v_hat_new)`.
   - If not near-antiparallel (`v_hat_prev · v_hat_new > −threshold`): use directly.
   - If near-antiparallel: `R_vel` = 180° rotation about the Wind lateral axis in NED:
     `y_W_N = q_nw_prev ⊗ [0, 1, 0]`. This axis is perpendicular to v_hat_prev by construction
     (it is the Wind y-axis), so it correctly rotates v_hat_prev toward v_hat_new without
     introducing any roll.
2. Accumulate roll and apply: `q_nw = (R_vel ⊗ q_nw_prev ⊗ Rx(p_W · dt)).normalized()`.

`q_nw` is singular only when `v → 0` (zero airspeed), not at any particular flight path angle.
`q_nVW` still has an inherent azimuth discontinuity at γ = ±90°, but that is an intrinsic property
of the azimuth coordinate; it no longer affects `q_nw` because `q_nVW` is not part of the
step-to-step update chain.

**OQ-3 — Resolved: velocity direction update first, then roll increment about the post-update velocity vector.**  
In the OQ-2 algorithm, `R_vel` is applied first (step 2), bringing the Wind x-axis into alignment
with `v_new`. The right-multiplication `⊗ Rx(p_W · dt)` then rolls about `v_new` — the
instantaneous (post-update) velocity vector. This ordering is not equivalent to the reverse:
rolling first and then reorienting would apply the roll about `v_prev` before the frame is
aligned with `v_new`, placing the roll in the wrong frame. The ordering in step 2 of OQ-2 is
physically correct.

**OQ-4 — Resolved: do not add scalar μ to `KinematicStateSnapshot`.**  
The primary stored state remains `q_nw` (a full quaternion). `v_prev` is captured internally by
`stepPV` at the start of each step from `snapshot_.velocity_ned_mps` and held until `commitAttitude`
consumes it — it is transient within a single step and does not need to be persisted in the
snapshot. No new `KinematicStateSnapshot` fields are needed.

**OQ-5 — Resolved: both force sources correctly contribute to path curvature during ground contact.**  
Path curvature `omega_wn_n` is derived from the total NED acceleration `snapshot_.acceleration_ned_mps2`,
which in `Aircraft::step()` is the Wind-frame sum of aerodynamic forces, thrust, gear contact
forces, and gravity — all rotated to NED. Both gear forces and aerodynamic forces drive path
curvature, and the body rate computation in `KinematicState::step()` correctly reflects this. The
body rates are physically meaningful during ground contact.

The residual concern is that `snapshot_.acceleration_ned_mps2` is set from the Wind-frame
acceleration passed to `KinematicState::step()`, which depends on `q_nw`. Until D-1 is fixed,
a corrupted `q_nw` produces a corrupted `acceleration_ned_mps2`, which in turn produces corrupted
path curvature and body rates. This is a symptom of D-1, not a separate defect.

---

---

## Implementation Plan

All open questions are resolved. Implementation proceeds in TDD order: failing tests first,
production code second. Items are sequenced so each builds on verified predecessors.

### IP-1 — Tests for algebraic VW-frame construction (D-2 prerequisite)

**Target:** `test/KinematicState_test.cpp`

Write tests verifying that `stepQnv` produces the correct `q_nVW` from known velocity vectors:

- **Level flight, north (γ = 0, χ = 0):** `q_nVW` = identity.
- **Level flight, east (γ = 0, χ = 90°):** correct yaw-only rotation; no pitch component.
- **Climbing flight (γ = 45°, χ = 0):** correct pitch-only rotation; no yaw component.
- **Zero-velocity degeneracy:** returns a defined (identity or last-valid) quaternion without NaN.
- **Inherent azimuth discontinuity at vertical — expected behavior:** For two consecutive velocity
  vectors that straddle γ = 90° with opposite horizontal components (`v_prev = [+ε, 0, −V]`,
  `v_new = [−ε, 0, −V]`), the raw atan2 azimuth flips from χ = 0° to χ = 180°, and
  `q_nVW_new` will differ from `q_nVW_prev` by approximately 180° about the velocity axis.
  This is the inherent singularity of the azimuth coordinate at vertical; it is expected
  behavior for `stepQnv`, not a bug. **`stepQnv` is a pure function of the current velocity
  vector and does not suppress this discontinuity.** The discontinuity does not propagate to
  `q_nw` because `q_nVW` is not part of the step-to-step `q_nw` update chain (see OQ-2).
  Verify that `stepQnv` returns the geometrically correct result for each input velocity
  independently — both outputs should pass the DCM verification `C_{VW,N} = Cy(−γ)Cz(χ)`.
- **Continuity on approach from below vertical:** Sweep γ from 80° up through 89.9° with
  fixed χ = 45°. Verify that consecutive `q_nVW` outputs change smoothly — no jump before
  the singularity threshold. Above the threshold, the azimuth discontinuity is expected.
- **Continuity on approach from above vertical:** Same sweep descending through −89.9° and −90°.

These tests specify the contract for `stepQnv` as a pure algebraic function: correct in the
non-singular regime, and explicitly documented (not suppressed) at the vertical singularity.

### IP-2 — Activate `stepQnv`; fix `q_nv()` accessor (D-2)

**Targets:** `src/KinematicState.cpp`, `include/KinematicState.hpp`

Make `stepQnv` callable and return a real value from `q_nv()`. No behavior change to `stepQnw`
yet — this item just removes the dead-code status and verifies IP-1 tests pass.

Steps:

1. Call `stepQnv` from within `step()` to keep `q_nv_` up to date each tick.
2. Replace the stub `q_nv()` accessor body with `return q_nv_;` (or equivalent internal member).
3. IP-1 tests must pass. No other tests may regress.

### IP-3 — Tests for `applyTerrainHardConstraint` q_nw consistency and `stepQnw` continuity (D-1 prerequisite)

**Target:** `test/KinematicState_test.cpp`

The existing `applyTerrainHardConstraint` tests (lines 787–831) verify altitude shift, velocity
zeroing, and upward-velocity preservation, but contain no q_nw consistency check. Write the
following tests that will fail against the current implementation and pass only after IP-4.

#### Terrain constraint q_nw consistency tests

The fundamental postcondition: after every `commitAttitude` call, `q_nw.x =
velocity_NED_mps.normalized()`. The fix (stepPV/commitAttitude split) ensures `commitAttitude`
sees the truly final velocity — after all modifications — so the postcondition always holds.
These tests verify that the postcondition holds even when `applyTerrainHardConstraint` fires
between `stepPV` and `commitAttitude`.

- **q_nw aligned after constraint — zero bank:** Initialize with a descending aircraft
  (γ = 10°, V = 50 m/s northward, zero bank). Verify: `q_nw * [1,0,0] ≈ v_NED.normalized()`.
  Call `stepPV(...)`. Call `applyTerrainHardConstraint(pen)` — downward velocity is zeroed.
  Call `commitAttitude(rollRate=0, dt)`. Verify:
  - `q_nw * [1,0,0] ≈ velocity_NED_mps().normalized()` (invariant holds with v_final)
  - `q_nw.toRotationMatrix().col(1).z() ≈ 0` (zero bank preserved)

  With the bug (no split; `step()` calls `stepQnw` before constraint), the invariant does
  NOT hold: `q_nw.x` points at `v_post_RK4.hat`, not `v_constrained.hat`.

- **q_nw aligned after constraint — nonzero bank:** Same scenario with 30° wind-axis bank.
  After the full stepPV / constraint / commitAttitude sequence, verify velocity alignment is
  correct AND bank angle is preserved (the rotation axis is perpendicular to the velocity
  vector and does not affect roll).

- **Post-bounce q_nw alignment across multiple steps:** Initialize descending as above. Run
  a sequence of: stepPV / constraint / commitAttitude / stepPV / commitAttitude / ... for 20
  cycles. Verify `q_nw * [1,0,0] ≈ velocity_NED_mps().normalized()` at every cycle. With
  the bug, the misalignment introduced in the first constrained cycle persists across all
  subsequent cycles. With the fix, the invariant holds at every cycle.

- **No effect when constraint does not fire:** Call stepPV / commitAttitude without any
  constraint call. Verify `q_nw * [1,0,0] ≈ velocity_NED_mps().normalized()` — normal
  operation must be unaffected by the split.

#### `stepQnw` continuity tests (robustness, not D-1 fix)

- **q_nw continuity across vertical — no roll input:** Initialize `KinematicState` with a
  steep-climb velocity `v_prev = [+ε, 0, −V]` (γ ≈ 90° − ε/V, zero bank, zero roll rate).
  Apply a single `step()` that produces `v_new = [−ε, 0, −V]` (the horizontal component
  reverses sign, atan2 azimuth would flip 180°). Verify that the angular distance between
  `q_nw` before and after the step is bounded by the true angular change of the velocity
  direction (≈ 2ε/V), **not ≈ 180°**. With the direct velocity-rotation algorithm, `R_vel =
  setFromTwoVectors(v_hat_prev, v_hat_new)` produces a rotation of ≈ 2ε/V about the NED
  y-axis — small and correct. The atan2 azimuth flip in `q_nVW` is irrelevant because
  `q_nVW` is not used in the update.

- **q_nw continuity across vertical — with nonzero roll input:** Same setup but with a
  nonzero constant `p_W`. `q_nw` after the step must satisfy the same angular-distance bound
  plus the roll increment `|p_W · dt|`. Verify that roll accumulates at the commanded rate
  and does not stall, reverse, or jump as the velocity direction crosses vertical.

- **Antiparallel fallback — near-180° velocity reversal:** Initialize with a steep descent
  `v_prev ≈ [0, 0, +V]` (nearly straight down, γ ≈ −90°, zero bank). Apply a step that
  produces `v_new ≈ [0, 0, −V]` (nearly straight up). `setFromTwoVectors` is undefined for
  exactly antiparallel inputs; the fallback must engage. Verify that `q_nw` after the step
  has its x-axis aligned with `v_hat_new` (the Wind frame points upward), and that the bank
  angle is preserved (zero bank before → zero bank after). The fallback must produce a
  well-defined result without NaN or an arbitrary large roll rotation.

### IP-4 — Split `KinematicState::step()` to Defer the Attitude Update (D-1)

**Targets:** `src/KinematicState.cpp`, `include/KinematicState.hpp`, `src/Aircraft.cpp`

The fix restructures the step sequence so that `stepQnw` is called only after all velocity
modifications — including the terrain hard constraint — are complete.

#### Phase A: Split `KinematicState::step()` into two methods

Extract the position/velocity integration from the attitude update:

```text
KinematicState::stepPV(double time_sec,
                        Eigen::Vector3f acceleration_Wind_mps,
                        float alpha_rad, float beta_rad,
                        float alphaDot_rps, float betaDot_rps,
                        const Eigen::Vector3f& wind_NED_mps)
    → runs RK4 on position and velocity
    → stores alpha, beta, alpha_dot, beta_dot, wind
    → does NOT call stepQnw or update body rates
    → stores v_prev internally (snapshot before RK4) for use by commitAttitude()

KinematicState::commitAttitude(float rollRate_Wind_rps, float dt_s)
    → calls stepQnw(v_prev_stored, snapshot_.velocity_ned_mps, rollRate, dt, snapshot_.q_nw)
    → computes body rates from the now-final q_nw
    → clears the stored v_prev
```

`KinematicState::step()` is retained as a convenience wrapper that calls `stepPV` then
`commitAttitude` immediately, for callers that have no post-integration velocity modifications.

#### Phase B: Update `Aircraft::step()` call sequence

```text
const float dt_s = static_cast<float>(time_sec - _state.time_sec());

_state.stepPV(time_sec, accel_wind, alpha, beta, alphaDot, betaDot, wind_NED);

if (_has_body_collider && _terrain != nullptr) {
    const float pen = _body_collider.maxCornerPenetration_m(...);
    if (pen > 0.f)
        _state.applyTerrainHardConstraint(pen);   // modifies velocity only — correct
}

_state.commitAttitude(rollRate_Wind_rps, dt_s);
    // stepQnw(v_prev, v_final) runs here — v_final includes constraint modification
```

`applyTerrainHardConstraint` is unchanged: it modifies position and velocity only, with no
q_nw involvement. `commitAttitude` is the single, unambiguous point where `stepQnw` is called,
and it always sees the truly final velocity as `v_new`.

No change to `KinematicStateSnapshot` fields is required. IP-3 tests must pass.

### IP-5 — Reorganize and correct `equations_of_motion.md` (D-4, D-5)

**Target:** `docs/algorithms/equations_of_motion.md`

The document currently interleaves trim aero and 6DOF content without labeling either. The
required changes are a reorganization of the document's top-level structure combined with
targeted corrections to the attitude sections. The section inventory below identifies what
belongs where.

#### Section inventory

| Section (current) | Model | Action |
| --- | --- | --- |
| Overview | Common | Update the flowchart to include the `q_nw` attitude output; otherwise keep |
| Reference Frames | Common | Keep as-is |
| Velocity and Velocity-Wind Frames | **Trim aero** | Keep; move into trim aero part |
| Aerodynamic Angles | **Trim aero** | Keep; move into trim aero part |
| Attitude Kinematics — Quaternion ODE | **6DOF only** | Move into a clearly labeled 6DOF reference section; add a note that this model is NOT implemented |
| Velocity Kinematics — Wind Frame Input | **Trim aero** | Keep; move into trim aero part |
| Position Kinematics — WGS84 Integration | Common | Keep; move into trim aero part (RK4 applies to both but the implementation is shared) |
| Euler Angles (3-2-1 Convention) | Derived output | Keep; label as derived quantities for display, applicable to both models |
| Body Rate–Euler Rate Kinematics | 6DOF reference | Move into 6DOF reference section; the trim aero model derives body rates via the Wind-frame decomposition, not this Euler-rate inversion |
| Body Rates from Wind Frame Rates | **Trim aero** | Keep; move into trim aero part; this is the trim aero body rate equation |
| Plane of Motion Frame | **Trim aero** | Keep; move into trim aero part |
| Aerodynamic Angle Rates | **Trim aero** | Keep; move into trim aero part |
| Aerodynamic Force Vectors | **Trim aero** | Keep; move into trim aero part |
| Lift Curve Model | **Trim aero** | Keep; move into trim aero part |
| Thrust Decomposition and Load Factor Allocation | **Trim aero** | Keep; move into trim aero part |
| Derived Quantities | Common | Keep |
| Integration Scheme Summary | **Wrong** | Rewrite — see below |
| Implementation Notes | **Wrong** | Rewrite — see below |

#### Top-level document structure after reorganization

```text
# Equations of Motion

## Overview
## Reference Frames

## Part 1 — Trim Aero Model (Implemented)

  ### Trim Aero Overview
  ### Velocity and Velocity-Wind Frames
  ### Aerodynamic Angles
  ### Body Rates from Wind Frame Rates
  ### Plane of Motion Frame
  ### Aerodynamic Angle Rates
  ### Velocity Kinematics — Wind Frame Input
  ### Position Kinematics — WGS84 Integration
  ### Aerodynamic Force Vectors
  ### Lift Curve Model
  ### Thrust Decomposition and Load Factor Allocation
  ### Integration Scheme Summary — Trim Aero
  ### Implementation Notes

## Part 2 — 6DOF Reference (Not Implemented)

  ### 6DOF Overview
  ### Attitude Kinematics — Quaternion ODE
  ### Body Rate–Euler Rate Kinematics
  ### Euler Angles (3-2-1 Convention)

## Derived Quantities
```

A short preamble under "Trim Aero Overview" must state:

- This simulator implements the trim aero model. Part 1 describes this model.
- Part 2 is retained as reference material only. None of Part 2 is implemented.
- The primary attitude quaternion is `q_nw` (Wind-to-NED). `q_nb` is a derived quantity.

#### Corrections to Integration Scheme Summary

Replace the current mermaid flowchart node:

```text
ATT["Propagate quaternion
     q_{nb,k+1} = q_{nb,k} + Δt/2 · q ⊗ ω
     then renormalize"]
```

with the correct trim aero attitude update, which is an incremental velocity-direction tracking
step, not a quaternion ODE integration:

```text
ATT["Update q_nw (trim aero)
     R_vel = setFromTwoVectors(v_hat_prev, v_hat_new)
     q_nw  = (R_vel ⊗ q_nw_prev ⊗ Rx(p_W·Δt)).normalized()
     Invariant: q_nw.x = v_hat_new after each step"]
```

Also update the prose below the flowchart: attitude is NOT updated with forward Euler on a
quaternion ODE. `q_nw` is tracked incrementally: the minimal rotation from the previous velocity
direction to the final velocity direction is applied in NED, preserving roll automatically. The
invariant `q_nw.x = v_hat` holds after every `commitAttitude` call because `commitAttitude`
always receives the step-start `v_prev` (aligned with `q_nw.x` by construction) and the
fully-modified final velocity as inputs.

#### Corrections to Implementation Notes

Replace the stale member-variable list with one that reflects the current
`KinematicStateSnapshot` fields:

- Primary attitude quaternion: `q_nw` (Wind-to-NED, `Eigen::Quaternionf`) — stored in
  `KinematicStateSnapshot::q_nw`.
- `q_nb` is NOT stored; it is derived on demand from `q_nw`, `alpha_rad`, and `beta_rad`.
- Body rates `[p, q, r]` are derived each step from Wind frame rates, α, β, α̇, β̇ — they are
  not integrated.
- NED velocity: `snapshot_.velocity_ned_mps`
- NED acceleration: `snapshot_.acceleration_ned_mps2`
- Position: `snapshot_.position_wgs84` (WGS84 lat/lon/alt)
- Aerodynamic angles α, β are inputs to `step()` — computed by the aerodynamics subsystem.

### IP-6 — Cross-reference in `aircraft.md`

**Target:** `docs/architecture/aircraft.md`

Add a note in the Physics Integration Loop section referencing this defect document and the
corrected `stepQnw` algorithm, so future readers understand why the attitude update is not a
simple quaternion ODE.

---

## Files Requiring Changes

| File | Change required | Plan item |
| --- | --- | --- |
| `src/KinematicState.cpp` | Add `stepPV` and `commitAttitude` methods; keep `step()` as wrapper; activate `stepQnv` call | IP-2, IP-4 |
| `include/KinematicState.hpp` | Declare `stepPV`, `commitAttitude`; update `q_nv()` to return real value | IP-2, IP-4 |
| `src/Aircraft.cpp` | Replace `_state.step(...)` with `_state.stepPV(...)` / constraint / `_state.commitAttitude(...)` sequence | IP-4 |
| `test/KinematicState_test.cpp` | Add VW-frame tests; add stepPV/commitAttitude/constraint sequence tests and `stepQnw` continuity tests | IP-1, IP-3 |
| `docs/algorithms/equations_of_motion.md` | Label trim aero vs. 6DOF; correct Integration Scheme Summary and Implementation Notes | IP-5 |
| `docs/architecture/aircraft.md` | Cross-reference this defect in the Physics Integration Loop section | IP-6 |
