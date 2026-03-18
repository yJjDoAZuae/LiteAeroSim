# Filter Discretization

This document describes the discretization methods used for all discrete filters in
LiteAeroSim. The canonical method is the **Tustin (bilinear) transform with frequency
prewarping**.

---

## Tustin Transform with Frequency Prewarping

The Tustin substitution maps continuous $s$ to discrete $z$ as:

$$
s = \frac{\omega_c}{\tan\!\left(\dfrac{\omega_c \Delta t}{2}\right)} \cdot \frac{z - 1}{z + 1}
$$

where $\omega_c$ is the prewarping frequency in rad/s (typically the filter's cutoff or
design frequency). This ensures the discrete filter's frequency response exactly matches
the continuous prototype at $\omega_c$.

---

## First-Order Low-Pass

Continuous prototype:

$$
H(s) = \frac{\omega_n}{s + \omega_n}
$$

Discrete transfer function:

$$
H(z) = \frac{b_0 + b_1 z^{-1}}{1 + a_1 z^{-1}}
$$

With $K = \omega_c / \tan\!\left(\omega_c \Delta t / 2\right)$:

$$
b_0 = \frac{\omega_n}{K + \omega_n}, \qquad
b_1 = \frac{\omega_n}{K + \omega_n}, \qquad
a_1 = \frac{\omega_n - K}{K + \omega_n}
$$

---

## State-Space Discretization (Second-Order)

For a continuous state-space model $(A, B, C, D)$, the Tustin-discretized matrices are:

$$
\Phi = (w_0 I + A)(w_0 I - A)^{-1}, \qquad
\Gamma = 2(w_0 I - A)^{-1} B
$$

$$
H = w_0 C (w_0 I - A)^{-1}, \qquad
J = D + C(w_0 I - A)^{-1} B
$$

where $w_0 = \omega_c / \tan(\omega_c \Delta t / 2)$.

This formulation is used in `FilterSS2` and the Dryden turbulence filters in
`Turbulence`.

---

## Timestep Convention

The timestep `dt_s` is a **configuration parameter**, not a runtime argument to `step()`.
It is parsed in `onInitialize(config)`, stored as a private member, and used to precompute
discrete filter coefficients. `step(float u)` takes only the input signal.

$$
y_k = f(u_k), \quad \Delta t\ \text{fixed at initialize()}
$$

Passing `dt_s` at every step call would be redundant, error-prone, and inconsistent with
how discrete filter coefficients are precomputed. The `setDt()` pattern is not used.
