---
description: Create and check algorithm documents in docs/algorithms/. Usage: /algo new <name>, /algo check <file>
---

# Algorithm Document — `/algo`

Create and audit mathematical algorithm documents in `docs/algorithms/`.

## Usage

```
/algo new <name>
/algo check <file>
```

- **`new`** — Scaffold a new algorithm document from the canonical template.
- **`check`** — Audit an algorithm document for completeness and format consistency without modifying it.

---

## What an algorithm document is

An algorithm document records a mathematical method — a filter, discretization technique,
numerical integration scheme, estimation algorithm, or aerodynamic model — at the level of
equations and derivations. It provides the mathematical reference that one or more subsystem
design documents cite. It does not describe a specific class or subsystem.

Algorithm documents are purely mathematical. They contain no code, no C++ class declarations,
no serialization format details, and no open questions. Undecided algorithmic choices belong
in the design document that uses the algorithm, not here.

---

## Distinction from other document types

| Dimension | Algorithm (`/algo`) | Design (`/design`) | Schema (`/schema`) |
| --- | --- | --- | --- |
| Location | `docs/algorithms/` | `docs/design/` | `docs/schemas/` |
| Scope | One mathematical method or family of related methods | One subsystem (class cluster) | One serialization format |
| Contains code | No | Yes (interface, proto) | Partial (proto message, JSON example) |
| Contains OQs | No | Yes | No |
| Is design authority for | Not a direct authority — cited by design docs | Implementation plans, roadmap items | Implementation serialization items |
| References | External literature (textbooks, papers, standards) | Algorithm docs, schema docs, architecture | Owning design doc |
| Changes when | The mathematical method is revised or extended | The subsystem design changes | The format changes |

---

## Canonical document format

```markdown
# [Algorithm Name]

[Preamble — 1–3 sentences: what mathematical problem this solves, what discrete-time
 implementation challenge it addresses, and which design documents cite it. Write for
 a reader who has general engineering knowledge but has not read the project design docs.]

---

## [Method or Topic Sections]

[One section per major method or filter type. A document covering a family of related
 filters (e.g., first-order and second-order low-pass) has one section per member.
 A document covering a single algorithm has one or more analysis sections.
 Within each section, follow the structure below.]

### [Section title — e.g., "First-Order Low-Pass" or "Tustin Transform"]

[One-sentence description of what this section covers.]

#### Continuous Formulation

[Transfer function H(s), differential equation, or continuous state-space (A, B, C, D).
 Define every symbol. Use SI units in all examples. Write every equation in a fenced
 display math block:]

$$
H(s) = \frac{\omega_n}{s + \omega_n}
$$

[Use inline math `$x$` for symbols appearing in prose. Use display math `$$...$$` for
 every stand-alone equation. Never embed equations as plain text or ASCII.]

#### Discrete Form

[Transfer function H(z), difference equation, or discrete state-space (Φ, Γ, H, J).
 Derive from the continuous form using the chosen discretization method. Show every
 algebraic step in display math — do not state the result without derivation.]

#### Coefficients

[Closed-form expressions for all coefficients as functions of the design parameters
 (e.g., cutoff frequency $\omega_c$, damping ratio $\zeta$, timestep $\Delta t$).
 State the prewarping frequency when using Tustin. Use display math for each expression.]

#### Numerical Properties

[Stability criterion (e.g., $\Delta t < 2/\omega_n$ for explicit Euler). Accuracy order.
 Frequency response behavior. Known edge cases: behavior at DC, at Nyquist,
 as $\omega_c \Delta t \to \pi$. Quantify where possible.]

---

## References

| Reference | Relevance |
| --- | --- |
```

---

## Inter-referencing rules

- Algorithm documents are cited **by** design documents, not the reverse. An algorithm
  document does not reference the design documents that use it.
- Algorithm documents cite external literature in their References section (textbooks,
  papers, standards). They do not reference roadmap or implementation plan documents.
- When a design document cites an algorithm document, it links to the specific section:
  `[filters.md §Tustin](../algorithms/filters.md#tustin-transform)`.
- When a design document reproduces a formula from an algorithm document, it cites the
  source inline — it does not reproduce the full derivation.

---

## Consistency checks (for `/algo check`)

| Check | Failure condition |
| --- | --- |
| Continuous and discrete forms both present | A method has a discrete form but no continuous prototype, or vice versa |
| Coefficient derivations shown | A coefficient is stated without algebraic derivation from the continuous form |
| No code snippets | Any C++ or Python code appears in the document |
| No open questions | A section is marked as an open question (these belong in the using design document) |
| All symbols defined | A symbol appears in an equation without a preceding definition |
| Numerical properties stated | A method has no stability criterion or accuracy order stated |
| All document references are hyperlinks | Any reference to a local file uses a Markdown relative link |
| References are external only | The References table contains links to local project files instead of external literature |

---

## Style rules

- **Math markdown is mandatory.** Use fenced display math (`$$...$$`) for every stand-alone
  equation. Use inline math (`$...$`) for symbols in prose. Never write equations as plain
  text (e.g., `H(s) = wn/(s+wn)` is not acceptable).
- LaTeX conventions: scalars italic ($x$), vectors bold ($\mathbf{v}$), matrices bold
  upright ($\mathbf{A}$), operators roman ($\sin$, $\arctan$, $\exp$).
- Define every symbol the first time it appears in each section. Do not rely on notation
  from other sections or other documents.
- State the prewarping frequency $\omega_c$ explicitly whenever the Tustin transform is used.
- Numerical examples are welcome but not required. Keep them brief (one or two parameter sets).
- Do not include implementation notes (which class calls this, what the C++ looks like).
  That information belongs in the design document's model section.
- American English throughout.
