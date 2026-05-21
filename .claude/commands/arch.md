---
description: Audit system architecture documents in docs/architecture/. Usage: /arch check <file>, /arch check-all
---

# System Architecture Document — `/arch`

Audit system-level architecture documents in `docs/architecture/`.

## Usage

```text
/arch check <file>
/arch check-all
```

- **`check`** — Audit one architecture document for completeness and consistency with the
  codebase and with subsystem design documents. Report every failure without modifying.
- **`check-all`** — Audit all architecture documents: `overview.md` and both states of
  the system registry (`system/present/` and `system/future/`).

---

## What a system architecture document is

A system architecture document describes the project at the level above individual
subsystems: how subsystems are organized into layers, which subsystems exist, how they
communicate, and what the project's coordination conventions are. Architecture documents
answer "how is the system organized?" — not "how does this class work?".

The per-subsystem design authority documents — which contain class hierarchies, model
equations, OQs, serialization specs, and test strategies — live in `docs/design/`, not
here. The architecture document's subsystem registry links to those design documents.

---

## Distinction from other document types

| Dimension | Architecture (`/arch`) | Design (`/design`) | Algorithm (`/algo`) | Schema (`/schema`) |
| --- | --- | --- | --- | --- |
| Location | `docs/architecture/` | `docs/design/` | `docs/algorithms/` | `docs/schemas/` |
| Scope | Entire system or cross-subsystem layer | One subsystem (class cluster) | One mathematical method | One serialization format |
| Contains OQs | Yes — system-level decisions | Yes — subsystem-level | No | No |
| Subsystem registry | Yes — all subsystems listed with design doc links | No | No | No |
| Updated when | System structure changes, new subsystems added, layer model revised | Subsystem design changes | Mathematical method revised | Serialization format changes |

---

## Architecture documents in this project

### `docs/architecture/overview.md`

The primary system architecture document. Its canonical sections are:

1. **Preamble** — project purpose and design scope in one paragraph.
2. **Layer model** — the four-layer architecture (Interface, Application, Domain,
   Infrastructure) with responsibilities and class examples per layer. Draw as a mermaid
   `block-beta` or `flowchart TD` diagram with one block per layer.
3. **Subsystem map** — a mermaid `flowchart` or `graph` diagram of all subsystems and
   their dependencies. Every subsystem is a node; every ownership or call dependency is
   an edge.
4. **Subsystem registry** — a table listing every subsystem with its location, design
   authority document (link to `docs/design/<file>.md`), and implementation status.
5. **Coordinate frames** — the project's reference frame conventions (NED, body, ENU)
   with axis definitions and the notation used in all design documents.
6. **Data flow** — how simulation state flows between the major components in one step
   loop iteration. Draw as a mermaid `sequenceDiagram` or `flowchart LR` diagram.
7. **Component lifecycle** — the canonical `initialize → reset → step → serialize`
   contract that all `DynamicElement` and `SisoElement` subclasses follow. Draw as a
   mermaid `stateDiagram-v2` diagram.
8. **Open questions** — system-level OQs using the `/oq` skill format (e.g., namespace
   migration, cross-subsystem registry design, global configuration format).
9. **References** — design documents, algorithm documents, external standards.

### `docs/architecture/system/`

The state registry holds two frozen snapshots of the system architecture: one for the
current codebase baseline (`present/`) and one for the roadmap target (`future/`).

Each state contains exactly six documents:

| File | Contents |
| --- | --- |
| `requirements.md` | System-level requirements derived from use cases and design targets |
| `use_cases.md` | Actor–use-case decomposition for the full system |
| `element_registry.md` | All `DynamicElement`s and `SisoElement`s: name, type, owning class, inputs, outputs |
| `dataflow_registry.md` | All channel names, types, producers, and consumers |
| `diagrams.md` | System-level mermaid diagrams (block diagram, call tree, state machine) |
| `icds.md` | Interface control documents for cross-subsystem data exchange |

State registry documents do not contain OQs. Unresolved structural decisions belong in
`overview.md` as system-level OQs. The state registry is updated at project milestones
by reading the current codebase and roadmap and revising both states to match.

---

## Inter-referencing rules

- Architecture documents reference subsystem design documents in their subsystem registry:
  one row per subsystem, linking to `docs/design/<subsystem>.md`.
- Architecture OQs (in `overview.md`) follow the `/oq` format and may be cited as blocking
  dependencies by roadmap items.
- Architecture documents do not reference implementation plan documents (`IP-*` IDs).
- The roadmap references architecture documents; the relationship does not go the reverse.
- Architecture documents reference algorithm documents only if a cross-subsystem algorithm
  (e.g., a coordinate frame transformation convention) is documented algorithmically.

---

## Consistency checks (for `/arch check`)

### `overview.md`

| Check | Failure condition |
| --- | --- |
| All canonical sections present | Any of the nine canonical sections (Preamble through References) is absent |
| Subsystem registry is complete | A subsystem in `include/` or `src/` has no entry in the registry table |
| Design document links resolve | A registry row links to a design document path that does not exist in `docs/design/` |
| No stale `docs/architecture/` design doc links | Registry rows link to `docs/architecture/*.md` instead of `docs/design/*.md` (migration needed) |
| Layer model is current | A subsystem is placed in the wrong layer per the four-layer convention |
| Coordinate frame conventions stated | No coordinate frame section, or conventions conflict with the project naming in code |
| OQs follow `/oq` format | Any open OQ does not have the canonical five-part structure (problem, alternatives with Benefits/Drawbacks/Prerequisites, recommendation) |
| All document references are hyperlinks | Any reference to a local file is a bare path rather than a Markdown relative link |

### `system/{present,future}/`

| Check | Failure condition |
| --- | --- |
| All six state documents exist | A state directory is missing one or more of the six required files |
| Element registry matches codebase | A `DynamicElement` or `SisoElement` subclass exists in code with no entry in `element_registry.md` |
| Present state reflects current code | `present/` describes subsystems, interfaces, or channels that no longer exist in the codebase |
| Future state aligns with roadmap | `future/` describes capabilities absent from any active roadmap item, or omits capabilities in the active roadmap |
| Dataflow channels are consistent | A channel in `dataflow_registry.md` has no producer or no consumer |

---

## Style rules

- Subsystem registry entries link using the design document file name as the link text:
  `[landing_gear.md](../design/landing_gear.md)`, not `[LandingGear](...)`.
- Mermaid diagrams in overview.md use consistent node IDs: `PascalCase` for classes,
  `snake_case` for channels or data flows.
- Coordinate frame conventions are defined once in `overview.md` and cross-referenced
  from all design documents — never duplicated.
- State registry documents use consistent terminology: "present" means the current
  committed codebase; "future" means the target described by the active roadmap.
- American English throughout.
