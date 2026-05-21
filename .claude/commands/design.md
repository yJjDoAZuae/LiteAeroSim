---
description: Create, check, and update subsystem design authority documents in docs/design/. Usage: /design new <name>, /design check <file>, /design update <file>
---

# Subsystem Design Document — `/design`

Create, audit, and maintain subsystem design authority documents in `docs/design/`.

## Usage

```
/design new <subsystem-name>
/design check <file>
/design update <file>
```

- **`new`** — Scaffold a new design document from the canonical template in `docs/design/`.
- **`check`** — Audit a design document for completeness and format consistency without modifying it. Report every failure before making any changes.
- **`update`** — Re-read the relevant source files and update the document after implementation changes: refresh class diagrams, interface declarations, and proto messages to match the code; note newly discovered gaps; close resolved open questions per the `/oq` skill.

---

## What a subsystem design authority document is

A subsystem design authority document specifies *what* a subsystem must do, *how* it integrates with the rest of the system, and *what design choices remain open*. It is the single source of truth for a subsystem's interface, physical or algorithmic models, serialization format, and test requirements.

Implementation plans derive work items from this document. Roadmap items cite it as their design authority. Open questions in this document block implementation plan items. The document is never aspirational — it describes the decided design, not proposals. Undecided elements are open questions, not forward-looking prose.

---

## Distinction from other document types

| Dimension | Design (`/design`) | Architecture (`/arch`) | Algorithm (`/algo`) | Schema (`/schema`) |
| --- | --- | --- | --- | --- |
| Location | `docs/design/` | `docs/architecture/` | `docs/algorithms/` | `docs/schemas/` |
| Scope | One subsystem or small cluster of classes | System topology, layer boundaries, subsystem registry | One mathematical algorithm or method | One class's or config file's serialization format |
| Contains OQs | Yes — blocks implementation | Yes — system-level decisions | No | No |
| Is design authority for | Implementation plan work items; roadmap items | High-level roadmap items; cross-subsystem OQs | Not a direct authority — cited by design docs | Implementation plan serialization work items |
| References algorithm docs | Yes — cites math derivations | No | N/A | No |
| References schema docs | Yes — cites field specs for complex formats | No | No | N/A |

---

## Canonical document format

Every design document must contain all of the following sections in this order. If a section is genuinely not applicable, retain the heading and add a one-line explanation (e.g., "No computational estimate — this is a pure interface class with no arithmetic").

```markdown
# [Subsystem Name] — Design

[Preamble — 1–3 sentences: the design target use case, validity bounds, and which
 parent model owns this subsystem. Never a bulleted list.]

---

## Use Case Decomposition

[mermaid flowchart: actors on left, use cases on right, arrows showing which actor
 triggers which use case. Group actors in a subgraph labeled "Actors".]

| ID | Use Case | Primary Actor | Mechanism |
| --- | --- | --- | --- |
| UC-N | [name] | [class or role] | [method call or event] |

---

## Class Hierarchy

[mermaid classDiagram: the primary class and all directly owned classes, with
 multiplicity relationships and key members. Show public methods and private member
 types. Do not show implementation internals.]

---

## [Model / Algorithm / Data Flow] — §N

[One or more numbered sections describing the core behavior. Title varies by subsystem:
 "Physical Models" for physics classes, "Algorithms" for processing classes, "Data Flow"
 for infrastructure classes. Number subsections §N.sub-section so that implementation
 plans can reference specific sections (e.g., "§3b. Asymmetric Orifice Damping").
 Cite algorithm documents for mathematical derivations — reference them, do not reproduce.]

---

## Integration

[How this subsystem is owned and called by its parent class. Quote the call signature.
 Describe what data flows in and out. Include a mermaid flowchart for non-trivial
 integration contracts. State which call sites exist (e.g., "called before
 LoadFactorAllocator solve in Aircraft::step()").]

---

## Interface

[C++ class declaration — public members and key private types only. Include the actual
 namespace and header path in a comment above the code block.]

---

## Serialization

### Serialized State

[Table: fields that change between reset() and a mid-flight step().]

| Field | Type | Unit | Description |
| --- | --- | --- | --- |

[Configuration parameters loaded from JSON on initialize() are NOT serialized state.
 If a detailed schema document exists in docs/schemas/, cross-reference it here.]

### Proto Message

[proto3 message definition.]

---

## Computational Resource Estimate

[Table of operation counts per outer step (by substep count and total). Memory footprint
 table. Timing estimate at the assumed outer step rate.]

---

## Open Questions

| ID | Summary | Blocking |
| --- | --- | --- |

[Full OQ subsections using the `/oq` skill format. Resolved OQs are replaced with a
 resolution note, not deleted. The summary table lists only open questions; resolved ones
 appear only in their subsection with a strikethrough title line.]

---

## Test Strategy

[Four subsections: Unit Tests, Integration Tests, Scenario Tests, Serialization Tests.
 Each subsection has a table: Test name | Input | Pass criterion.
 Test names follow ClassName_Condition_ExpectedOutcome (C++) or test_class_condition (Python).]

---

## References

| Reference | Relevance |
| --- | --- |
```

---

## Inter-referencing rules

### Outbound references from this document

- **Algorithm documents** — cite for mathematical derivations used in model sections:
  `[filters.md §Tustin](../algorithms/filters.md#tustin-transform)`. Do not reproduce.
- **Schema documents** — cite in the Serialization section when a standalone schema exists:
  `[landing_gear_state.md](../schemas/landing_gear_state.md)`.
- **Architecture documents** — cite for system context:
  `[overview.md §Layer Model](../architecture/overview.md#layer-model)`.
- **Roadmap** — cite the roadmap item that authorized this subsystem:
  `[aircraft.md §Item 7](../roadmap/aircraft.md)`.

### Inbound references to this document

- **Implementation plan work items** cite specific sections in their `Design refs` column:
  `[landing_gear.md §4a](../design/landing_gear.md#4a-integration-method-and-stability)`.
- **Roadmap pending items** list this document under their `**Design authority:**` field.
- **Implementation plan blocked items** reference OQs defined here: `blocked (OQ-LG-3)`.
- **Architecture overview** subsystem registry links to this document for every subsystem.

The design document itself does not contain a list of which plans or roadmap items reference
it. Those cross-references live in `docs/implementation/PLANS.md` and the roadmap documents.

---

## Creating a new design document (`/design new`)

1. Read `docs/design/` to assign a canonical file name (`snake_case_subsystem.md`).
2. Read two or three existing design documents as format references.
3. Create the file at `docs/design/<name>.md` with all canonical sections as labeled placeholders.
4. Add an entry to the subsystem registry table in `docs/architecture/overview.md`.
5. Do not add a roadmap item or implementation plan — those are separate tasks requiring explicit instruction.

---

## Updating an existing design document (`/design update`)

1. Read the current document and identify all sections that reference source files.
2. Read the referenced source files (headers, proto files) and compare against the document.
3. Update class diagrams, interface declarations, and proto messages where they have diverged.
4. If implementation is ahead of the design, add the covered design content.
5. If the design is ahead of the implementation, label the section **"Not yet implemented"**.
6. Do not close open questions without an explicit user instruction.

---

## Consistency checks (for `/design check`)

| Check | Failure condition |
| --- | --- |
| All required sections present | Any canonical section is absent without a "not applicable" note |
| No aspirational prose | A section describes something as "will be implemented" or "planned" without a "Not yet implemented" label |
| OQ table matches subsections | A row in the OQ summary table has no corresponding `### OQ-XX-N` subsection, or vice versa |
| OQ subsections follow `/oq` format | Each open OQ has: problem description, `**Alternatives:**` (each with **Benefits/Drawbacks/Prerequisites**), `**Recommendation:**` |
| Resolved OQs correctly retired | A resolved OQ still has an open-question subsection instead of a resolution note |
| Interface section matches source | Quoted C++ declarations match the current header on disk |
| Proto messages match source | Quoted proto definitions match the current `.proto` file on disk |
| All document references are hyperlinks | Any reference to a local file is a Markdown relative link, not a bare path |
| Document is in `docs/design/` | Warn if the file lives in `docs/architecture/` (migration needed per `docs/design/README.md`) |
| Roadmap cross-reference exists | The subsystem appears in a roadmap item with this file as its design authority |

---

## Style rules

- Preamble is 1–3 sentences. No bullet points, no headings.
- Use mermaid diagrams for class hierarchies and integration flows. No ASCII art.
- Model subsections use `§N.sub` anchors so implementation plans can cite them precisely.
- Quote only the public API in the Interface section — no implementation internals.
- SI units everywhere inside the domain layer. Never embed unit conversions inside model equations.
- Test names: `ClassName_Condition_ExpectedOutcome` (C++), `test_class_condition` (Python).
- American English throughout.
