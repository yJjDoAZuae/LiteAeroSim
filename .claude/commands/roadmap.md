---
description: Audit and maintain roadmap documents in docs/roadmap/. Usage: /roadmap check <file>, /roadmap update <file>, /roadmap delivered <file> <item-id>
---

# Roadmap Maintenance — `/roadmap`

Audit and maintain roadmap documents in `docs/roadmap/`.

## Usage

```
/roadmap check <roadmap-file>
/roadmap update <roadmap-file>
/roadmap delivered <roadmap-file> <item-id>
```

- **`check`** — Audit one roadmap file for consistency against the current codebase and
  implementation plans. Report discrepancies without modifying anything.
- **`update`** — Re-read the codebase and implementation plans; update blocking dependency
  status, mark newly unblocked items, add newly discovered constraints, and update the
  Current State table.
- **`delivered`** — Mark a specific roadmap item as delivered: move it from the pending
  section to the Delivered table and add the delivery description. Does not begin
  implementation — only records that it is already complete.

---

## What a roadmap is

A roadmap describes **what to build and why**, at the level of named capabilities and
subsystems. It is the source of planning authority: it determines what has been built,
what comes next, and what is blocked and why.

A roadmap answers the question: *given the project's goals and constraints, what
capabilities have been delivered, what is pending, and in what order should pending work
proceed?*

A roadmap is **not** an implementation plan. It does not track individual work items,
assign IP-N identifiers, or order code-level tasks. That is the job of the `/impl` skill.

---

## Distinction from implementation plans (`/impl`)

| Dimension | Roadmap (`/roadmap`) | Implementation plan (`/impl`) |
| --- | --- | --- |
| Location | `docs/roadmap/` | `docs/implementation/` |
| Granularity | Named capabilities and subsystems | Individual code-level work items |
| ID scheme | Human-readable item labels (e.g. `LG-1`, `SB-3`, `Item 7`) | `IP-PLAN-N` (e.g. `IP-LGD-3`) |
| Status granularity | Delivered / Pending / Blocked | `todo` / `active` / `done` / `blocked (IDs)` |
| Dependency tracking | Named item-level blocking dependencies | Explicit `IP-*` and `OQ-*` IDs |
| Design refs | Refers to design authority documents | Refers to specific sections of design authority documents |
| Created when | A new capability or subsystem is planned | A roadmap item's design is settled and implementation is authorized |
| Maintained by | `/roadmap` skill | `/impl` skill |

**One roadmap item typically spawns one implementation plan.** When a roadmap item is
authorized for implementation (all open questions resolved, design document complete), the
user asks for an implementation plan via `/impl new`. The implementation plan is the
fine-grained execution artifact; the roadmap item is the planning artifact that authorized
it.

**Roadmaps outlive implementation plans.** A delivered roadmap item remains in the
`Delivered` section permanently. A completed implementation plan is marked `Complete` in
the master index (`docs/implementation/PLANS.md`) and is no longer maintained.

---

## Roadmap document structure

Every roadmap document in `docs/roadmap/` follows this structure:

### 1. Preamble

One-paragraph description of the subsystem or workstream. Scope statement: what is in
and what is out. Cross-references to related roadmaps (e.g. LiteAero Flight).

### 2. Current State table

Per-class or per-subsystem status table. Each row names one C++ class or major component,
its header file, and its current status using one of these values:

| Status value | Meaning |
| --- | --- |
| ✅ Implemented + serialization (JSON + proto) | Fully implemented; JSON and proto round-trips in place and tested |
| ✅ Implemented | Fully implemented; stateless (no serialization required) |
| 🔲 Planned | Design document exists or is in progress; not yet implemented |
| 🔲 Stub only | Empty header file exists; no design document |

The Current State table is updated whenever a class is implemented. It must reflect the
current codebase, not a projected future state.

### 3. Delivered section

A table listing every delivered roadmap item, in the order they were completed. Each row
contains the item identifier, a one-line description, and the test evidence. Delivered
items are never removed.

For roadmap items with substantial implementation notes, a subsection below the table
provides the full delivery description: scope, design decisions, tests, and known issues.

### 4. Pending items

One subsection per pending item. Each subsection contains:

- **Blocking dependencies** — named items (delivered or pending) that must be complete
  before this item can begin. An item with no blocking dependencies may begin at any time.
- **Design authority** — the document(s) that specify what to build.
- **Scope** — what the item produces. Written in declarative style (what it will be, not
  what to do). Mirrors the scope in the design authority document but need not repeat all
  detail.
- **Deliverables** — a bulleted or tabular list of concrete outputs.
- **Tests** — the test files and test count expected on completion. Must be consistent
  with the design authority's test strategy.

### 5. Open questions (if present)

If a pending item has an open question that prevents its design from being written, a
brief note pointing to the relevant `/oq` entries in the design document is permitted. Do
not duplicate open question content here — refer to the design document.

---

## Consistency checks

When running `/roadmap check`, verify all of the following. Report every failure.

| Check | Failure condition |
| --- | --- |
| Current State table accurate | A class listed as ✅ Implemented cannot be found implemented in `include/` + `src/`; or a class listed as 🔲 Stub only has a non-empty header file. |
| Delivered items match implementation | A roadmap item listed as delivered has no corresponding implementation plan in `done` state or no verifiable evidence in source files. |
| Blocking dep IDs exist | A blocking dependency named in a pending item does not correspond to any delivered or pending item in this roadmap or a registered cross-roadmap item. |
| Design authority exists | A pending item references a design authority document that does not exist on disk. |
| No pending items without design authority | Every pending item with "Blocking dependencies: None" must reference an existing design authority document before implementation can be authorized. |
| Implementation plans registered | Every delivered roadmap item that required implementation has a corresponding plan in `docs/implementation/PLANS.md` (status `Complete` or `Active`). |
| OQ references current | Any reference to an OQ in a blocking dependency statement references an open (unresolved) OQ. If the OQ is resolved, the blocking statement must be updated. |

---

## Marking an item delivered

When the user reports that a roadmap item is complete and instructs `/roadmap delivered`:

1. Read the source files and test files to verify the implementation is present.
2. Read the roadmap item's scope and deliverables list; verify each deliverable is present.
3. Move the item from the pending section to the Delivered table: add a row with the item
   ID, a one-line summary, and the test evidence (test file names and count).
4. If the item had a detailed subsection in the pending section, replace it with a compact
   delivery description subsection in the Delivered section (scope summary, design
   decisions made, tests, known issues). Do not leave the full pending-item subsection in
   place.
5. Update the Current State table to reflect the newly implemented classes.
6. Update any pending items that named this item as a blocking dependency: remove it from
   their blocking dependency list. If that removal unblocks the pending item entirely,
   note that it is now unblocked.
7. Update the corresponding implementation plan (if one exists) to `Complete` in
   `docs/implementation/PLANS.md`.
8. **Do not begin implementation of the next item.** A delivery record is documentation
   work only.

---

## Legacy implementation records in `docs/roadmap/`

Some files in `docs/roadmap/` (e.g. `terrain-implementation-plan.md`,
`liteaero-flight-migration-plan.md`) predate the `/impl` skill and contain step-by-step
implementation notes rather than roadmap descriptions. These are **implementation records**
— historical evidence of completed work — not active planning documents. They should not be
updated. If a legacy record describes work that is no longer complete or needs rework,
create a new roadmap item and implementation plan for the rework rather than editing the
record.

---

## Style rules

- Use American English spellings throughout.
- Use relative Markdown hyperlinks for all document references.
- Roadmap item identifiers are stable: once assigned, they do not change. Do not renumber
  items when new items are added.
- Do not describe the future state of a delivered item as pending. If an extension or
  rework is needed, add a new pending item (e.g. `LG-2` extends `LG-1`).
- Pending items describe *what will be built*, not *how to build it*. Save the how for
  the implementation plan and design documents.
- The Delivered section is append-only. Items are never removed or retroactively
  edited to change their scope description, unless correcting a factual error.
