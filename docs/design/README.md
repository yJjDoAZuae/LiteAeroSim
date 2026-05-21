# Subsystem Design Documents

This folder is the canonical home for all subsystem design authority documents in
LiteAero Sim. A design authority document is the single source of truth for a
subsystem's interface, models, serialization format, test requirements, and open design
questions.

## Role

Design documents answer *how does this subsystem work?* They specify:

- What the subsystem does (use cases and actors)
- How it is structured (class hierarchy and interface)
- What algorithms and models it implements (citing `docs/algorithms/` for derivations)
- What data it serializes and in what format (citing `docs/schemas/` for complex schemas)
- What design choices remain open (open questions that block implementation)
- How it is tested (unit, integration, scenario, and serialization tests)

Design documents do not answer *what to build next* (see [`docs/roadmap/`](../roadmap/))
or *exactly which code changes are needed in what order* (see
[`docs/implementation/`](../implementation/)).

## Relation to other document types

| Other type | Relationship |
| --- | --- |
| [`docs/architecture/`](../architecture/) | Architecture documents describe the system at the layer above subsystems. The subsystem registry in `overview.md` links to design documents here. |
| [`docs/algorithms/`](../algorithms/) | Design document model sections cite algorithm documents for mathematical derivations — they reference, do not reproduce. |
| [`docs/schemas/`](../schemas/) | Design document serialization sections cite schema documents when field tables are complex enough to warrant a standalone specification. |
| [`docs/roadmap/`](../roadmap/) | Roadmap pending items list a design document as their `**Design authority:**`. The design document does not link back to the roadmap item. |
| [`docs/implementation/`](../implementation/) | Implementation plan work items cite specific sections of design documents in their `Design refs` column. Open questions in design documents (`OQ-*`) appear in `blocked (OQ-*)` status cells in implementation plans. |

## Maintenance

Design documents are created and audited with the [`/design`](../../.claude/commands/design.md) skill:

- `/design new <name>` — scaffold a new design document from the canonical template
- `/design check <file>` — audit for completeness and format consistency
- `/design update <file>` — update after implementation changes

Open questions within design documents are managed with the [`/oq`](../../.claude/commands/oq.md) skill.

## Migration note

Prior to 2026-05-21, subsystem design authority documents were stored in
`docs/architecture/`. Files in that folder that describe a specific subsystem's class
hierarchy, models, interface, and tests are design documents in this taxonomy, not
architecture documents. Their current location in `docs/architecture/` is a nonconformance
to be resolved in a dedicated migration task — they are fully valid design authority
documents and remain the authoritative source until moved.

The `/design check` skill warns when it finds a design document in `docs/architecture/`
rather than here.

## Current documents

*(No documents yet — migration from `docs/architecture/` is a pending task.)*
