---
description: Create and check serialization schema documents in docs/schemas/. Usage: /schema new <class-name>, /schema check <file>
---

# Schema Document — `/schema`

Create and audit serialization schema documents in `docs/schemas/`.

## Usage

```
/schema new <class-name>
/schema check <file>
```

- **`new`** — Scaffold a new schema document for a given class or configuration file.
- **`check`** — Audit a schema document for completeness and consistency with the
  corresponding source files and the owning class's design document.

---

## What a schema document is

A schema document provides the complete, authoritative field-by-field specification for
a single JSON configuration file or a class's runtime serialization format. It extends the
Serialization section of the owning subsystem design document: where the design document
names the fields and gives their physical meaning, the schema document provides field types,
units, valid ranges, cross-field constraints, and a validation example in one place.

A schema document covers exactly one logical schema — one JSON object or one proto message
cluster that maps to one class or one configuration section. It is not a general data
dictionary.

---

## Distinction from other document types

| Dimension | Schema (`/schema`) | Design (`/design`) | Algorithm (`/algo`) |
| --- | --- | --- | --- |
| Location | `docs/schemas/` | `docs/design/` | `docs/algorithms/` |
| Scope | One JSON schema or proto message | One subsystem (class cluster) | One mathematical method |
| Contains OQs | No | Yes | No |
| Is design authority for | Field names, types, units, constraints, ranges | Class interface, model equations, test strategy | Mathematical derivations |
| Updated when | A serialized field is added, removed, or constrained | The subsystem design changes | The mathematical method changes |
| Links to | Owning design document, validator script | Schema doc (if one exists), algorithm docs | External literature |

---

## When to create a schema document

Not every class requires a standalone schema document. Create one when:

- The JSON configuration has more than ten fields spread across multiple named sections, or
- Cross-field validation constraints are complex enough to warrant dedicated documentation, or
- A validator script (e.g., `validate_aircraft_config.py`) implements rules that should be
  documented alongside the schema.

For classes with fewer than ten simple fields, the Serialization section in the design
document is sufficient. No separate schema document is needed.

---

## Canonical document format

```markdown
# [ClassName / ConfigName] — Schema

**Owning class:** [`ClassName`]([relative path to design document])
**Schema version:** 1
**Formats:** JSON (initialization, read by `initialize()`), protobuf v3 (runtime state)
**Validator:** [`validate_script.py`]([relative path]) *(omit if no validator exists)*

[Preamble — 1–2 sentences: what this config initializes or what state this message
 captures. Cross-reference the design document's Serialization section.]

---

## JSON Format

### Top-Level Structure

```json
{
    "schema_version": 1,
    "section_name": { ... },
    ...
}
```

| Field | Type | Unit | Constraint | Description |
| --- | --- | --- | --- | --- |
| `schema_version` | integer | — | Must equal `1` | Schema version sentinel |

### [Section Name]

[One subsection per named JSON sub-object. Open each subsection with a one-line statement
 of which C++ class or initializer reads this section.]

| Field | Type | Unit | Constraint | Description |
| --- | --- | --- | --- | --- |
| `field_name` | type | unit or `—` | constraint | description |

---

## Proto Message

```proto
// Runtime serialized state — not configuration.
message ClassName {
    int32 schema_version = 1;
    // fields...
}
```

[Note: proto messages cover runtime serialized state produced by serializeProto() /
 consumed by deserializeProto(). Configuration is always JSON only. If this schema
 covers configuration only and has no proto message, replace this section with:
 "Configuration only — no proto message. Runtime state is not serialized for this class."]

---

## Constraints and Validation

[List cross-field constraints that cannot be expressed in the per-field Constraint column.
 Example: "All filter natural frequencies wn must satisfy wn * inner_dt < π, where
 inner_dt = outer_dt_s / cmd_filter_substeps. initialize() throws std::invalid_argument
 on violation."]

[State the exact exception type and message pattern thrown on validation failure.]

---

## Example

[Minimal valid JSON showing at least one entry in each required section.
 Do not reproduce every field — show enough to be illustrative.]

```json
{
    "schema_version": 1,
    ...
}
```

---

## References

| Reference | Relevance |
| --- | --- |
| [DesignDoc.md](../design/designdoc.md) | Owning class — field semantics and physical meaning |
```

---

## Inter-referencing rules

- A schema document always links to its owning design document in the header and in the
  References section. A design document's Serialization section always links back to the
  schema document when one exists.
- Schema documents do not contain open questions. Unresolved format questions belong in
  the owning design document's Open Questions section.
- Schema documents do not reference roadmap or implementation plan documents directly.
  Implementation plan work items that cover serialization cite the schema document in their
  `Design refs` column when a schema document exists.
- Schema documents do not reference algorithm documents.

---

## Consistency checks (for `/schema check`)

| Check | Failure condition |
| --- | --- |
| Owning design document linked | The header does not link to an existing design document |
| All JSON fields covered | A field present in the validator script or in `initialize()` source is absent from the field table |
| Types and units correct | A field type or unit in the table differs from what the source code reads or validates |
| Constraints match source | A constraint in the field table differs from the validation logic in `initialize()` or the validator |
| Proto message matches source | The quoted proto message differs from the current `.proto` file on disk |
| Example satisfies constraints | The example JSON violates any stated constraint |
| No OQ sections | The document contains an open question subsection |
| All document references are hyperlinks | Any reference to a local file uses a Markdown relative link |

---

## Style rules

- Field names use the JSON key exactly, in backtick code style: `` `field_name` ``.
- Units column uses SI abbreviations: `m`, `m/s`, `rad`, `N`, `kg`, `s`. Use `—` for
  dimensionless quantities.
- Constraints use mathematical notation: `> 0`, `∈ [0, 1]`, `Must equal 1`.
- The Example section is illustrative, not exhaustive — show one entry per section, not
  every possible field.
- The proto message block quotes only message definitions, not service or option declarations.
- American English throughout.
