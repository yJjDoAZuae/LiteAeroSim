# LiteAeroSim Documentation

## Contents

| Section | Description |
|---|---|
| [Architecture](architecture/overview.md) | System design, layer model, subsystem breakdown |
| [Dynamic Block Design](architecture/dynamic_block.md) | SISOBlock / DynamicBlock refactoring — the core simulation element pattern |
| [Algorithms](algorithms/filters.md) | Filter design, discretization, control algorithms |
| [Dependencies](dependencies/README.md) | External libraries — versions, licenses, integration method |
| [Installation](installation/README.md) | Build from source, toolchain setup, first run |
| [Testing](testing/strategy.md) | TDD strategy, test patterns, coverage requirements |
| [Examples](examples/siso_elements.md) | Usage examples for simulation elements |
| [Guidelines](guidelines/general.md) | Coding standards — TDD, SI units, naming, serialization |

## Quick Links

- [Coding guidelines — general](guidelines/general.md)
- [Coding guidelines — C++](guidelines/cpp.md)
- [Coding guidelines — Python](guidelines/python.md)
- [Component lifecycle](architecture/overview.md#component-lifecycle)
- [DynamicBlock proposed interface](architecture/dynamic_block.md#proposed-interface)
- [Migration strategy](architecture/dynamic_block.md#migration-strategy)
