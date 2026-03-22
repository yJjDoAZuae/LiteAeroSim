# FlightCode Repository Migration — Implementation Plan

## Context

This plan covers FC-1 (flightcode repository creation and repo split), the cross-cutting
milestone described in `docs/roadmap/README.md` and `docs/roadmap/flight_code.md`. It is
a three-phase operation:

- **Phase 0** (Step 0): Seed the `flightcode` repository from LiteAeroSim using
  `git filter-repo`, extracting only the commits that touch migrating files. The resulting
  repository has full LiteAeroSim development history for every file that migrates, with
  all unrelated history discarded.
- **Phase 1** (Steps 1–10): Build out the `flightcode` repository — add CMake scaffolding,
  reorganize paths into the `flightcode` directory layout, apply the `avraero::` namespace
  changes, add the shared interface target, and relocate FlightCode stubs. LiteAeroSim is
  not touched. Each step produces a buildable, tested addition to `flightcode`.
- **Phase 2** (Steps 11–13): Execute the LiteAeroSim repo split as a single coordinated
  event — update CMake to depend on `flightcode`, apply `avraero::simulation` namespace to
  all remaining LiteAeroSim code, and remove code that has migrated. The coordinated
  approach is required by the no-backward-compatibility policy and the single-step migration
  decision in `docs/architecture/system/future/decisions.md`.

Namespace decisions are final. All target namespaces are recorded in `decisions.md`.

---

## Key Design Decisions

| Decision | Rationale |
| --- | --- |
| `git filter-repo` for history extraction | Preserves full LiteAeroSim development history for every migrated file; a plain fork would also carry all terrain, aircraft, and environment history that has no place in a flight-code repository; `git filter-repo` discards that history precisely |
| Three-phase approach | Phase 0 seeds the repo from history; Phase 1 builds it out independently; Phase 2 is the single coordinated LiteAeroSim update |
| No forwarding aliases or backward-compat shims | Project is in initial development; no-backward-compat policy applies |
| `KinematicStateSnapshot` requires a design step | The current `KinematicState` is a rich class with computed properties and serialization; converting it to a plain value struct requires deciding which fields to store vs. derive, and how the control subsystem accesses derived quantities |
| Terrain type split requires a design step | `TerrainTile` currently bundles mesh data with serialization and file I/O; only the data elements migrate to `avraero::terrain`; the serialization machinery stays in `avraero::simulation`; a class-level split is needed |
| Shared interface target name is TBD | The CMake target and C++ namespace for `AircraftCommand`, `KinematicStateSnapshot`, `NavigationState`, and sensor measurement structs has not been decided; Step 6 resolves this |
| Tests migrate alongside the code | The flightcode repo includes its own test suite; relevant tests are moved from LiteAeroSim in Phase 1 and removed from LiteAeroSim in Phase 2 |
| `ControlLoop` and `Control*` elements stay in LiteAeroSim | `ControlAltitude`, `ControlHeading`, `ControlHeadingRate`, `ControlLoadFactor`, `ControlRoll`, `ControlVerticalSpeed`, and `ControlLoop` are simulation-internal; they will use `avraero::control` infrastructure after Phase 2 but remain in `avraero::simulation` |
| Estimation stubs (`NavigationFilter`, `WindEstimator`, `FlowAnglesEstimator`) are part of Phase 1 | Create them in `flightcode` at Step 9, not in LiteAeroSim; any LiteAeroSim stub files for these must be removed |

---

## Phase 0 — Seed the `flightcode` Repository

### Step 0 — Extract History with `git filter-repo`

Clone LiteAeroSim into a new `flightcode` directory and run `git filter-repo` to discard
every commit that does not touch at least one migrating file. The result is a repository
whose entire history is relevant to `flightcode`. The original LiteAeroSim repository is
not modified.

**Prerequisites:** `git filter-repo` must be installed (`pip install git-filter-repo`).

**Procedure:**

```bash
# Clone LiteAeroSim — this clone becomes flightcode
git clone /path/to/LiteAeroSim flightcode
cd flightcode

# Remove the upstream remote; the clone is now independent
git remote remove origin

# Run filter-repo using the paths file (see below)
git filter-repo --paths-from-file ../flightcode-migrate-paths.txt
```

**`flightcode-migrate-paths.txt`** — one path per line, relative to repo root. Directories
are matched as prefixes (all files below them are kept). Keep this file alongside the repos
for reference; it is not committed to either repository.

```text
include/ILogger.hpp
include/logger/
src/logger/
test/Logger_test.cpp
include/DynamicElement.hpp
include/SisoElement.hpp
src/DynamicElement.cpp
src/SisoElement.cpp
include/control/Antiwindup.hpp
include/control/ControlLoop.hpp
include/control/Derivative.hpp
include/control/Filter.hpp
include/control/FilterFIR.hpp
include/control/FilterSS.hpp
include/control/FilterSS2.hpp
include/control/FilterSS2Clip.hpp
include/control/FilterTF.hpp
include/control/FilterTF2.hpp
include/control/Gain.hpp
include/control/Integrator.hpp
include/control/Limit.hpp
include/control/LimitBase.hpp
include/control/LimitElement.hpp
include/control/RateLimit.hpp
include/control/RectilinearTable.hpp
include/control/SISOPIDFF.hpp
include/control/TableAxis.hpp
include/control/Unwrap.hpp
include/control/control.hpp
include/control/filter_realizations.hpp
src/control/Antiwindup.cpp
src/control/Derivative.cpp
src/control/FilterFIR.cpp
src/control/FilterSS.cpp
src/control/FilterSS2.cpp
src/control/FilterSS2Clip.cpp
src/control/FilterSS_copy_from_SS2.cpp
src/control/FilterTF.cpp
src/control/FilterTF2.cpp
src/control/Integrator.cpp
src/control/Limit.cpp
src/control/RateLimit.cpp
src/control/SISOPIDFF.cpp
src/control/Unwrap.cpp
src/control/control.cpp
src/control/filter_realizations.cpp
test/Antiwindup_test.cpp
test/Derivative_test.cpp
test/FilterFIR_test.cpp
test/FilterRealizations_test.cpp
test/FilterSS2Clip_test.cpp
test/FilterSS2_test.cpp
test/FilterSS_test.cpp
test/FilterTF2_test.cpp
test/FilterTF_test.cpp
test/Gain_test.cpp
test/Integrator_test.cpp
test/Limit_test.cpp
test/RateLimit_test.cpp
test/RectilinearTable_test.cpp
test/SISOPIDFF_test.cpp
test/TableAxis_test.cpp
test/Unwrap_test.cpp
include/environment/GeodeticAABB.hpp
include/environment/GeodeticPoint.hpp
include/environment/LocalAABB.hpp
include/environment/Terrain.hpp
include/environment/TerrainFacet.hpp
include/environment/TerrainTile.hpp
include/environment/TerrainVertex.hpp
test/Terrain_test.cpp
test/TerrainTile_test.cpp
include/KinematicState.hpp
src/KinematicState.cpp
test/KinematicState_test.cpp
include/navigation/WGS84.hpp
src/navigation/WGS84.cpp
test/WGS84_test.cpp
```

**Notes on path selection:**

- `include/control/Control*.hpp` and `src/control/Control*.cpp`
  (`ControlAltitude`, `ControlHeading`, etc.) are intentionally excluded — they stay in
  LiteAeroSim as `avraero::simulation` elements.
- `include/KinematicState.hpp` and `src/KinematicState.cpp` are included to preserve
  history for `KinematicStateSnapshot` (Step 7), even though the current class will be
  redesigned rather than moved directly.
- `include/navigation/WGS84.hpp` is included because `KinematicState` depends on it and
  it is a candidate for the shared interface target; include or exclude after resolving
  the `WGS84_Datum` dependency question in Step 6.
- Terrain source files (`src/environment/TerrainTile.cpp` etc.) are excluded until the
  Step 8 design resolves which implementation code, if any, migrates alongside the headers.

**After `git filter-repo` completes:**

The repository contains only the extracted files at their original LiteAeroSim paths. The
commit count will be significantly lower than LiteAeroSim's — only commits that touched at
least one kept path are retained. Verify with `git log --oneline | wc -l` and spot-check
that `git log include/DynamicElement.hpp` shows the expected history.

Add the new remote and push:

```bash
git remote add origin <flightcode-remote-url>
git push -u origin main
```

**Verification:** `git log --oneline` shows a meaningful history. No files exist outside
the paths listed above. The repository does not build yet (no CMakeLists.txt) — that is
expected and addressed in Step 1.

---

## Phase 1 — Build the `flightcode` Repository

### Step 1 — Repository Scaffolding and CMake Skeleton

The `flightcode` repository already exists (seeded in Step 0) with source files at their
original LiteAeroSim paths. This step adds the CMake build system and reorganizes the
extracted files into the `flightcode` directory layout using `git mv`. No namespace changes
yet — those are applied per subsystem in Steps 2–8.

**Directory structure:**

```text
flightcode/
  CMakeLists.txt          # root — fetches dependencies, includes subdirs
  cmake/
    Dependencies.cmake    # FetchContent for googletest, nlohmann/json, protobuf, Eigen
  log/
    include/avraero/log/
    src/
    test/
    CMakeLists.txt
  control/
    include/avraero/control/
    src/
    test/
    CMakeLists.txt
  terrain/
    include/avraero/terrain/
    src/
    test/
    CMakeLists.txt
  interfaces/             # name TBD — resolved in Step 6
    include/avraero/interfaces/
    src/
    test/
    CMakeLists.txt
  nav/
    include/avraero/nav/
    src/
    test/
    CMakeLists.txt
  guidance/
    include/avraero/guidance/
    src/
    test/
    CMakeLists.txt
  autopilot/
    include/avraero/autopilot/
    src/
    test/
    CMakeLists.txt
  proto/
    flightcode.proto
```

Each subdirectory `CMakeLists.txt` defines one `STATIC` library target with the matching
`avraero::X` alias. The root `CMakeLists.txt` includes all subdirectories and sets up
`FetchContent` for Eigen, nlohmann/json, protobuf, and googletest (same versions pinned
in LiteAeroSim). No target links anything yet — dependency edges are added per step.

**Verification:** `cmake -B build && cmake --build build` produces an empty build with no
errors.

---

### Step 2 — `avraero::log`: ILogger and Logging Infrastructure

Migrate the logging interface and sinks from LiteAeroSim to `flightcode/log/`.

**Source files to copy and adapt:**

| LiteAeroSim source | flightcode destination | Namespace change |
| --- | --- | --- |
| `include/ILogger.hpp` | `log/include/avraero/log/ILogger.hpp` | `liteaerosim` → `avraero::log` |
| `include/logger/LogSource.hpp` | `log/include/avraero/log/LogSource.hpp` | `liteaerosim::logger` → `avraero::log` |
| `include/logger/Logger.hpp` | `log/include/avraero/log/Logger.hpp` | `liteaerosim::logger` → `avraero::log` |
| `include/logger/LogReader.hpp` | `log/include/avraero/log/LogReader.hpp` | `liteaerosim::logger` → `avraero::log` |
| `src/logger/Logger.cpp` | `log/src/Logger.cpp` | `liteaerosim::logger` → `avraero::log` |
| `src/logger/LogReader.cpp` | `log/src/LogReader.cpp` | `liteaerosim::logger` → `avraero::log` |
| `src/logger/mcap_impl.cpp` | `log/src/mcap_impl.cpp` | (no namespace; MCAP implementation) |
| `src/logger/mcap_static.hpp` | `log/src/mcap_static.hpp` | (private header) |
| `test/Logger_test.cpp` | `log/test/Logger_test.cpp` | Update includes and namespace |

The `avraero::log` CMake target links `nlohmann_json` and the MCAP include path. The
`ILogger` interface has no implementation dependencies.

**Verification:** `cmake --build build && log/test/avraero_log_test` — all Logger tests
pass.

---

### Step 3 — `avraero::control`: `DynamicElement` and `SisoElement`

Migrate the root abstract lifecycle interfaces.

**Source files to copy and adapt:**

| LiteAeroSim source | flightcode destination | Namespace change |
| --- | --- | --- |
| `include/DynamicElement.hpp` | `control/include/avraero/control/DynamicElement.hpp` | `liteaerosim` → `avraero::control` |
| `src/DynamicElement.cpp` | `control/src/DynamicElement.cpp` | `liteaerosim` → `avraero::control` |
| `include/SisoElement.hpp` | `control/include/avraero/control/SisoElement.hpp` | `liteaerosim` → `avraero::control` |
| `src/SisoElement.cpp` | `control/src/SisoElement.cpp` | `liteaerosim` → `avraero::control` |

The `avraero::control` target links `avraero::log` (for `ILogger`), `nlohmann_json`, and
protobuf. No test files added in this step — `DynamicElement` and `SisoElement` are
abstract; tests arrive with their concrete subclasses.

**Verification:** `cmake --build build` — compiles cleanly, no errors.

---

### Step 4 — `avraero::control`: Filter Hierarchy

Migrate the full discrete-filter infrastructure.

**Source files to copy and adapt** (namespace `liteaerosim::control` → `avraero::control`
throughout):

| LiteAeroSim source | flightcode destination |
| --- | --- |
| `include/control/Filter.hpp` | `control/include/avraero/control/Filter.hpp` |
| `include/control/FilterSS.hpp` | `control/include/avraero/control/FilterSS.hpp` |
| `include/control/FilterSS2.hpp` | `control/include/avraero/control/FilterSS2.hpp` |
| `include/control/FilterSS2Clip.hpp` | `control/include/avraero/control/FilterSS2Clip.hpp` |
| `include/control/FilterTF.hpp` | `control/include/avraero/control/FilterTF.hpp` |
| `include/control/FilterTF2.hpp` | `control/include/avraero/control/FilterTF2.hpp` |
| `include/control/FilterFIR.hpp` | `control/include/avraero/control/FilterFIR.hpp` |
| `include/control/filter_realizations.hpp` | `control/include/avraero/control/filter_realizations.hpp` |
| `src/control/FilterSS.cpp` | `control/src/FilterSS.cpp` |
| `src/control/FilterSS2.cpp` | `control/src/FilterSS2.cpp` |
| `src/control/FilterSS2Clip.cpp` | `control/src/FilterSS2Clip.cpp` |
| `src/control/FilterTF.cpp` | `control/src/FilterTF.cpp` |
| `src/control/FilterTF2.cpp` | `control/src/FilterTF2.cpp` |
| `src/control/FilterFIR.cpp` | `control/src/FilterFIR.cpp` |
| `src/control/filter_realizations.cpp` | `control/src/filter_realizations.cpp` |
| `src/control/FilterSS_copy_from_SS2.cpp` | `control/src/FilterSS_copy_from_SS2.cpp` |
| `test/FilterSS_test.cpp` | `control/test/FilterSS_test.cpp` |
| `test/FilterSS2_test.cpp` | `control/test/FilterSS2_test.cpp` |
| `test/FilterSS2Clip_test.cpp` | `control/test/FilterSS2Clip_test.cpp` |
| `test/FilterTF_test.cpp` | `control/test/FilterTF_test.cpp` |
| `test/FilterTF2_test.cpp` | `control/test/FilterTF2_test.cpp` |
| `test/FilterFIR_test.cpp` | `control/test/FilterFIR_test.cpp` |
| `test/FilterRealizations_test.cpp` | `control/test/FilterRealizations_test.cpp` |

Preserve the GCC vtable fix: `FilterSS.hpp` must not include `FilterSS2.hpp`;
`FilterSS_copy_from_SS2.cpp` is the separate translation unit that includes both.

**Verification:** `cmake --build build && control/test/avraero_control_test` — all filter
tests pass; `FilterTFTest.FirstOrderLP00` and `FilterTFTest.SecondOrderLP00` are
pre-existing failures and should appear unchanged.

---

### Step 5 — `avraero::control`: SISO Elements and Scheduling Infrastructure

Migrate integrators, rate limiters, PID, antiwindup, gain scheduling, and related
infrastructure.

**Source files to copy and adapt:**

| LiteAeroSim source | flightcode destination |
| --- | --- |
| `include/control/Integrator.hpp` | `control/include/avraero/control/Integrator.hpp` |
| `include/control/Derivative.hpp` | `control/include/avraero/control/Derivative.hpp` |
| `include/control/Limit.hpp` | `control/include/avraero/control/Limit.hpp` |
| `include/control/LimitBase.hpp` | `control/include/avraero/control/LimitBase.hpp` |
| `include/control/LimitElement.hpp` | `control/include/avraero/control/LimitElement.hpp` |
| `include/control/RateLimit.hpp` | `control/include/avraero/control/RateLimit.hpp` |
| `include/control/Unwrap.hpp` | `control/include/avraero/control/Unwrap.hpp` |
| `include/control/SISOPIDFF.hpp` | `control/include/avraero/control/SISOPIDFF.hpp` |
| `include/control/Antiwindup.hpp` | `control/include/avraero/control/Antiwindup.hpp` |
| `include/control/Gain.hpp` | `control/include/avraero/control/Gain.hpp` |
| `include/control/RectilinearTable.hpp` | `control/include/avraero/control/RectilinearTable.hpp` |
| `include/control/TableAxis.hpp` | `control/include/avraero/control/TableAxis.hpp` |
| `include/control/control.hpp` | `control/include/avraero/control/control.hpp` |
| `src/control/Integrator.cpp` | `control/src/Integrator.cpp` |
| `src/control/Derivative.cpp` | `control/src/Derivative.cpp` |
| `src/control/Limit.cpp` | `control/src/Limit.cpp` |
| `src/control/RateLimit.cpp` | `control/src/RateLimit.cpp` |
| `src/control/Unwrap.cpp` | `control/src/Unwrap.cpp` |
| `src/control/SISOPIDFF.cpp` | `control/src/SISOPIDFF.cpp` |
| `src/control/Antiwindup.cpp` | `control/src/Antiwindup.cpp` |
| `src/control/control.cpp` | `control/src/control.cpp` |
| `test/Integrator_test.cpp` | `control/test/Integrator_test.cpp` |
| `test/Derivative_test.cpp` | `control/test/Derivative_test.cpp` |
| `test/Limit_test.cpp` | `control/test/Limit_test.cpp` |
| `test/RateLimit_test.cpp` | `control/test/RateLimit_test.cpp` |
| `test/Unwrap_test.cpp` | `control/test/Unwrap_test.cpp` |
| `test/SISOPIDFF_test.cpp` | `control/test/SISOPIDFF_test.cpp` |
| `test/Antiwindup_test.cpp` | `control/test/Antiwindup_test.cpp` |
| `test/Gain_test.cpp` | `control/test/Gain_test.cpp` |
| `test/RectilinearTable_test.cpp` | `control/test/RectilinearTable_test.cpp` |
| `test/TableAxis_test.cpp` | `control/test/TableAxis_test.cpp` |

**Verification:** `cmake --build build && control/test/avraero_control_test` — all SISO
element and PID tests pass.

---

### Step 6 — `KinematicStateSnapshot` Design Document

**Deliverable:** Design authority document at
`flightcode/docs/architecture/kinematic_state_snapshot.md`.

Do not implement in this step.

**Design questions to resolve:**

- **Struct fields:** Which quantities from `KinematicState` are stored vs. derived?
  `KinematicState` currently computes `POM`, `TurnCircle`, `q_ns`, `q_nv`, `q_nl`,
  `crab()`, `crabRate()`, and other derived quantities on demand. The snapshot must decide
  which of these are stored values (computed once per step by the aircraft model) and which
  are removed from the interface.
- **Shared interface target name:** Decide the CMake target name and C++ namespace for
  `AircraftCommand`, `KinematicStateSnapshot`, `NavigationState`, and sensor measurement
  structs. Candidates: `avraero::interfaces`, `avraero::icd`, `avraero::types`. Update
  `flightcode/CMakeLists.txt` and `interfaces/` directory name accordingly. Record the
  decision in `docs/architecture/system/future/decisions.md`.
- **Control subsystem access to derived quantities:** The `ControlAltitude`, `ControlRoll`,
  etc. classes currently accept `const KinematicState&` and call methods like `roll()`,
  `pitch()`, `heading()`. After migration to `avraero::control`, they will receive
  `KinematicStateSnapshot`. Any derived quantities they need must be present as stored
  fields or computable from the snapshot's plain fields.
- **`WGS84_Datum` dependency:** `KinematicState` includes a `WGS84_Datum` position. Decide
  whether `KinematicStateSnapshot` includes position as a `WGS84_Datum` or as a simpler
  ECEF/NED triple; the `WGS84_Datum` type and `WGS84.hpp` may need to migrate or be
  replicated.

---

### Step 7 — Shared Interface Target: `KinematicStateSnapshot`, `AircraftCommand`, `NavigationState`

Implement the shared interface target as designed in Step 6. Namespace and CMake target
name as decided in Step 6.

**Files to create** (using the namespace and target name decided in Step 6; `avraero::ifc`
used as a placeholder below):

| File | Contents |
| --- | --- |
| `interfaces/include/avraero/ifc/AircraftCommand.hpp` | Extract from `Aircraft.hpp`; plain value struct |
| `interfaces/include/avraero/ifc/KinematicStateSnapshot.hpp` | New plain value struct per Step 6 design |
| `interfaces/include/avraero/ifc/NavigationState.hpp` | New plain value struct; at minimum a stub with the fields used by `Autopilot`, `PathGuidance`, `VerticalGuidance`, `ParkTracking`, `WindEstimator`, `FlowAnglesEstimator` |
| `interfaces/include/avraero/ifc/AirDataMeasurement.hpp` | Sensor measurement struct for air data (airspeed, altitude, AOA, sideslip) |
| `interfaces/include/avraero/ifc/GnssMeasurement.hpp` | Sensor measurement struct stub for GNSS |
| `interfaces/include/avraero/ifc/MagMeasurement.hpp` | Sensor measurement struct stub for magnetometer |
| `interfaces/test/Interfaces_test.cpp` | Static-assert plain-struct properties; round-trip layout tests |

All types in this target are plain value structs with no virtual methods, no lifecycle, and
no serialization machinery (per the `decisions.md` module communication interface decision).

**Verification:** `cmake --build build && interfaces/test/avraero_ifc_test` — all static
assertion tests pass.

---

### Step 8 — `avraero::terrain`: Terrain Element Types and `V_Terrain`

Migrate the shared terrain mesh types and the `V_Terrain` query interface.

**Design note — `TerrainTile` split:** The current `TerrainTile` class bundles mesh data
(`lod_`, list of `TerrainCell*`) with serialization and file I/O. Only the data elements
listed in `decisions.md` migrate to `avraero::terrain`. If `TerrainTile` cannot be cleanly
separated, create a new `TerrainTileData` plain struct in `avraero::terrain` and keep the
full `TerrainTile` class (with serialization) in `avraero::simulation`. Update `TerrainMesh`
and `LodSelector` to use the split accordingly. Resolve this before starting file moves.

**Files to create in `flightcode/terrain/`:**

| LiteAeroSim source | flightcode destination | Namespace change |
| --- | --- | --- |
| `include/environment/TerrainVertex.hpp` | `terrain/include/avraero/terrain/TerrainVertex.hpp` | `liteaerosim::environment` → `avraero::terrain` |
| `include/environment/TerrainFacet.hpp` | `terrain/include/avraero/terrain/TerrainFacet.hpp` | `liteaerosim::environment` → `avraero::terrain` |
| `include/environment/TerrainTile.hpp` (or `TerrainTileData.hpp`) | `terrain/include/avraero/terrain/TerrainTileData.hpp` | See design note |
| `include/environment/GeodeticPoint.hpp` | `terrain/include/avraero/terrain/GeodeticPoint.hpp` | `liteaerosim::environment` → `avraero::terrain` |
| `include/environment/GeodeticAABB.hpp` | `terrain/include/avraero/terrain/GeodeticAABB.hpp` | `liteaerosim::environment` → `avraero::terrain` |
| `include/environment/LocalAABB.hpp` | `terrain/include/avraero/terrain/LocalAABB.hpp` | `liteaerosim::environment` → `avraero::terrain` |
| `include/environment/Terrain.hpp` | `terrain/include/avraero/terrain/V_Terrain.hpp` | `liteaerosim::environment` → `avraero::terrain`; rename file to `V_Terrain.hpp` to match the class name |

The `avraero::terrain` target has no source files (all headers are either pure abstractions
or plain structs) and no runtime dependencies. It links Eigen only.

**Tests:** Add a test file `terrain/test/Terrain_test.cpp` covering basic geometric
property checks for each migrated struct type; copy relevant assertions from
`test/Terrain_test.cpp`.

**Verification:** `cmake --build build && terrain/test/avraero_terrain_test` — all struct
tests pass.

---

### Step 9 — FlightCode Stub Headers

Create stub headers in `flightcode` for all FlightCode elements listed in `flight_code.md`
Current State table, under their target namespaces. Remove any corresponding stubs that
currently exist as untracked files in LiteAeroSim (`include/control/Autopilot.hpp`,
`include/guidance/`, `include/path/`).

**Stubs to create:**

| Target namespace | File | Notes |
| --- | --- | --- |
| `avraero::autopilot` | `autopilot/include/avraero/autopilot/Autopilot.hpp` | Replace `include/control/Autopilot.hpp` |
| `avraero::guidance` | `guidance/include/avraero/guidance/PathGuidance.hpp` | Replace `include/guidance/PathGuidance.hpp` |
| `avraero::guidance` | `guidance/include/avraero/guidance/VerticalGuidance.hpp` | Replace `include/guidance/VerticalGuidance.hpp` |
| `avraero::guidance` | `guidance/include/avraero/guidance/ParkTracking.hpp` | Replace `include/guidance/ParkTracking.hpp` |
| `avraero::guidance` | `guidance/include/avraero/guidance/V_PathSegment.hpp` | Replace `include/path/V_PathSegment.hpp` |
| `avraero::guidance` | `guidance/include/avraero/guidance/PathSegmentHelix.hpp` | Replace `include/path/PathSegmentHelix.hpp` |
| `avraero::guidance` | `guidance/include/avraero/guidance/Path.hpp` | Replace `include/path/Path.hpp` |
| `avraero::nav` | `nav/include/avraero/nav/NavigationFilter.hpp` | Create; design authority in `docs/architecture/navigation_filter.md` |
| `avraero::nav` | `nav/include/avraero/nav/WindEstimator.hpp` | Create; design authority in `docs/architecture/wind_estimator.md` |
| `avraero::nav` | `nav/include/avraero/nav/FlowAnglesEstimator.hpp` | Create; design authority in `docs/architecture/flow_angles_estimator.md` |

Each stub is a comment-only or minimal class declaration noting the design authority
document and the target namespace. No implementation.

**Update `flight_code.md` Current State table:** Replace the LiteAeroSim stub paths with
the new `flightcode` paths. Mark the LiteAeroSim entries as removed.

**Verification:** `cmake --build build` — flightcode builds cleanly with stubs in place.

---

### Step 10 — flightcode Build and Test Verification

Run the complete flightcode test suite. Confirm zero failures beyond the two known
pre-existing filter failures.

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Document the test count baseline (expected: all tests from Steps 2–8 passing). This
baseline is the regression reference for Phase 2.

---

## Phase 2 — LiteAeroSim Repo Split

### Step 11 — LiteAeroSim CMake: Dependency on `flightcode`

Update `LiteAeroSim/CMakeLists.txt` and `LiteAeroSim/cmake/Dependencies.cmake` to add
`flightcode` as a versioned `FetchContent` or `find_package` dependency. Add `avraero::log`,
`avraero::control`, `avraero::terrain`, and the shared interface target to the
`liteaerosim` library's `target_link_libraries`. Do not yet remove any LiteAeroSim source
files — this step establishes that both copies coexist and the build does not break before
the namespace changes are applied.

**Verification:** `cmake --build build` — LiteAeroSim builds with flightcode as a
dependency; all existing LiteAeroSim tests still pass.

---

### Step 12 — LiteAeroSim Namespace Migration and Code Removal

Execute the full namespace migration as a single coordinated change:

**Namespace changes applied across all LiteAeroSim source files:**

| Old namespace | New namespace | Scope |
| --- | --- | --- |
| `liteaerosim` (root) | `avraero::simulation` | `Aircraft`, `KinematicState`, `AeroCoeffEstimator`, all aerodynamics, airframe, propulsion, environment, sensor classes |
| `liteaerosim::control` | `avraero::simulation::control` | `ControlAltitude`, `ControlHeading`, `ControlHeadingRate`, `ControlLoadFactor`, `ControlRoll`, `ControlVerticalSpeed`, `ControlLoop` |
| `liteaerosim::environment` | `avraero::simulation` | All environment implementation classes that remain in LiteAeroSim after terrain type split |
| `liteaerosim::logger` | `avraero::log` | Logger and sinks now redirect to flightcode's `avraero::log` |

**Infrastructure headers replaced by flightcode equivalents** (includes updated at all call
sites):

| Remove from LiteAeroSim | Replace with |
| --- | --- |
| `include/DynamicElement.hpp` | `avraero/control/DynamicElement.hpp` from `avraero::control` |
| `include/SisoElement.hpp` | `avraero/control/SisoElement.hpp` from `avraero::control` |
| `include/ILogger.hpp` | `avraero/log/ILogger.hpp` from `avraero::log` |
| `include/logger/Logger.hpp` | `avraero/log/Logger.hpp` from `avraero::log` |
| `include/logger/LogReader.hpp` | `avraero/log/LogReader.hpp` from `avraero::log` |
| `include/logger/LogSource.hpp` | `avraero/log/LogSource.hpp` from `avraero::log` |
| `include/control/Filter.hpp` and filter hierarchy | `avraero/control/Filter.hpp` etc. from `avraero::control` |
| `include/control/Integrator.hpp` etc. | Corresponding `avraero::control` headers |
| `include/control/SISOPIDFF.hpp` | `avraero/control/SISOPIDFF.hpp` from `avraero::control` |
| `include/environment/TerrainVertex.hpp` etc. (migrated terrain element types) | Corresponding `avraero::terrain` headers |
| `include/environment/Terrain.hpp` (`V_Terrain`) | `avraero/terrain/V_Terrain.hpp` from `avraero::terrain` |

**LiteAeroSim stub headers removed** (FlightCode stubs now live in `flightcode`):

- `include/control/Autopilot.hpp`
- `include/guidance/PathGuidance.hpp`, `VerticalGuidance.hpp`, `ParkTracking.hpp`
- `include/path/Path.hpp`, `PathSegmentHelix.hpp`, `V_PathSegment.hpp`

**`KinematicState` → `KinematicStateSnapshot`:** All LiteAeroSim call sites that use
`KinematicState` are updated to use `KinematicStateSnapshot` from the shared interface
target. The `Aircraft::state()` return type changes to `KinematicStateSnapshot`.
`KinematicState.hpp` and `KinematicState.cpp` are removed from LiteAeroSim after migration.
`KinematicState_test.cpp` is removed; tests for the snapshot struct already exist in the
flightcode interfaces test (Step 7).

**`AircraftCommand`:** All LiteAeroSim references to `liteaerosim::AircraftCommand` are
updated to the shared interface target's `AircraftCommand`. The definition is removed from
`Aircraft.hpp`.

**Verification:** `cmake --build build && build/test/liteaerosim_test.exe` — all LiteAeroSim
tests pass. flightcode tests are unchanged.

---

### Step 13 — Final Build and Regression Verification

Run both repositories' test suites in sequence and confirm:

1. **flightcode:** All tests pass (same baseline as Step 10).
2. **LiteAeroSim:** All tests pass (same count as before Step 11; zero regressions).
3. **Namespace audit:** `grep -r "liteaerosim" include/ src/` returns no results in either
   repository (any remaining occurrences indicate an incomplete migration).
4. **Dead header audit:** No files in `LiteAeroSim/include/` or `LiteAeroSim/src/` are
   copies of files now in `flightcode/` (verify no stale duplicates).

Update `docs/roadmap/README.md` cross-cutting milestones table: mark `flightcode`
repository creation ✅ and Repo split and namespace migration ✅.

Update `docs/roadmap/flight_code.md` Current State table: update all stub paths to their
`flightcode` locations.

---

## Risk Notes

1. **`KinematicState` to `KinematicStateSnapshot` redesign (Step 6)** — the most
   significant design risk. The current `KinematicState` has 15+ computed methods that are
   called by the control subsystem and tests. Determining which are stored vs. derived in the
   snapshot requires understanding all call sites. A full call-site audit before writing the
   Step 6 design document is recommended.

2. **`TerrainTile` split (Step 8)** — `TerrainTile` bundles tile geometry (which migrates)
   with serialization and LOD selection state (which does not). If the class cannot be split
   cleanly without breaking `TerrainMesh`, `LodSelector`, and `.las_terrain` I/O, introduce
   a `TerrainTileData` plain struct and keep `TerrainTile` in `avraero::simulation` as a
   thin wrapper.

3. **Shared interface target name (Step 6)** — naming is a prerequisite for Steps 7, 11,
   and 12. Resolve before starting Step 7.

4. **`WGS84_Datum` in `KinematicStateSnapshot`** — `WGS84_Datum` is currently a LiteAeroSim
   type. If `KinematicStateSnapshot` includes it, `WGS84_Datum` must also migrate to
   `flightcode`. If the snapshot uses lat/lon/alt floats instead, the dependency is avoided.

5. **Phase 2 is a large single changeset** — Step 12 touches every source file in
   LiteAeroSim. Work in a feature branch; keep CI green throughout the branch with at least
   one intermediate commit per subsystem updated.
