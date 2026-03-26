# Flight Code Migration — History Repair Plan

## Problem Statement

The flight code migration violated the stated requirement that all migrated files retain
their full developmental git history in `liteaero-flight`, accessible via
`git log --follow` on each file's current path. Two distinct root causes:

### Root Cause 1 — `ControlLoop.hpp` deleted instead of renamed in step 1

`include/control/ControlLoop.hpp` was included in the `git filter-repo` paths file and its
history (5 commits) was correctly extracted into `liteaero-flight`. However, in commit
`cda31a1` (flight code migration step 1), the file was **deleted** (`D`) rather than
renamed to its correct destination under `include/liteaero/autopilot/`. This broke the path
chain: `git log --follow include/liteaero/autopilot/ControlLoop.hpp` cannot trace back
through the deletion.

### Root Cause 2 — Control* subclass files excluded from `git filter-repo` paths

The `git filter-repo` paths file omitted the 12 Control* subclass files. Those files and
their complete 13-commit developmental history exist **only in `liteaero-sim`**. They have
zero commits in `liteaero-flight`. Their history cannot be connected to `liteaero-flight`
without re-running `git filter-repo` with an updated paths file.

These 12 files are the ones that should have been classified as flight code from the
beginning:

| File | Commits in sim | Commits in flight |
| --- | --- | --- |
| `include/control/ControlAltitude.hpp` | 13 | 0 |
| `include/control/ControlHeading.hpp` | 13 | 0 |
| `include/control/ControlHeadingRate.hpp` | 13 | 0 |
| `include/control/ControlLoadFactor.hpp` | 13 | 0 |
| `include/control/ControlRoll.hpp` | 13 | 0 |
| `include/control/ControlVerticalSpeed.hpp` | 13 | 0 |
| `src/control/ControlAltitude.cpp` | 13 | 0 |
| `src/control/ControlHeading.cpp` | 13 | 0 |
| `src/control/ControlHeadingRate.cpp` | 13 | 0 |
| `src/control/ControlLoadFactor.cpp` | 13 | 0 |
| `src/control/ControlRoll.cpp` | 13 | 0 |
| `src/control/ControlVerticalSpeed.cpp` | 13 | 0 |

---

## Verification That Other Migrated Files Are Correct

Before proceeding: `git log --follow include/liteaero/control/SISOPIDFF.hpp` in
`liteaero-flight` returns 12 commits tracing back to the original sim-era development.
The `R100` renames performed in `cda31a1` for all other files work correctly.
The problem is confined to the 13 Control* files listed above.

---

## Current Uncommitted State (Today's Work — Not Yet Committed)

**`liteaero-sim`** — 13 staged deletions:
- `D include/control/ControlLoop.hpp`
- `D include/control/ControlAltitude.hpp` (×5 more headers)
- `D src/control/ControlAltitude.cpp` (×5 more sources)

**`liteaero-flight`** — untracked new files + 2 modified tracked files:
- `?? include/liteaero/autopilot/ControlLoop.hpp` (×6 more headers)
- `?? src/autopilot/` (6 source files)
- `M src/CMakeLists.txt` (liteaero::autopilot target added)
- `M docs/roadmap/flight_code.md` (Current State table updated)

---

## Fix Approach

Because the Control* subclass files have no history in `liteaero-flight`, **re-running
`git filter-repo` is required**. There is no other way to bring their developmental
history into the flight repository. This rewrites `liteaero-flight`'s commit SHAs from
the beginning and requires a force-push.

### Phase 1 — Undo today's uncommitted work

**`liteaero-sim`:** restore all 13 staged deletions:
```bash
cd liteaero-sim
git checkout -- include/control/ src/control/
```

**`liteaero-flight`:** delete untracked files and revert modified tracked files:
```bash
cd liteaero-flight
rm -rf include/liteaero/autopilot/Control*.hpp src/autopilot/
git checkout -- src/CMakeLists.txt docs/roadmap/flight_code.md
```

After Phase 1: both repos are back to their last committed state. No history has been
touched.

### Phase 2 — Re-run `git filter-repo` with updated paths

Add the 12 missing Control* files to the paths file and re-run filter-repo from a fresh
clone of `liteaero-sim`. The updated paths file additions (to append to the existing
file):

```text
include/control/ControlAltitude.hpp
include/control/ControlHeading.hpp
include/control/ControlHeadingRate.hpp
include/control/ControlLoadFactor.hpp
include/control/ControlRoll.hpp
include/control/ControlVerticalSpeed.hpp
src/control/ControlAltitude.cpp
src/control/ControlHeading.cpp
src/control/ControlHeadingRate.cpp
src/control/ControlLoadFactor.cpp
src/control/ControlRoll.cpp
src/control/ControlVerticalSpeed.cpp
```

Procedure:
```bash
# Fresh clone of liteaero-sim — becomes the new liteaero-flight base
git clone /c/Users/Alex/avraero/liteaero/liteaero-sim liteaero-flight-new
cd liteaero-flight-new
git remote remove origin
git filter-repo --paths-from-file ../liteaero-flight-migrate-paths-updated.txt
```

This produces a repo whose full history includes all 13 Control* files. All commit SHAs
are rewritten (filter-repo always rewrites SHAs). This new repo becomes the base onto
which the migration commits are replayed.

### Phase 3 — Replay the 11 post-step-1 migration commits

The current `liteaero-flight` has 11 commits after `cda31a1` (steps 1–12 + one doc
commit). These need to be replayed onto the new filtered base.

The `cda31a1` equivalent commit must be corrected: instead of deleting
`include/control/ControlLoop.hpp`, it must `git mv` it to
`include/liteaero/autopilot/ControlLoop.hpp`. The other 12 Control* files must also be
`git mv`'d to their destination paths in this same step-1 commit (or an immediately
following rename commit).

Destination paths:

| Source (sim path) | Destination (flight path) |
| --- | --- |
| `include/control/ControlLoop.hpp` | `include/liteaero/autopilot/ControlLoop.hpp` |
| `include/control/ControlAltitude.hpp` | `include/liteaero/autopilot/ControlAltitude.hpp` |
| `include/control/ControlHeading.hpp` | `include/liteaero/autopilot/ControlHeading.hpp` |
| `include/control/ControlHeadingRate.hpp` | `include/liteaero/autopilot/ControlHeadingRate.hpp` |
| `include/control/ControlLoadFactor.hpp` | `include/liteaero/autopilot/ControlLoadFactor.hpp` |
| `include/control/ControlRoll.hpp` | `include/liteaero/autopilot/ControlRoll.hpp` |
| `include/control/ControlVerticalSpeed.hpp` | `include/liteaero/autopilot/ControlVerticalSpeed.hpp` |
| `src/control/ControlAltitude.cpp` | `src/autopilot/ControlAltitude.cpp` |
| `src/control/ControlHeading.cpp` | `src/autopilot/ControlHeading.cpp` |
| `src/control/ControlHeadingRate.cpp` | `src/autopilot/ControlHeadingRate.cpp` |
| `src/control/ControlLoadFactor.cpp` | `src/autopilot/ControlLoadFactor.cpp` |
| `src/control/ControlRoll.cpp` | `src/autopilot/ControlRoll.cpp` |
| `src/control/ControlVerticalSpeed.cpp` | `src/autopilot/ControlVerticalSpeed.cpp` |

The remaining 10 post-step-1 commits (steps 2–12 + doc) do not touch the Control* files
and should replay cleanly via `git cherry-pick` or `git rebase` onto the new base.

### Phase 4 — Apply content changes to Control* files

After the renames are committed, a second commit applies the content changes required by
the new home in `liteaero-flight`:

- Namespace: `liteaero::simulation` → `liteaero::autopilot`
- Include paths: `"control/ControlLoop.hpp"` → `<liteaero/autopilot/ControlLoop.hpp>`
- State type: `KinematicState` → `liteaero::nav::KinematicStateSnapshot`
- Accessor calls: `KinematicState` methods → `KinematicStateSnapshot` fields +
  `KinematicStateUtil` free functions
- Base class: `ControlLoop.hpp` drops `using json = nlohmann::json;`

The `liteaero::autopilot` CMake target is added to `src/CMakeLists.txt` in this same
commit (or the step-1 commit, whichever is cleaner).

### Phase 5 — Force-push `liteaero-flight` and commit `liteaero-sim` deletion

**`liteaero-flight`:** force-push the rewritten history to remote.

**`liteaero-sim`:** commit the deletion of all 13 Control* files from `include/control/`
and `src/control/`, with a commit message referencing the liteaero-flight destination.

---

## Scale

| Metric | Value |
| --- | --- |
| `liteaero-flight` commits rewritten | 100 (all — filter-repo rewrites from root) |
| Post-step-1 commits to replay | 11 |
| Files renamed in step-1 replay | 13 |
| Files with content changes | 13 |
| Files deleted from `liteaero-sim` | 13 |
| Force-push required | Yes |

**Risk:** All `liteaero-flight` commit SHAs change. Any external reference to current
SHAs (bookmarks, CI run references, cross-repo links in documentation) will be
invalidated. Given this is a single-developer project the practical impact is low, but
any local `liteaero-flight` clones other than the one at
`c:/Users/Alex/avraero/liteaero/liteaero-flight` would need to be re-cloned.

---

## Verification

After the repair, for each of the 13 Control* files:

```bash
git log --follow include/liteaero/autopilot/ControlAltitude.hpp
```

Must show the full chain from the content-change commit, through the rename commit, back
through the pre-fork developmental history to the original "work in progress implementing
control loops" commit (`d2dfe93` equivalent in the rewritten history).
