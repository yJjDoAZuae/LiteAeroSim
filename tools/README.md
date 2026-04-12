# `live_sim` — Real-Time Simulation Launcher

`live_sim.exe` runs the C++ simulation engine in real time, streams position and
attitude to Godot 4 over UDP, and applies landing gear contact forces against the
actual terrain mesh. It is the operator-facing binary for joystick-driven flight.

**Architecture authority:** [`docs/architecture/live_sim_view.md`](../docs/architecture/live_sim_view.md)

---

## Data Flow

```text
Aircraft config JSON
        │
        ▼
build_terrain.py ────────────────────────────────────────┐
  Step 1  Download DEM + imagery (Copernicus GLO-30)      │
  Step 2  Mosaic chunks → seamless rasters                │
  Step 3  Triangulate + colorize  LOD 0–6                 │
  Step 4  Write  data/terrain/<name>/derived/             │
          las_terrain/terrain.las_terrain  (all LODs)     │
  Step 5  Select display tiles (geographic partitioning)  │
  Step 6  Export  godot/terrain/terrain.glb               │
  Step 7  Write   godot/terrain/terrain_config.json  ◄────┤
  Step 8  Write   data/terrain/<name>/derived/            │
          metadata.json  (presence signals success)       │
                                                          │
        ┌───────────────────────────────────────────────┐ │
        │ live_sim.exe                                  │ │
        │   reads terrain_config.json ◄─────────────────┘ │
        │   loads terrain.las_terrain ◄───────────────────┘
        │   constructs TerrainMesh                        │
        │   aircraft.setTerrain(terrain_mesh)             │
        │   SimRunner (RealTime, dt = 0.02 s)             │
        │     Aircraft::step()                            │
        │       LandingGear::step(..., terrain)           │
        │     UdpSimulationBroadcaster::broadcast()       │
        │       UDP 127.0.0.1:14560                       │
        └──────────────────┬────────────────────────────┘
                           │
                           ▼
                   Godot 4 scene
                     SimulationReceiver.gd
                       geodetic → ENU offset
                       body-to-NED quat → Godot quat
                       update Vehicle transform
                       update RibbonTrail
```

---

## Prerequisites

### 1. Build

All commands run from the `liteaero-sim/` directory with `ucrt64/bin` on `PATH`.

**Bash / Git Bash:**

```bash
cd liteaero-sim
export PATH="/c/msys64/ucrt64/bin:$PATH"

# First time only — install Conan packages:
conan install . --output-folder=build \
    --build=missing --profile=liteaero-gcc

# Configure and build:
cmake -B build -G "MinGW Makefiles" \
    -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release
mingw32-make -C build live_sim
```

**PowerShell:**

```powershell
cd liteaero-sim
$env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"

# First time only — install Conan packages:
conan install . --output-folder=build `
    --build=missing --profile=liteaero-gcc

# Configure and build:
cmake -B build -G "MinGW Makefiles" `
    -DCMAKE_TOOLCHAIN_FILE=build\conan_toolchain.cmake `
    -DCMAKE_BUILD_TYPE=Release
mingw32-make -C build live_sim
```

The executable lands at `build/tools/live_sim.exe`.
`SDL2.dll` is copied alongside it automatically at build time.

### 2. Python virtual environment

`build_terrain.py` and all other Python tools run inside the project virtual
environment, managed with [uv](https://docs.astral.sh/uv/).

**Install uv** (once per machine):

```powershell
winget install astral-sh.uv
```

**Create / sync the venv** (once, or after any `pyproject.toml` change):

```bash
cd python
uv sync --group dev
```

This installs all runtime dependencies (`rasterio`, `scipy`, `trimesh`, `pyproj`,
etc.) and dev tools (`pytest`, `jupyterlab`, `black`, `ruff`) into
`python/.venv/`. The venv is not on `PATH` — prefix all Python commands with
`uv run` to use it.

Run Python tests:

```bash
cd python
uv run pytest
```

Start JupyterLab:

```bash
cd python
uv run jupyter lab
```

### 3. Terrain dataset

The terrain pipeline produces the `.las_terrain` binary that `live_sim.exe` loads
at startup. Run it once per location, or any time the aircraft config changes.
`build_terrain.py` must run inside the project venv (step 2 above).
All commands run from the `python/` directory.

```bash
cd python

# Build terrain for the small UAS at KSBA (centroid and radius from config):
uv run python tools/terrain/build_terrain.py ../configs/small_uas_ksba.json

# Override coverage radius:
uv run python tools/terrain/build_terrain.py ../configs/general_aviation_ksba.json --radius-km 60

# Force full rebuild (keeps cached DEM / imagery downloads):
uv run python tools/terrain/build_terrain.py ../configs/small_uas_ksba.json --force

# Detailed per-step progress:
uv run python tools/terrain/build_terrain.py ../configs/small_uas_ksba.json --verbose
```

Outputs written:

| Path | Consumer |
| --- | --- |
| `data/terrain/<name>/derived/las_terrain/terrain.las_terrain` | `live_sim.exe` (physics) |
| `godot/terrain/terrain.glb` | Godot 4 scene (rendering) |
| `godot/terrain/terrain_config.json` | Both — links all outputs together |
| `data/terrain/<name>/derived/metadata.json` | Provenance record |

`terrain_config.json` stores the absolute path to `terrain.las_terrain` in
`las_terrain_path`. If this field is absent (e.g., the file was generated before
the field was added), regenerate it by re-running `build_terrain.py`.

### 3. Godot scene

Open the Godot 4 project at `godot/` in the Godot editor and press **Play** before
or after starting `live_sim.exe`. The scene listens on port 14560 and starts
rendering as soon as the first UDP datagram arrives. If Godot is not running,
datagrams are silently dropped — the simulation continues unaffected.

---

## Running

### Joystick mode (GX12 or compatible SDL2 device)

**Bash / Git Bash:**

```bash
# List available SDL joystick indices first:
build/tools/joystick_verify.exe

build/tools/live_sim.exe \
    --config   configs/small_uas_ksba.json \
    --joystick python/gx12_config.json \
    --device   0
```

**PowerShell:**

```powershell
# List available SDL joystick indices first:
build\tools\joystick_verify.exe

build\tools\live_sim.exe `
    --config   configs\small_uas_ksba.json `
    --joystick python\gx12_config.json `
    --device   0
```

### Scripted mode (neutral commands — useful for testing the pipeline end-to-end)

**Bash / Git Bash:**

```bash
build/tools/live_sim.exe \
    --config configs/small_uas_ksba.json
```

**PowerShell:**

```powershell
build\tools\live_sim.exe `
    --config configs\small_uas_ksba.json
```

### All arguments

| Argument | Type | Default | Description |
| --- | --- | --- | --- |
| `--config` | path | required | Aircraft JSON config file |
| `--joystick` | path | — | Joystick JSON config; enables joystick mode |
| `--device` | int | `0` | SDL device index to use with `--joystick` |
| `--dt` | float | `0.02` | Simulation timestep (s) |
| `--port` | int | `14560` | UDP broadcast port |

Press `Ctrl+C` to stop. The binary prints elapsed simulated time on exit.

### Environment variable

`LITEAERO_GODOT_DIR` overrides the default `godot/` path when locating
`terrain_config.json`. Useful when the Godot project is checked out to a
non-standard location:

**Bash / Git Bash:**

```bash
export LITEAERO_GODOT_DIR=/path/to/godot/project
build/tools/live_sim.exe --config ...
```

**PowerShell:**

```powershell
$env:LITEAERO_GODOT_DIR = "C:\path\to\godot\project"
build\tools\live_sim.exe --config ...
```

---

## Aircraft configs

Pre-built configs live in `configs/`. Each config includes an `initial_state`
(geodetic position, velocity, wind), a `landing_gear` section (spring/damper
parameters for each wheel unit), and a `terrain.radius_km` field that controls
how large a coverage area `build_terrain.py` downloads.

| Config | Aircraft | Location |
| --- | --- | --- |
| `configs/small_uas_ksba.json` | 5 kg fixed-wing UAS | KSBA runway 07 threshold |
| `configs/general_aviation_ksba.json` | General aviation | KSBA |
| `configs/jet_trainer_ksba.json` | Jet trainer | KSBA |

All three start at rest on the ground (`velocity_* = 0`). Landing gear contact
forces are active from the first simulation step.

---

## Joystick config format (`gx12_config.json`)

```json
{
    "device_name_contains": "GX12",
    "dead_zone_nd": 0.05,
    "nz_axis":        { "sdl_axis_index": 1, "center_output": 1.0, "scale": 3.0,      "inverted": true  },
    "ny_axis":        { "sdl_axis_index": 3, "center_output": 0.0, "scale": 1.0,      "inverted": false },
    "roll_axis":      { "sdl_axis_index": 0, "center_output": 0.0, "scale": 1.5708,   "inverted": false },
    "throttle_axis":  { "sdl_axis_index": 2, "center_output": 0.5, "scale": 0.5,      "inverted": false },
    "min_throttle_nd": 0.0,
    "idle_throttle_nd": 0.05,
    "min_nz_g": -2.0,
    "max_nz_g":  4.0,
    "max_ny_g":  2.0,
    "max_roll_rate_rad_s": 1.5708
}
```

Use `joystick_verify.exe` to print raw axis indices and values for a connected
device while configuring a new joystick mapping.

---

## Startup sequence

`live_sim.exe` prints each initialization step to stdout. A successful startup looks like:

```
Terrain loaded from C:/.../data/terrain/small_uas_ksba/derived/las_terrain/terrain.las_terrain
Joystick mode: device 0 from python/gx12_config.json
Broadcasting to 127.0.0.1:14560
Starting live simulation (dt=0.02 s).  Press Ctrl+C to stop.
```

**Hard errors on startup** (non-zero exit):

| Message | Cause | Fix |
| --- | --- | --- |
| `terrain_config.json not found` | `build_terrain.py` not yet run, or wrong working directory / `LITEAERO_GODOT_DIR` | Run `build_terrain.py`; check path |
| `terrain_config.json missing 'las_terrain_path'` | Config generated before `las_terrain_path` field was added | Re-run `build_terrain.py` |
| `.las_terrain file not found` | Binary deleted or path stale | Re-run `build_terrain.py` |
| `Aircraft initialization failed` | Malformed aircraft config JSON | Fix the config |
| `SDL_Init failed` | No SDL2 joystick subsystem | Ensure `SDL2.dll` is alongside the binary |
