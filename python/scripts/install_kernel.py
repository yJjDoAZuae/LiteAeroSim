"""Install the custom Jupyter kernel spec for this project.

Run once after initial setup, or after deliberately rebuilding the venv
(e.g. ``rm -rf .venv && uv sync`` or a ``.python-version`` change):

    uv run python scripts/install_kernel.py

Normal ``uv sync`` (package updates, no venv rebuild) does NOT require
re-running this script — the kernel spec persists in
``.venv/share/jupyter/kernels/``.

The installed kernel.json points at ``scripts/kernel_launcher.py`` instead
of the default ``ipykernel_launcher``.  The launcher applies the ipykernel
PR #1462 patch at every kernel startup without modifying installed packages.
See ``scripts/kernel_launcher.py`` for the full diagnosis.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path


def main() -> None:
    python_exe = Path(sys.executable).resolve()

    # Derive paths relative to the Python executable so this script works
    # regardless of where the project is cloned.
    #
    # Layout (Windows):  <project>/python/.venv/Scripts/python.exe
    # Layout (Unix):     <project>/python/.venv/bin/python
    #
    # python_exe.parent       → .venv/Scripts  (or .venv/bin)
    # python_exe.parent.parent → .venv
    # venv_root.parent         → python/
    venv_root = python_exe.parent.parent
    project_python = venv_root.parent
    launcher = project_python / "scripts" / "kernel_launcher.py"

    if not launcher.exists():
        raise FileNotFoundError(
            f"kernel_launcher.py not found at {launcher}\n"
            "Make sure you are running from the project root:\n"
            "    uv run python scripts/install_kernel.py"
        )

    kernel_spec = {
        "argv": [str(python_exe), str(launcher), "-f", "{connection_file}"],
        "display_name": "Python 3 (ipykernel)",
        "language": "python",
        "metadata": {"debugger": True},
    }

    kernel_dir = venv_root / "share" / "jupyter" / "kernels" / "python3"
    kernel_dir.mkdir(parents=True, exist_ok=True)
    kernel_json = kernel_dir / "kernel.json"
    kernel_json.write_text(json.dumps(kernel_spec, indent=1))

    print(f"Kernel spec installed at: {kernel_json}")
    print(f"  Python:   {python_exe}")
    print(f"  Launcher: {launcher}")
    print()
    print("Run 'uv run jupyter lab' to start JupyterLab.")


if __name__ == "__main__":
    main()
