"""Custom Jupyter kernel launcher that patches ipykernel PR #1462 before startup.

This file is the kernel executable registered in:
    .venv/share/jupyter/kernels/python3/kernel.json

It is NOT managed by uv.  It lives in the project repo and persists across
``uv sync`` runs.  The kernel spec (kernel.json) is installed once via:

    uv run python scripts/install_kernel.py

Run install_kernel.py again only when the Python interpreter changes (i.e.,
after a ``.python-version`` change + ``uv sync``).

-------------------------------------------------------------------------------
Why this file exists — ipykernel PR #1462 shared-context bug
-------------------------------------------------------------------------------
ipykernel 7.1.0 introduced ``_async_in_context()`` in ``ipykernel/utils.py``
(PR #1462) to fix ContextVar persistence across notebook cells.  The
implementation captures a single ``Context`` object at startup and passes it
to every ``asyncio.create_task()`` call via the ``context=`` keyword:

    context = copy_context()          # captured ONCE at closure creation

    async def run_in_context(*args, **kwargs):
        return await asyncio.create_task(f(*args, **kwargs), context=context)

CPython's ``Context.run()`` enforces mutual exclusion on every Python version:
a Context can only be entered by one execution frame at a time.  When
pyvista activates a trame widget, trame's tornado server schedules callbacks
from a background thread that shares the same ``context`` object with the
asyncio event loop running ``shell_main``.  The second ``PyContext_Enter``
fails:

    RuntimeError: cannot enter context: <Context object> is already entered

The error appears in the JupyterLab console as:
    Exception in callback Task.__step()

This patch replaces the shared closure object with ``copy_context()`` called
inside ``run_in_context``, so each ``create_task`` call gets a fresh snapshot
of the current ContextVar state.  ContextVar mutations within one task are
no longer visible to subsequent tasks — an acceptable trade-off for our use
case.

Re-evaluation trigger
---------------------
Check whether the upstream fix has been merged when ipykernel releases a
version > 7.2.0.  See the re-evaluation steps in
``docs/dependencies/README.md``.  When confirmed fixed upstream:

1. Remove the ``_apply_patch()`` call below (keep the function for reference).
2. Update ``_PATCH_IPYKERNEL_VERSION_RANGE`` to note the fix release.
3. Update ``docs/dependencies/README.md``.
4. Run ``python/test/test_ipykernel_compat.py`` to confirm tests still pass.
"""

from __future__ import annotations

import asyncio
import functools
import importlib.metadata
import sys
import warnings
from contextvars import copy_context

# Inclusive range of ipykernel versions known to have the shared-context bug.
# Update the upper bound when upstream ships a fix.
_PATCH_IPYKERNEL_VERSION_RANGE = ("7.1.0", "7.2.0")


def _apply_patch() -> None:
    """Monkey-patch ipykernel.utils._async_in_context before kernelbase loads it.

    Must run before ``ipykernel.kernelbase`` is imported because kernelbase does:

        from .utils import _async_in_context

    which binds the name at import time.  Since we import ``ipykernel.utils``
    here and patch it before importing ``ipykernel.kernelapp`` (which triggers
    the kernelbase import), the patched function is what kernelbase binds.
    """
    try:
        from ipykernel import utils as _utils  # noqa: PLC0415
    except ImportError:
        return  # ipykernel not installed

    if not hasattr(_utils, "_async_in_context"):
        return  # ipykernel < 7.1.0 — _async_in_context does not exist

    if sys.version_info < (3, 11):
        return  # pre-3.11 code path in _async_in_context is not affected

    try:
        installed = importlib.metadata.version("ipykernel")
    except importlib.metadata.PackageNotFoundError:
        installed = "unknown"

    if installed > _PATCH_IPYKERNEL_VERSION_RANGE[1]:
        warnings.warn(
            f"ipykernel {installed} is newer than the last known buggy version "
            f"({_PATCH_IPYKERNEL_VERSION_RANGE[1]}).  The shared-context patch "
            "in scripts/kernel_launcher.py may no longer be needed.  Check "
            "docs/dependencies/README.md for re-evaluation steps.",
            stacklevel=1,
        )

    _orig = _utils._async_in_context

    @functools.wraps(_orig)
    def _patched(
        f: object,
        context: object = None,  # noqa: ARG001 — ignored; fresh copy per call
    ) -> object:
        """Fixed _async_in_context: each task gets its own context snapshot.

        The original captures ``context = copy_context()`` once at closure-
        creation time and reuses it for every ``asyncio.create_task()`` call.
        The fix calls ``copy_context()`` inside ``run_in_context`` so each
        task gets an independent snapshot of the current ContextVar state.
        """
        @functools.wraps(f)  # type: ignore[arg-type]
        async def run_in_context(*args: object, **kwargs: object) -> object:
            return await asyncio.create_task(
                f(*args, **kwargs),  # type: ignore[operator]
                context=copy_context(),  # fresh copy — not the shared closure object
            )

        return run_in_context

    _utils._async_in_context = _patched


_apply_patch()

# ---------------------------------------------------------------------------
# Normal kernel launch — identical to ``python -m ipykernel_launcher``
# Must be imported AFTER _apply_patch() so kernelbase binds the patched name.
# ---------------------------------------------------------------------------
from ipykernel import kernelapp as app  # noqa: E402

app.launch_new_instance()
