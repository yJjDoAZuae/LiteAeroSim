"""Pytest session configuration for liteaero-sim Python tests."""
import os
import sys

import matplotlib

matplotlib.use("Agg")

# On Windows, Python 3.8+ no longer searches PATH for DLL dependencies of
# extension modules.  Register the MSYS2 ucrt64 bin directory so that
# liteaero_sim_py.pyd can find SDL2.dll and the MinGW runtime DLLs.
if sys.platform == "win32":
    _ucrt64_bin = r"C:\msys64\ucrt64\bin"
    if os.path.isdir(_ucrt64_bin):
        os.add_dll_directory(_ucrt64_bin)
