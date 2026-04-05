// liteaero_sim_py — pybind11 extension module entry point.
//
// Design authority: docs/architecture/python_bindings.md
//
// Each subsystem's exported symbols are registered by a dedicated bind_*()
// function defined in a co-located source file.  Add new subsystems by
// declaring and calling a new bind_*() function here.

#include <pybind11/pybind11.h>

namespace py = pybind11;

void bind_manual_input(py::module_& m);

PYBIND11_MODULE(liteaero_sim_py, m)
{
    m.doc() = "LiteAero Sim Python bindings";
    bind_manual_input(m);
}
