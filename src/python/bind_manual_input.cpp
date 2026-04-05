// Manual input bindings — exports AircraftCommand, ScriptedInput, and
// JoystickInput::enumerateDevices() to Python.
//
// Design authority: docs/architecture/python_bindings.md

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "Aircraft.hpp"
#include "input/ManualInput.hpp"
#include "input/ScriptedInput.hpp"
#include "input/JoystickInput.hpp"
#include <nlohmann/json.hpp>
#include <SDL2/SDL.h>

namespace py = pybind11;
using namespace liteaero::simulation;

// ---------------------------------------------------------------------------
// Python dict → nlohmann::json conversion
// Bool must be checked before int because Python bool is a subtype of int.
// ---------------------------------------------------------------------------

static nlohmann::json py_to_json(const py::object& obj)
{
    if (py::isinstance<py::bool_>(obj))  return obj.cast<bool>();
    if (py::isinstance<py::int_>(obj))   return obj.cast<int64_t>();
    if (py::isinstance<py::float_>(obj)) return obj.cast<double>();
    if (py::isinstance<py::str>(obj))    return obj.cast<std::string>();
    if (py::isinstance<py::none>(obj))   return nullptr;

    if (py::isinstance<py::dict>(obj)) {
        nlohmann::json result = nlohmann::json::object();
        for (const auto& [k, v] : obj.cast<py::dict>()) {
            result[k.cast<std::string>()] = py_to_json(v.cast<py::object>());
        }
        return result;
    }

    if (py::isinstance<py::list>(obj) || py::isinstance<py::tuple>(obj)) {
        nlohmann::json result = nlohmann::json::array();
        for (const auto& item : obj) {
            result.push_back(py_to_json(item.cast<py::object>()));
        }
        return result;
    }

    return nlohmann::json{};
}

// ---------------------------------------------------------------------------

void bind_manual_input(py::module_& m)
{
    // --- AircraftCommand ---
    py::class_<AircraftCommand>(m, "AircraftCommand")
        .def(py::init([](float n_z, float n_y, float roll_rate_wind_rps, float throttle_nd) {
                AircraftCommand cmd;
                cmd.n_z               = n_z;
                cmd.n_y               = n_y;
                cmd.rollRate_Wind_rps = roll_rate_wind_rps;
                cmd.throttle_nd       = throttle_nd;
                return cmd;
             }),
             py::arg("n_z")                = 1.0f,
             py::arg("n_y")                = 0.0f,
             py::arg("roll_rate_wind_rps") = 0.0f,
             py::arg("throttle_nd")        = 0.0f)
        .def_readwrite("n_z",                &AircraftCommand::n_z)
        .def_readwrite("n_y",                &AircraftCommand::n_y)
        .def_readwrite("roll_rate_wind_rps", &AircraftCommand::rollRate_Wind_rps)
        .def_readwrite("throttle_nd",        &AircraftCommand::throttle_nd)
        .def("__repr__", [](const AircraftCommand& c) {
            return "AircraftCommand(n_z="              + std::to_string(c.n_z)
                 + ", n_y="                            + std::to_string(c.n_y)
                 + ", roll_rate_wind_rps="             + std::to_string(c.rollRate_Wind_rps)
                 + ", throttle_nd="                    + std::to_string(c.throttle_nd) + ")";
        });

    // --- ManualInputFrame (return type of ScriptedInput::read()) ---
    py::class_<ManualInputFrame>(m, "ManualInputFrame")
        .def_readwrite("command", &ManualInputFrame::command)
        .def_readwrite("actions", &ManualInputFrame::actions);

    // --- ScriptedInput ---
    py::class_<ScriptedInput>(m, "ScriptedInput")
        .def(py::init<>())
        .def("initialize",
             [](ScriptedInput& self, py::object config) {
                 self.initialize(py_to_json(config));
             },
             py::arg("config") = py::dict())
        .def("reset", &ScriptedInput::reset)
        .def("read",  &ScriptedInput::read)
        .def("push",  &ScriptedInput::push);

    // --- JoystickInput: static enumeration only ---
    py::class_<JoystickInput>(m, "JoystickInput")
        .def_static("enumerate_devices", []() {
            py::list result;
            for (const auto& d : JoystickInput::enumerateDevices()) {
                py::dict entry;
                entry["device_index"] = d.device_index;
                entry["name"]         = d.name;
                entry["num_axes"]     = d.num_axes;
                result.append(entry);
            }
            return result;
        });

    // --- SDL lifecycle helpers ---
    // SDL must be initialized before calling JoystickInput.enumerate_devices().
    // These helpers allow Python callers to manage the joystick subsystem
    // without a direct SDL dependency.
    m.def("sdl_init_joystick", []() {
        SDL_InitSubSystem(SDL_INIT_JOYSTICK);
    });
    m.def("sdl_quit_joystick", []() {
        SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
    });
}
