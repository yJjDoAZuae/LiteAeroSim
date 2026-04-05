"""Tests for liteaero_sim_py manual input bindings.

Design authority: docs/architecture/python_bindings.md

The module is an optional build target (LITEAERO_SIM_BUILD_PYTHON_BINDINGS=ON).
These tests are skipped automatically when the module is not built.
"""
import pytest

liteaero_sim_py = pytest.importorskip(
    "liteaero_sim_py",
    reason="liteaero_sim_py not built (set LITEAERO_SIM_BUILD_PYTHON_BINDINGS=ON)",
)


# ---------------------------------------------------------------------------
# AircraftCommand
# ---------------------------------------------------------------------------


def test_aircraft_command_defaults():
    cmd = liteaero_sim_py.AircraftCommand()
    assert cmd.n_z == pytest.approx(1.0)
    assert cmd.n_y == pytest.approx(0.0)
    assert cmd.roll_rate_wind_rps == pytest.approx(0.0)
    assert cmd.throttle_nd == pytest.approx(0.0)


def test_aircraft_command_kwargs():
    cmd = liteaero_sim_py.AircraftCommand(
        n_z=2.0, n_y=0.5, roll_rate_wind_rps=0.3, throttle_nd=0.8
    )
    assert cmd.n_z == pytest.approx(2.0)
    assert cmd.n_y == pytest.approx(0.5)
    assert cmd.roll_rate_wind_rps == pytest.approx(0.3)
    assert cmd.throttle_nd == pytest.approx(0.8)


def test_aircraft_command_attribute_write():
    cmd = liteaero_sim_py.AircraftCommand()
    cmd.n_z = 3.0
    assert cmd.n_z == pytest.approx(3.0)
    cmd.throttle_nd = 0.6
    assert cmd.throttle_nd == pytest.approx(0.6)


# ---------------------------------------------------------------------------
# ScriptedInput
# ---------------------------------------------------------------------------


def test_scripted_input_push_then_read():
    scripted = liteaero_sim_py.ScriptedInput()
    scripted.initialize({})

    cmd = liteaero_sim_py.AircraftCommand(
        n_z=2.5, n_y=0.1, roll_rate_wind_rps=0.2, throttle_nd=0.7
    )
    scripted.push(cmd)
    frame = scripted.read()

    assert frame.command.n_z == pytest.approx(2.5)
    assert frame.command.n_y == pytest.approx(0.1)
    assert frame.command.roll_rate_wind_rps == pytest.approx(0.2)
    assert frame.command.throttle_nd == pytest.approx(0.7)
    assert frame.actions == 0


def test_scripted_input_reset_returns_neutral():
    scripted = liteaero_sim_py.ScriptedInput()
    scripted.initialize({})

    scripted.push(liteaero_sim_py.AircraftCommand(n_z=3.0, throttle_nd=0.9))
    scripted.reset()
    frame = scripted.read()

    assert frame.command.n_z == pytest.approx(1.0)
    assert frame.command.n_y == pytest.approx(0.0)
    assert frame.command.roll_rate_wind_rps == pytest.approx(0.0)
    assert frame.command.throttle_nd == pytest.approx(0.0)


def test_scripted_input_initialize_no_args():
    scripted = liteaero_sim_py.ScriptedInput()
    scripted.initialize()  # no args → empty config, must not raise


# ---------------------------------------------------------------------------
# JoystickInput.enumerate_devices
# ---------------------------------------------------------------------------


def test_enumerate_devices_returns_list():
    liteaero_sim_py.sdl_init_joystick()
    try:
        devices = liteaero_sim_py.JoystickInput.enumerate_devices()
        assert isinstance(devices, list)
        for d in devices:
            assert "device_index" in d
            assert "name" in d
            assert "num_axes" in d
    finally:
        liteaero_sim_py.sdl_quit_joystick()
