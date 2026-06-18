import pytest

import control.control_service as control_service
from control.control_service import ControlGains, build_6dof
from schema.pilot_common import PilotAxes, PilotFrame


def test_default_stick_mapping_matches_pilot_translation_model():
    gains = ControlGains(surge=1.0, sway=1.0, heave=1.0, yaw=1.0, pitch=1.0, roll=1.0)

    forward = build_6dof(PilotFrame(axes=PilotAxes(ly=1.0)), gains)
    assert forward["surge"] == pytest.approx(1.0)
    assert forward["sway"] == pytest.approx(0.0)
    assert forward["yaw"] == pytest.approx(0.0)

    # Sway and yaw axes are swapped: sway is on the right stick X (rx),
    # yaw is on the left stick X (lx).
    crab_left = build_6dof(PilotFrame(axes=PilotAxes(rx=-1.0)), gains)
    assert crab_left["surge"] == pytest.approx(0.0)
    assert crab_left["sway"] == pytest.approx(-1.0)
    assert crab_left["yaw"] == pytest.approx(0.0)

    crab_right = build_6dof(PilotFrame(axes=PilotAxes(rx=1.0)), gains)
    assert crab_right["surge"] == pytest.approx(0.0)
    assert crab_right["sway"] == pytest.approx(1.0)
    assert crab_right["yaw"] == pytest.approx(0.0)

    yaw_right = build_6dof(PilotFrame(axes=PilotAxes(lx=1.0)), gains)
    assert yaw_right["surge"] == pytest.approx(0.0)
    assert yaw_right["sway"] == pytest.approx(0.0)
    assert yaw_right["yaw"] == pytest.approx(1.0)

    vertical = build_6dof(PilotFrame(axes=PilotAxes(ry=1.0)), gains)
    assert vertical["heave"] == pytest.approx(1.0)


def test_build_6dof_applies_dpad_pitch_roll_inverts(monkeypatch):
    monkeypatch.setattr(control_service.cfg, "AXIS_PITCH", "dpad_y", raising=False)
    monkeypatch.setattr(control_service.cfg, "AXIS_ROLL", "dpad_x", raising=False)
    monkeypatch.setattr(control_service.cfg, "AXIS_PITCH_INVERT", -1.0, raising=False)
    monkeypatch.setattr(control_service.cfg, "AXIS_ROLL_INVERT", -1.0, raising=False)

    cmd = build_6dof(PilotFrame(dpad=(1, 1)), ControlGains(pitch=1.0, roll=1.0))

    assert cmd["pitch"] == pytest.approx(-1.0)
    assert cmd["roll"] == pytest.approx(-1.0)


def test_default_roll_dpad_direction_uses_configured_invert():
    cmd = build_6dof(PilotFrame(dpad=(1, 0)), ControlGains(roll=1.0))

    assert cmd["roll"] == pytest.approx(-1.0)
