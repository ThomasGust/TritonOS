import pytest

import control.control_service as control_service
from control.control_service import ControlGains, build_6dof
from schema.pilot_common import PilotFrame


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
