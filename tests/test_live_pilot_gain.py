from types import SimpleNamespace

import pytest

from control.control_service import ControlGains, ControlService
from control.mixer import global_limit


def _make_gain_service(base_power_scale: float = 0.75):
    svc = object.__new__(ControlService)
    svc.gains = ControlGains(power_scale=base_power_scale)
    svc._base_power_scale = float(base_power_scale)
    svc._last_pilot_max_gain = 1.0
    return svc


def test_live_pilot_max_gain_updates_effective_power_scale():
    svc = _make_gain_service(base_power_scale=0.75)

    ControlService._apply_pilot_gain(svc, SimpleNamespace(modes={"max_gain": 0.4}))
    assert svc._last_pilot_max_gain == pytest.approx(0.4)
    assert svc.gains.power_scale == pytest.approx(0.75 * 0.4)

    ControlService._apply_pilot_gain(svc, SimpleNamespace(modes={"max_gain": 0.8}))
    assert svc._last_pilot_max_gain == pytest.approx(0.8)
    assert svc.gains.power_scale == pytest.approx(0.75 * 0.8)


def test_rov_side_rejects_over_unity_gain_requests():
    svc = _make_gain_service(base_power_scale=0.75)

    ControlService._apply_pilot_gain(svc, SimpleNamespace(modes={"max_gain": 2.5}))

    assert svc._last_pilot_max_gain == pytest.approx(1.0)
    assert svc.gains.power_scale == pytest.approx(0.75)


def test_live_pilot_max_gain_caps_final_mixed_thrusters(monkeypatch):
    monkeypatch.setattr("control.control_service.cfg.THRUSTER_MAX_ABS", 1.0)
    svc = _make_gain_service()
    pilot = SimpleNamespace(modes={"max_gain": 0.4})

    cap = ControlService._live_thruster_max_abs(svc, pilot)
    limited = global_limit({"H_FL": 1.2, "H_FR": -0.8}, max_abs=cap)

    assert cap == pytest.approx(0.4)
    assert max(abs(v) for v in limited.values()) == pytest.approx(0.4)
    assert limited["H_FR"] == pytest.approx(-0.8 * (0.4 / 1.2))


def test_config_thruster_max_abs_still_bounds_live_pilot_cap(monkeypatch):
    monkeypatch.setattr("control.control_service.cfg.THRUSTER_MAX_ABS", 0.3)
    svc = _make_gain_service()
    pilot = SimpleNamespace(modes={"max_gain": 0.8})

    assert ControlService._live_thruster_max_abs(svc, pilot) == pytest.approx(0.3)
