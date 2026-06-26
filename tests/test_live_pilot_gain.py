from types import SimpleNamespace

import pytest

from control.control_service import ControlGains, ControlService


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
