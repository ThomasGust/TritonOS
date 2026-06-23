"""Tests for the feed-forward current model and the optional budget limiter.

These cover three things:
  1. The T200 model reproduces the BlueRobotics data at known points.
  2. The summed-current budget scaler is correct and monotonic.
  3. ControlService._apply_current_budget is a true no-op when disabled and
     fails open when the model misbehaves (the safety-critical guarantees).
"""

import pytest

from control.current_model import T200CurrentModel, current_budget_scale
from control.control_service import ControlService


@pytest.fixture(scope="module")
def model() -> T200CurrentModel:
    return T200CurrentModel.bundled()


# ---- model ------------------------------------------------------------------

def test_neutral_draws_near_zero(model):
    assert model.current_for_norm(0.0, 12.0) == pytest.approx(0.0, abs=0.2)
    assert model.current_for_norm(0.0, 14.0) == pytest.approx(0.0, abs=0.2)


def test_full_throttle_matches_datasheet(model):
    # From the BlueRobotics sheet: ~17 A at 12 V, ~20.5 A at 14 V (full forward).
    assert model.current_for_norm(1.0, 12.0) == pytest.approx(16.9, abs=0.6)
    assert model.current_for_norm(1.0, 14.0) == pytest.approx(20.4, abs=0.6)
    # Reverse is a positive magnitude and similar order (slightly higher here).
    assert model.current_for_norm(-1.0, 14.0) == pytest.approx(20.7, abs=0.8)


def test_current_monotonic_in_magnitude(model):
    prev = -1.0
    for i in range(0, 11):
        n = i / 10.0
        cur = model.current_for_norm(n, 14.0)
        assert cur >= prev - 1e-6
        prev = cur


def test_voltage_interpolation_is_between_curves(model):
    c12 = model.current_for_norm(1.0, 12.0)
    c14 = model.current_for_norm(1.0, 14.0)
    c13 = model.current_for_norm(1.0, 13.0)
    assert c12 < c13 < c14


def test_voltage_clamped_outside_range(model):
    # Below 10 V / above 20 V clamp to the end curves rather than extrapolating.
    assert model.current_for_norm(1.0, 5.0) == pytest.approx(model.current_for_norm(1.0, 10.0))
    assert model.current_for_norm(1.0, 99.0) == pytest.approx(model.current_for_norm(1.0, 20.0))


# ---- budget scaler ----------------------------------------------------------

def test_scale_is_one_when_under_budget(model):
    norms = {"H_FL": 0.2, "H_FR": 0.2}
    scale, before, after = current_budget_scale(norms, model, voltage=14.0, budget_a=20.0)
    assert scale == 1.0
    assert before == pytest.approx(after)
    assert before < 20.0


def test_scale_brings_total_to_budget(model):
    # Two thrusters at full at 14 V ~= 40 A; budget 20 A must roughly halve draw.
    norms = {"H_FL": 1.0, "H_FR": 1.0}
    scale, before, after = current_budget_scale(norms, model, voltage=14.0, budget_a=20.0)
    assert 0.0 < scale < 1.0
    assert before > 35.0
    assert after == pytest.approx(20.0, abs=0.5)


def test_scale_preserves_direction_and_ratio(model):
    norms = {"H_FL": 1.0, "H_FR": -0.5, "V_FL": 0.25}
    scale, _, _ = current_budget_scale(norms, model, voltage=14.0, budget_a=15.0)
    # A single shared factor preserves sign and relative magnitudes.
    assert 0.0 < scale < 1.0


def test_min_scale_floor_is_respected(model):
    norms = {"H_FL": 1.0, "H_FR": 1.0, "V_FL": 1.0, "V_FR": 1.0}
    scale, _, _ = current_budget_scale(
        norms, model, voltage=14.0, budget_a=1.0, min_scale=0.3
    )
    assert scale == pytest.approx(0.3)


# ---- ControlService integration: safety guarantees --------------------------

def _svc_with_budget(enabled, model=None, **over):
    svc = object.__new__(ControlService)
    svc._current_budget_enabled = bool(enabled)
    svc._current_model = model
    svc._current_budget_max_a = over.get("max_a", 22.0)
    svc._current_budget_reserve_a = over.get("reserve_a", 2.0)
    svc._current_budget_voltage_v = over.get("voltage_v", 14.0)
    svc._current_budget_min_scale = over.get("min_scale", 0.0)
    svc._current_budget_warned = False
    return svc


def test_disabled_is_exact_no_op():
    svc = _svc_with_budget(enabled=False)
    thr = {"H_FL": 0.9, "H_FR": -0.8, "V_FL": 0.5}
    out, diag = ControlService._apply_current_budget(svc, thr)
    assert out is thr  # same object, untouched
    assert diag == {"enabled": False}


def test_enabled_scales_when_over_budget(model):
    svc = _svc_with_budget(enabled=True, model=model, max_a=20.0, reserve_a=0.0)
    thr = {"H_FL": 1.0, "H_FR": 1.0}
    out, diag = ControlService._apply_current_budget(svc, thr)
    assert diag["enabled"] and diag["applied"]
    assert 0.0 < diag["scale"] < 1.0
    assert out["H_FL"] == pytest.approx(thr["H_FL"] * diag["scale"])
    assert diag["predicted_after_a"] == pytest.approx(20.0, abs=0.5)


def test_inactive_predicts_but_does_not_apply(model):
    # Live toggle off: the estimate is still produced (for the topside readout)
    # but thrust is NOT scaled, even when well over budget.
    svc = _svc_with_budget(enabled=True, model=model, max_a=20.0, reserve_a=0.0)
    thr = {"H_FL": 1.0, "H_FR": 1.0}
    out, diag = ControlService._apply_current_budget(svc, thr, active=False)
    assert out == thr  # unchanged
    assert diag["enabled"] and diag["active"] is False and diag["applied"] is False
    assert diag["scale"] == 1.0
    assert diag["predicted_before_a"] > 35.0  # estimate still available


def test_enabled_under_budget_passes_through(model):
    svc = _svc_with_budget(enabled=True, model=model, max_a=50.0, reserve_a=0.0)
    thr = {"H_FL": 0.3, "V_FL": 0.2}
    out, diag = ControlService._apply_current_budget(svc, thr)
    assert out is thr
    assert diag["applied"] is False
    assert diag["scale"] == 1.0


def test_live_max_a_override_changes_budget(model):
    # Same command, two different live caps -> the lower cap scales harder.
    svc = _svc_with_budget(enabled=True, model=model, max_a=22.0, reserve_a=0.0)
    thr = {"H_FL": 1.0, "H_FR": 1.0}

    _, diag_hi = ControlService._apply_current_budget(svc, dict(thr), max_a_override=24.0)
    _, diag_lo = ControlService._apply_current_budget(svc, dict(thr), max_a_override=14.0)

    assert diag_hi["budget_a"] == pytest.approx(24.0)
    assert diag_lo["budget_a"] == pytest.approx(14.0)
    assert diag_lo["scale"] < diag_hi["scale"]
    assert diag_lo["predicted_after_a"] == pytest.approx(14.0, abs=0.5)


def test_bad_max_a_override_falls_back_to_config(model):
    svc = _svc_with_budget(enabled=True, model=model, max_a=20.0, reserve_a=0.0)
    thr = {"H_FL": 1.0, "H_FR": 1.0}
    # Non-numeric / non-positive overrides are ignored (config cap is used).
    for bad in (None, "nope", float("nan"), -5.0, 0.0):
        _, diag = ControlService._apply_current_budget(svc, dict(thr), max_a_override=bad)
        assert diag["budget_a"] == pytest.approx(20.0)


def test_non_thruster_keys_untouched(model):
    svc = _svc_with_budget(enabled=True, model=model, max_a=20.0, reserve_a=0.0)
    thr = {"H_FL": 1.0, "H_FR": 1.0, "lights": 0.7}
    out, _ = ControlService._apply_current_budget(svc, thr)
    assert out["lights"] == 0.7  # aux output never scaled


def test_fails_open_on_broken_model():
    class Boom:
        def current_for_norm(self, *_a, **_k):
            raise RuntimeError("sensor on fire")

    svc = _svc_with_budget(enabled=True, model=Boom(), max_a=20.0, reserve_a=0.0)
    thr = {"H_FL": 1.0, "H_FR": 1.0}
    out, diag = ControlService._apply_current_budget(svc, thr)
    assert out == thr  # unchanged on error
    assert diag["applied"] is False
    assert "error" in diag
