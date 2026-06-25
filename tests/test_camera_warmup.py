"""Unit tests for the boot-time camera warmup readiness/kick policy."""

import importlib.util
from pathlib import Path

# Load camera_warmup straight from its file so the test does not import the
# `video` package __init__ (which eagerly pulls in the gi/GStreamer modules).
# camera_warmup itself is pure stdlib.
_MODULE_PATH = Path(__file__).resolve().parents[1] / "video" / "camera_warmup.py"
_spec = importlib.util.spec_from_file_location("camera_warmup_under_test", _MODULE_PATH)
camera_warmup = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(camera_warmup)


class FakeClock:
    """Monotonic clock that only advances when something sleeps."""

    def __init__(self):
        self.t = 0.0

    def monotonic(self):
        return self.t

    def sleep(self, s):
        self.t += max(0.0, float(s))


def _glob_present(present_hints):
    """glob_fn that reports a node for each hint in present_hints."""

    def _glob(pattern):
        return [pattern] if any(f"*{h}*" in pattern for h in present_hints) else []

    return _glob


def test_ready_immediately_when_all_present():
    clock = FakeClock()
    summary = camera_warmup.wait_for_cameras_ready(
        expected_hints=["1.2.1", "1.2.2"],
        glob_fn=_glob_present({"1.2.1", "1.2.2"}),
        sleep=clock.sleep,
        monotonic=clock.monotonic,
        timeout_s=5.0,
    )
    assert summary["ready"] is True
    assert summary["missing"] == []
    assert clock.t == 0.0  # no waiting needed


def test_becomes_ready_after_enumeration_delay():
    clock = FakeClock()

    def glob_fn(pattern):
        # 1.2.1 is always there; 1.2.4 only shows up once the clock passes 1.0s.
        if "*1.2.1*" in pattern:
            return [pattern]
        if "*1.2.4*" in pattern and clock.t >= 1.0:
            return [pattern]
        return []

    summary = camera_warmup.wait_for_cameras_ready(
        expected_hints=["1.2.1", "1.2.4"],
        glob_fn=glob_fn,
        sleep=clock.sleep,
        monotonic=clock.monotonic,
        poll_s=0.5,
        timeout_s=5.0,
        kick_missing=False,
    )
    assert summary["ready"] is True
    assert summary["elapsed_s"] >= 1.0


def test_kicks_missing_port_after_grace_then_times_out():
    clock = FakeClock()
    kicks = []
    summary = camera_warmup.wait_for_cameras_ready(
        expected_hints=["1.2.1", "1.2.9"],
        glob_fn=_glob_present({"1.2.1"}),  # 1.2.9 never appears
        rebind_fn=lambda hint, msgs: kicks.append(hint),
        kick_missing=True,
        kick_after_s=2.0,
        poll_s=0.5,
        timeout_s=4.0,
        sleep=clock.sleep,
        monotonic=clock.monotonic,
    )
    assert summary["ready"] is False
    assert summary["missing"] == ["1.2.9"]
    assert kicks == ["1.2.9"]  # kicked exactly once, only the missing port
    assert summary["kicked"] == ["1.2.9"]


def test_no_kick_before_grace_window():
    clock = FakeClock()
    kicks = []
    camera_warmup.wait_for_cameras_ready(
        expected_hints=["1.2.9"],
        glob_fn=_glob_present(set()),
        rebind_fn=lambda hint, msgs: kicks.append(hint),
        kick_missing=True,
        kick_after_s=10.0,  # longer than the timeout below
        poll_s=0.5,
        timeout_s=2.0,
        sleep=clock.sleep,
        monotonic=clock.monotonic,
    )
    assert kicks == []  # timed out before the grace window elapsed


def test_no_hints_returns_ready_without_waiting():
    clock = FakeClock()
    summary = camera_warmup.wait_for_cameras_ready(
        expected_hints=[],
        glob_fn=lambda pattern: ["/dev/v4l/by-path/a-video-index0"],
        sleep=clock.sleep,
        monotonic=clock.monotonic,
    )
    assert summary["ready"] is True
    assert clock.t == 0.0
