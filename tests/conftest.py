import os
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

PYTEST_TEMP_ROOT = ROOT / ".pytest-work"
PYTEST_TEMP_ROOT.mkdir(exist_ok=True)
os.environ.setdefault("PYTEST_DEBUG_TEMPROOT", str(PYTEST_TEMP_ROOT))


_TRUTHY_ENV = {"1", "true", "yes", "on"}
_OPTIONAL_MARKERS = {
    "network": (
        "run_network",
        "TRITON_RUN_NETWORK",
        "opens sockets or depends on an active local/network stack",
    ),
    "groundtruth": (
        "run_groundtruth",
        "TRITON_RUN_GROUNDTRUTH",
        "depends on optional saved ground-truth media/data",
    ),
    "slow": (
        "run_slow",
        "TRITON_RUN_SLOW",
        "is intentionally slower than the quick trust check",
    ),
    "hardware": (
        "run_hardware",
        "TRITON_RUN_HARDWARE",
        "requires physical ROV hardware or live services",
    ),
}


def _env_enabled(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in _TRUTHY_ENV


def pytest_addoption(parser):
    """Register Triton trust-harness switches for optional test tiers."""
    group = parser.getgroup("triton trust harness")
    group.addoption(
        "--run-network",
        action="store_true",
        default=False,
        help="Run tests marked network.",
    )
    group.addoption(
        "--run-groundtruth",
        action="store_true",
        default=False,
        help="Run tests marked groundtruth.",
    )
    group.addoption(
        "--run-slow",
        action="store_true",
        default=False,
        help="Run tests marked slow.",
    )
    group.addoption(
        "--run-hardware",
        action="store_true",
        default=False,
        help="Run tests marked hardware.",
    )
    group.addoption(
        "--run-extended",
        action="store_true",
        default=False,
        help="Run all optional non-hardware tiers.",
    )
    group.addoption(
        "--run-all-trust",
        action="store_true",
        default=False,
        help="Run every test tier, including hardware.",
    )


def _marker_enabled(config, marker: str) -> bool:
    option, env_name, _description = _OPTIONAL_MARKERS[marker]
    if config.getoption("run_all_trust"):
        return True
    if marker != "hardware" and config.getoption("run_extended"):
        return True
    return bool(config.getoption(option) or _env_enabled(env_name))


def pytest_collection_modifyitems(config, items):
    """Keep the default suite fast and deterministic by skipping opt-in tiers."""
    for marker, (option, env_name, description) in _OPTIONAL_MARKERS.items():
        if _marker_enabled(config, marker):
            continue
        skip_marker = pytest.mark.skip(
            reason=(
                f"{marker} test skipped by default because it {description}; "
                f"pass --{option.replace('_', '-')} or set {env_name}=1"
            )
        )
        for item in items:
            if item.get_closest_marker(marker):
                item.add_marker(skip_marker)
