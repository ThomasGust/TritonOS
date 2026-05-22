"""Run TritonOS test tiers with the standard trust-harness switches."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def _pytest_command(mode: str) -> list[str]:
    base = [sys.executable, "-m", "pytest"]
    commands = {
        "quick": base,
        "network": [*base, "--run-network", "-m", "network"],
        "groundtruth": [*base, "--run-groundtruth", "-m", "groundtruth"],
        "extended": [*base, "--run-extended"],
        "hardware": [*base, "--run-hardware", "-m", "hardware"],
        "full": [*base, "--run-all-trust"],
        "collect": [*base, "--collect-only"],
        "coverage": [
            *base,
            "--cov=.",
            "--cov-report=term-missing",
            "--cov-report=html",
        ],
    }
    return commands[mode]


def _coverage_available() -> bool:
    try:
        import pytest_cov  # noqa: F401
    except ImportError:
        return False
    return True


def build_parser() -> argparse.ArgumentParser:
    """Build the trust-check command parser."""
    parser = argparse.ArgumentParser(
        description="Run TritonOS's quick or opt-in test tiers.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "mode",
        nargs="?",
        default="quick",
        choices=(
            "quick",
            "network",
            "groundtruth",
            "extended",
            "hardware",
            "full",
            "collect",
            "coverage",
        ),
        help="Test tier to run.",
    )
    parser.add_argument(
        "pytest_args",
        nargs=argparse.REMAINDER,
        help="Extra pytest arguments. Prefix with -- to separate them from this command.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the selected test tier and return pytest's exit code."""
    args = build_parser().parse_args(argv)
    if args.mode == "coverage" and not _coverage_available():
        print("coverage mode requires pytest-cov. Install it with: python -m pip install pytest-cov")
        return 2

    extra_args = list(args.pytest_args)
    if extra_args and extra_args[0] == "--":
        extra_args = extra_args[1:]

    cmd = [*_pytest_command(args.mode), *extra_args]
    print("Running:", " ".join(cmd), flush=True)
    return subprocess.call(cmd, cwd=REPO_ROOT)


if __name__ == "__main__":
    raise SystemExit(main())
