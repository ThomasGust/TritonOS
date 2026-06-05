"""Best-effort V4L2 camera encoder control helpers.

Native UVC H.264 cameras often expose bitrate/GOP controls through V4L2 rather
than through the GStreamer caps that carry the encoded stream. These helpers
keep those controls optional: if a camera or host lacks a control, streaming
continues with the camera defaults.
"""

from __future__ import annotations

import re
import subprocess
from shutil import which
from typing import Any, Iterable


CONTROL_NAME_RE = re.compile(r"^\s*([A-Za-z][A-Za-z0-9_]*)\s+0x[0-9a-fA-F]+")

BITRATE_CONTROL_NAMES = (
    "video_bitrate",
    "h264_video_bitrate",
)

GOP_CONTROL_NAMES = (
    "h264_i_frame_period",
    "h264_gop_size",
)


def parse_control_names(text: str | None) -> set[str]:
    """Extract V4L2 control names from ``v4l2-ctl --list-ctrls`` output."""

    names: set[str] = set()
    for line in (text or "").splitlines():
        match = CONTROL_NAME_RE.match(line)
        if match:
            names.add(match.group(1))
    return names


def _int_or_none(value: Any) -> int | None:
    try:
        out = int(value)
    except Exception:
        return None
    return out if out > 0 else None


def _first_available(candidates: Iterable[str], available: set[str]) -> str | None:
    for name in candidates:
        if str(name) in available:
            return str(name)
    return None


def build_h264_quality_controls(
    available_controls: set[str],
    *,
    h264_bitrate: int,
    h264_gop: int,
    extra: dict[str, Any] | None = None,
) -> dict[str, int]:
    """Return native H.264 V4L2 control updates supported by this camera."""

    extra = dict(extra or {})
    if str(extra.get("apply_h264_v4l2_controls", "1")).strip().lower() in {"0", "false", "no", "off"}:
        return {}

    updates: dict[str, int] = {}
    bitrate = _int_or_none(extra.get("v4l2_h264_bitrate", h264_bitrate))
    if bitrate is not None:
        candidates = extra.get("h264_bitrate_control_names", BITRATE_CONTROL_NAMES)
        if isinstance(candidates, str):
            candidates = [part.strip() for part in candidates.split(",") if part.strip()]
        name = _first_available(candidates, available_controls)
        if name:
            updates[name] = int(bitrate)

    gop = _int_or_none(extra.get("v4l2_h264_gop", h264_gop))
    if gop is not None:
        candidates = extra.get("h264_gop_control_names", GOP_CONTROL_NAMES)
        if isinstance(candidates, str):
            candidates = [part.strip() for part in candidates.split(",") if part.strip()]
        name = _first_available(candidates, available_controls)
        if name:
            updates[name] = int(gop)

    explicit = extra.get("v4l2_controls")
    if isinstance(explicit, dict):
        for key, value in explicit.items():
            name = str(key).strip()
            int_value = _int_or_none(value)
            if name and int_value is not None and name in available_controls:
                updates[name] = int(int_value)

    return updates


def set_ctrl_arg(updates: dict[str, int]) -> str:
    """Format a ``--set-ctrl=...`` argument body."""

    return ",".join(f"{name}={int(value)}" for name, value in sorted(updates.items()))


def list_control_names(device: str) -> set[str]:
    """Return available V4L2 controls for ``device`` or an empty set."""

    if which("v4l2-ctl") is None:
        return set()
    try:
        out = subprocess.check_output(
            ["v4l2-ctl", "-d", str(device), "--list-ctrls"],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=2.0,
        )
    except Exception:
        return set()
    return parse_control_names(out)


def apply_h264_quality_controls(
    device: str,
    *,
    h264_bitrate: int,
    h264_gop: int,
    extra: dict[str, Any] | None = None,
    logger: Any | None = None,
) -> dict[str, int]:
    """Apply supported native H.264 camera controls and return attempted values."""

    controls = list_control_names(device)
    updates = build_h264_quality_controls(
        controls,
        h264_bitrate=h264_bitrate,
        h264_gop=h264_gop,
        extra=extra,
    )
    if not updates:
        return {}
    if which("v4l2-ctl") is None:
        return {}

    arg = set_ctrl_arg(updates)
    try:
        subprocess.check_call(
            ["v4l2-ctl", "-d", str(device), f"--set-ctrl={arg}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=2.0,
        )
        if logger is not None:
            logger.info("Applied V4L2 H.264 controls for %s: %s", device, arg)
    except Exception as exc:
        if logger is not None:
            logger.warning("Could not apply V4L2 H.264 controls for %s (%s): %s", device, arg, exc)
        return {}
    return updates
