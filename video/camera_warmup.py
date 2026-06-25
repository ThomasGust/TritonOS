"""Boot-time camera readiness warmup for the ROV video service.

At power-on the exploreHD cameras enumerate one-by-one over a few seconds on the
shared USB2 bus. If the topside connects during that window, the first
``start_stream`` for a not-yet-present camera fails and has to recover, which is
what makes the panes pop in raggedly. This module front-loads that enumeration so
the cameras are already present by the time the topside asks for them, and gives
a slow/stuck camera a kick *at boot* (off the request path) instead of mid-herd.

Safety properties (this must never disturb live video):
  * It only *reads* the ``/dev/v4l`` node set (cheap globbing).
  * The only side effect is a **narrow, per-port** USB rebind of a configured
    camera port that has not enumerated after a grace window. That touches just
    that one camera's hub port — never a hub-level reset.
  * It runs once at service start, before any stream exists, so there is nothing
    streaming for it to interrupt.

Everything is dependency-injected (glob, sleep, clock, rebind) so the policy is
unit-testable without sysfs, real cameras, or real sleeps.
"""

from __future__ import annotations

import glob
import logging
import threading
import time
from typing import Callable, Sequence

logger = logging.getLogger("camera_warmup")

# One stable symlink per physical camera (the index0 node always exists when a
# camera is enumerated, regardless of which node we ultimately stream from).
CAMERA_NODE_GLOB = "/dev/v4l/by-path/*video-index0"


def _hint_node_glob(hint: str) -> str:
    return f"/dev/v4l/by-path/*{hint}*video-index0"


def _present_hints(hints: Sequence[str], glob_fn: Callable[[str], list]) -> set[str]:
    return {h for h in hints if glob_fn(_hint_node_glob(h))}


def wait_for_cameras_ready(
    *,
    expected_hints: Sequence[str] = (),
    timeout_s: float = 25.0,
    poll_s: float = 0.5,
    kick_missing: bool = False,
    kick_after_s: float = 6.0,
    glob_fn: Callable[[str], list] = glob.glob,
    rebind_fn: Callable[[str, list], bool] | None = None,
    sleep: Callable[[float], None] = time.sleep,
    monotonic: Callable[[], float] = time.monotonic,
) -> dict:
    """Block until every configured camera port has enumerated, or timeout.

    Returns ``{"ready", "present", "missing", "kicked", "elapsed_s"}``.

    With no ``expected_hints`` there is nothing deterministic to wait for, so we
    just snapshot what is present and return immediately.
    """

    hints = [str(h).strip() for h in expected_hints if str(h).strip()]
    start = monotonic()

    if not hints:
        present = sorted(_present_hints_from_nodes(glob_fn))
        return {
            "ready": True,
            "present": present,
            "missing": [],
            "kicked": [],
            "elapsed_s": 0.0,
        }

    deadline = start + max(0.0, float(timeout_s))
    kicked: set[str] = set()

    while True:
        present = _present_hints(hints, glob_fn)
        missing = [h for h in hints if h not in present]
        now = monotonic()
        if not missing:
            return {
                "ready": True,
                "present": sorted(present),
                "missing": [],
                "kicked": sorted(kicked),
                "elapsed_s": now - start,
            }

        # A camera still hasn't shown up after the grace window: kick its port
        # once (narrow rebind only). Nothing is streaming yet, so this is safe.
        if (
            kick_missing
            and rebind_fn is not None
            and (now - start) >= max(0.0, float(kick_after_s))
        ):
            for h in missing:
                if h in kicked:
                    continue
                kicked.add(h)
                msgs: list[str] = []
                try:
                    rebind_fn(h, msgs)
                except Exception:
                    logger.exception("camera warmup: rebind of port %s failed", h)
                for m in msgs:
                    logger.info("camera warmup: %s", m)

        if now >= deadline:
            return {
                "ready": False,
                "present": sorted(present),
                "missing": missing,
                "kicked": sorted(kicked),
                "elapsed_s": now - start,
            }
        sleep(min(max(0.0, float(poll_s)), max(0.0, deadline - now)))


def _present_hints_from_nodes(glob_fn: Callable[[str], list]) -> list[str]:
    """Best-effort: list the raw camera node paths currently present."""
    try:
        return [str(p) for p in glob_fn(CAMERA_NODE_GLOB)]
    except Exception:
        return []


def start_in_thread(
    *,
    expected_hints: Sequence[str] = (),
    timeout_s: float = 25.0,
    poll_s: float = 0.5,
    kick_missing: bool = False,
    kick_after_s: float = 6.0,
    rebind_fn: Callable[[str, list], bool] | None = None,
) -> threading.Thread:
    """Run :func:`wait_for_cameras_ready` in a daemon thread and log the result."""

    def _run() -> None:
        try:
            summary = wait_for_cameras_ready(
                expected_hints=expected_hints,
                timeout_s=timeout_s,
                poll_s=poll_s,
                kick_missing=kick_missing,
                kick_after_s=kick_after_s,
                rebind_fn=rebind_fn,
            )
        except Exception:
            logger.exception("camera warmup thread crashed")
            return
        if summary["ready"]:
            logger.info(
                "camera warmup: %d camera(s) ready in %.1fs (%s)",
                len(summary["present"]),
                summary["elapsed_s"],
                ", ".join(summary["present"]) or "none",
            )
        else:
            logger.warning(
                "camera warmup: %d camera(s) still missing after %.1fs: %s (kicked: %s)",
                len(summary["missing"]),
                summary["elapsed_s"],
                ", ".join(summary["missing"]) or "none",
                ", ".join(summary["kicked"]) or "none",
            )

    thread = threading.Thread(target=_run, name="camera-warmup", daemon=True)
    thread.start()
    return thread
