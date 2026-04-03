import shutil
import uuid
from pathlib import Path

import numpy as np

from utils.vehicle_reference import (
    compute_level_mount,
    load_mount_reference,
    load_surface_pressure_reference_mbar,
    save_mount_reference,
    save_surface_pressure_reference,
)


def _scratch_dir() -> Path:
    p = Path("tests") / "_tmp_vehicle_reference" / str(uuid.uuid4())
    p.mkdir(parents=True, exist_ok=True)
    return p


def test_surface_pressure_reference_round_trip():
    root = _scratch_dir()
    try:
        path = root / "depth_reference.json"
        save_surface_pressure_reference(path, 1015.25, meta={"samples": 20})

        assert load_surface_pressure_reference_mbar(path) == 1015.25
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_flat_mount_reference_round_trip():
    root = _scratch_dir()
    try:
        path = root / "flat_mount.json"
        mount = compute_level_mount((0.0, 0.0, 1.0))
        save_mount_reference(path, mount, meta={"kind": "flat"})

        loaded = load_mount_reference(path)
        assert loaded is not None
        assert np.allclose(loaded.R, np.eye(3), atol=1e-6)
    finally:
        shutil.rmtree(root, ignore_errors=True)
