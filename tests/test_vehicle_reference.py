import shutil
import uuid
from pathlib import Path

from utils.vehicle_reference import (
    load_surface_pressure_reference_mbar,
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
