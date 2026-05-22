import shutil
import uuid
from pathlib import Path

from utils.vehicle_reference import (
    load_attitude_reference,
    load_surface_pressure_reference_mbar,
    save_attitude_reference,
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


def test_attitude_reference_round_trip():
    root = _scratch_dir()
    try:
        path = root / "attitude_reference.json"
        reference = {
            "schema": 1,
            "reference_accel": {"x": 0.0, "y": 0.0, "z": 1.0},
            "reference_norm": 9.80665,
            "gyro_bias": {"x": 0.001, "y": -0.002, "z": 0.003},
            "reference_mag": {
                "mmc5983": {"x": 1.0, "y": 0.0, "z": 0.0},
            },
            "reference_mag_norm": {"mmc5983": 42.0},
            "reference_mag_samples": {"mmc5983": 12},
        }
        save_attitude_reference(path, reference, meta={"source": "test"})

        loaded = load_attitude_reference(path)
        assert loaded is not None
        assert loaded["reference_accel"]["z"] == 1.0
        assert loaded["reference_mag_norm"]["mmc5983"] == 42.0
        assert loaded["meta"]["source"] == "test"
        assert loaded["created_ts"] > 0.0
    finally:
        shutil.rmtree(root, ignore_errors=True)
