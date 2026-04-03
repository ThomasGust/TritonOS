import shutil
import uuid
from pathlib import Path

from utils.config_store import update_config_values


def _scratch_dir() -> Path:
    p = Path("tests") / "_tmp_config_store" / str(uuid.uuid4())
    p.mkdir(parents=True, exist_ok=True)
    return p


def test_update_config_values_rewrites_python_literals():
    root = _scratch_dir()
    try:
        path = root / "sample_config.py"
        path.write_text(
            'FOO = 1\nBAR = {"a": 1}\nBAZ = (1, 2)\n',
            encoding="utf-8",
        )

        update_config_values(
            {
                "FOO": 2,
                "BAR": {"enabled": False, "name": "depth"},
                "BAZ": (3, 4, 5),
            },
            path=path,
        )

        updated = path.read_text(encoding="utf-8")
        assert "FOO = 2" in updated
        assert "False" in updated
        assert "'depth'" in updated
        assert "BAZ = (3, 4, 5)" in updated
    finally:
        shutil.rmtree(root, ignore_errors=True)
