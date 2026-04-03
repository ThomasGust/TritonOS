from __future__ import annotations

import ast
import importlib
import pprint
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, Optional


DEFAULT_CONFIG_PATH = "rov_config.py"


def resolve_config_path(path: str | Path = DEFAULT_CONFIG_PATH) -> Path:
    return Path(path).expanduser()


def _is_editable_name(name: str) -> bool:
    return bool(name) and name.isupper()


def _serialize_value(value: Any) -> Any:
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, tuple):
        return [_serialize_value(v) for v in value]
    if isinstance(value, list):
        return [_serialize_value(v) for v in value]
    if isinstance(value, dict):
        out: Dict[str, Any] = {}
        for k, v in value.items():
            out[str(k)] = _serialize_value(v)
        return out
    return str(value)


def load_runtime_config_snapshot() -> Dict[str, Any]:
    cfg = importlib.import_module("rov_config")
    snapshot: Dict[str, Any] = {}
    for name in dir(cfg):
        if not _is_editable_name(name):
            continue
        try:
            snapshot[name] = _serialize_value(getattr(cfg, name))
        except Exception:
            continue
    return snapshot


def reload_runtime_config_module() -> Any:
    if "rov_config" in sys.modules:
        return importlib.reload(sys.modules["rov_config"])
    return importlib.import_module("rov_config")


def _load_module_source(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _assignment_map(source: str) -> Dict[str, ast.AST]:
    tree = ast.parse(source)
    out: Dict[str, ast.AST] = {}
    for node in tree.body:
        target_name: Optional[str] = None
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            target_name = node.targets[0].id
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            target_name = node.target.id

        if target_name and _is_editable_name(target_name):
            out[target_name] = node
    return out


def _to_python_literal(value: Any) -> str:
    return pprint.pformat(value, sort_dicts=False, width=100)


def update_config_values(updates: Dict[str, Any], *, path: str | Path = DEFAULT_CONFIG_PATH) -> Dict[str, Any]:
    if not updates:
        return {}

    cfg_path = resolve_config_path(path)
    source = _load_module_source(cfg_path)
    line_ending = "\r\n" if "\r\n" in source else "\n"
    lines = source.splitlines(keepends=True)
    assignments = _assignment_map(source)

    missing = [k for k in updates.keys() if k not in assignments]
    if missing:
        raise KeyError(f"Unknown or unsupported config keys: {missing}")

    replacements: Dict[str, tuple[int, int, str]] = {}
    for key, value in updates.items():
        node = assignments[key]
        start = int(getattr(node, "lineno")) - 1
        end = int(getattr(node, "end_lineno")) - 1
        indent = ""
        if 0 <= start < len(lines):
            indent = lines[start][: len(lines[start]) - len(lines[start].lstrip(" "))]
        if isinstance(node, ast.Assign):
            rendered = f"{indent}{key} = {_to_python_literal(value)}{line_ending}"
        elif isinstance(node, ast.AnnAssign):
            annotation = ast.get_source_segment(source, node.annotation) or "Any"
            rendered = f"{indent}{key}: {annotation} = {_to_python_literal(value)}{line_ending}"
        else:
            raise KeyError(f"Unsupported config assignment for {key}")
        replacements[key] = (start, end, rendered)

    ordered = sorted(replacements.values(), key=lambda x: x[0], reverse=True)
    for start, end, rendered in ordered:
        lines[start : end + 1] = [rendered]

    cfg_path.write_text("".join(lines), encoding="utf-8")
    return dict(updates)
