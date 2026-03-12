from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

import yaml


MODELS_ROOT = Path("/home/ros/models")


def find_model_configs(models_root: Path = MODELS_ROOT) -> list[Path]:
    if not models_root.exists():
        return []
    return sorted(
        p for p in models_root.glob("*/model.yaml")
        if p.is_file()
    )


def load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_model_registry(models_root: Path = MODELS_ROOT) -> Dict[str, Dict[str, Any]]:
    registry: Dict[str, Dict[str, Any]] = {}
    for config_path in find_model_configs(models_root):
        data = load_yaml(config_path)
        name = data.get("name") or config_path.parent.name
        data["name"] = name
        data["model_dir"] = str(config_path.parent)
        data["config_path"] = str(config_path)
        registry[name] = data
    return registry


def model_dir(model_cfg: Dict[str, Any]) -> Path:
    return Path(model_cfg["model_dir"])


def artifact_path(model_cfg: Dict[str, Any], relative_path: str) -> Path:
    return model_dir(model_cfg) / relative_path


def required_files_present(model_cfg: Dict[str, Any]) -> bool:
    for artifact in model_cfg.get("artifacts", []):
        if not artifact_path(model_cfg, artifact["path"]).exists():
            return False
    return True


def engine_path(model_cfg: Dict[str, Any]) -> Path | None:
    build_cfg = model_cfg.get("build", {})
    engine_rel = build_cfg.get("engine")
    if not engine_rel:
        return None
    return artifact_path(model_cfg, engine_rel)