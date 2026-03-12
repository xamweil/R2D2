#!/usr/bin/env python3
from __future__ import annotations

import sys
import urllib.request
from pathlib import Path

from common import load_model_registry


def download_file(url: str, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    tmp = dst.with_suffix(dst.suffix + ".part")
    print(f"[visual_perception:fetch_models] Downloading {url}")
    urllib.request.urlretrieve(url, tmp)
    tmp.replace(dst)


def ensure_model_files(model_name: str, model_cfg: dict) -> None:
    model_dir = Path(model_cfg["model_dir"])
    artifacts = model_cfg.get("artifacts", [])
    if not artifacts:
        print(f"[visual_perception:fetch_models] WARNING: no artifacts defined for {model_name}")
        return

    for artifact in artifacts:
        rel_path = artifact["path"]
        url = artifact["url"]
        dst = model_dir / rel_path
        if dst.exists():
            print(f"[visual_perception:fetch_models] Present: {dst}")
            continue
        download_file(url, dst)


def main() -> int:
    registry = load_model_registry()
    if not registry:
        print("[visual_perception:fetch_models] No model.yaml files found under /home/ros/models")
        return 0

    print(f"[visual_perception:fetch_models] Found models: {', '.join(sorted(registry.keys()))}")
    for model_name, model_cfg in registry.items():
        ensure_model_files(model_name, model_cfg)

    print("[visual_perception:fetch_models] Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())