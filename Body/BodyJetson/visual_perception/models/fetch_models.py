#!/usr/bin/env python3
from __future__ import annotations

import sys
import urllib.request
from pathlib import Path

from huggingface_hub import hf_hub_download

from common import load_model_registry


MODELS_ROOT = Path("/home/ros/models")


def download_file(url: str, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    tmp = dst.with_suffix(dst.suffix + ".part")
    print(f"[visual_perception:fetch_models] Downloading {url}")
    urllib.request.urlretrieve(url, tmp)
    tmp.replace(dst)


def download_from_huggingface(repo_id: str, filename: str, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    print(
        f"[visual_perception:fetch_models] Downloading from Hugging Face "
        f"repo_id={repo_id} filename={filename}"
    )

    downloaded_path = hf_hub_download(
        repo_id=repo_id,
        filename=filename,
        local_dir=str(dst.parent),
        local_dir_use_symlinks=False,
    )

    downloaded_path = Path(downloaded_path)

    if downloaded_path != dst:
        downloaded_path.replace(dst)


def ensure_model_files(model_name: str, model_cfg: dict) -> None:
    model_dir = Path(model_cfg["model_dir"])

    # Legacy artifact list (current YOLO path)
    artifacts = model_cfg.get("artifacts", [])
    for artifact in artifacts:
        rel_path = artifact["path"]
        url = artifact["url"]
        dst = model_dir / rel_path
        if dst.exists():
            print(f"[visual_perception:fetch_models] Present: {dst}")
            continue
        download_file(url, dst)

    # New fetch block (OSNet path)
    fetch_cfg = model_cfg.get("fetch")
    if not fetch_cfg:
        return

    source = fetch_cfg.get("source")

    if source == "huggingface":
        repo_id = fetch_cfg["repo_id"]
        filename = fetch_cfg["filename"]
        dst = model_dir / Path(filename).name

        if dst.exists():
            print(f"[visual_perception:fetch_models] Present: {dst}")
            return

        download_from_huggingface(repo_id, filename, dst)
        return

    if source == "url":
        url = fetch_cfg["url"]
        rel_path = fetch_cfg.get("path", Path(url).name)
        dst = model_dir / rel_path
        if dst.exists():
            print(f"[visual_perception:fetch_models] Present: {dst}")
            return
        download_file(url, dst)
        return

    raise ValueError(f"Unsupported fetch source '{source}' for model '{model_name}'")


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