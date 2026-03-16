#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

from common import artifact_path, engine_path, load_model_registry, required_files_present


TRTEXEC_CANDIDATES = [
    "trtexec",
    "/usr/src/tensorrt/bin/trtexec",
    "/usr/local/bin/trtexec",
]


def find_trtexec() -> str | None:
    for candidate in TRTEXEC_CANDIDATES:
        resolved = shutil.which(candidate)
        if resolved:
            return resolved
        if Path(candidate).exists():
            return candidate
    return None


def check_model(model_name: str, model_cfg: dict) -> int:
    print(f"[visual_perception:prepare_models] Model: {model_name}")
    ok = True

    # Legacy artifacts list
    for artifact in model_cfg.get("artifacts", []):
        path = artifact_path(model_cfg, artifact["path"])
        exists = path.exists()
        print(f"  artifact: {path} -> {'OK' if exists else 'MISSING'}")
        ok &= exists

    # New fetch block
    fetch_cfg = model_cfg.get("fetch")
    if fetch_cfg:
        filename = fetch_cfg.get("filename")
        if filename:
            path = artifact_path(model_cfg, Path(filename).name)
            exists = path.exists()
            print(f"  fetch artifact: {path} -> {'OK' if exists else 'MISSING'}")
            ok &= exists

    build_cfg = model_cfg.get("build", {})
    source_onnx = build_cfg.get("source_onnx")
    if source_onnx:
        onnx_path = artifact_path(model_cfg, source_onnx)
        print(f"  onnx:     {onnx_path} -> {'OK' if onnx_path.exists() else 'MISSING'}")

    eng_path = engine_path(model_cfg)
    if eng_path is not None:
        print(f"  engine:   {eng_path} -> {'OK' if eng_path.exists() else 'MISSING'}")

    builder = build_cfg.get("builder")
    if builder in ("trtexec", "torchreid_onnx_trtexec"):
        trtexec = find_trtexec()
        print(f"  builder:  trtexec -> {'OK: ' + trtexec if trtexec else 'MISSING'}")
        ok &= trtexec is not None

    if builder == "torchreid_onnx_trtexec":
        export_script = artifact_path(model_cfg, "export_to_onnx.py")
        print(f"  exporter: {export_script} -> {'OK' if export_script.exists() else 'MISSING'}")
        ok &= export_script.exists()

    return 0 if ok else 1


def run_cmd(cmd: list[str]) -> int:
    print(f"[visual_perception:prepare_models] Command: {' '.join(cmd)}")
    result = subprocess.run(cmd, check=False)
    return result.returncode


def build_with_trtexec(model_name: str, model_cfg: dict, force: bool) -> int:
    if not required_files_present(model_cfg):
        print(f"[visual_perception:prepare_models] Missing raw artifacts for {model_name}", file=sys.stderr)
        return 1

    trtexec = find_trtexec()
    if not trtexec:
        print("[visual_perception:prepare_models] trtexec not found", file=sys.stderr)
        return 1

    build_cfg = model_cfg.get("build", {})
    src_rel = build_cfg.get("source")
    engine_rel = build_cfg.get("engine")
    precision = build_cfg.get("precision", "fp16")
    extra_args = build_cfg.get("extra_args", [])

    if not all([src_rel, engine_rel]):
        print(f"[visual_perception:prepare_models] Incomplete build config for {model_name}", file=sys.stderr)
        return 1

    src_path = artifact_path(model_cfg, src_rel)
    eng_path = artifact_path(model_cfg, engine_rel)

    if eng_path.exists() and not force:
        print(f"[visual_perception:prepare_models] Engine already exists: {eng_path}")
        return 0

    eng_path.parent.mkdir(parents=True, exist_ok=True)

    cmd = [
        trtexec,
        f"--onnx={src_path}",
        f"--saveEngine={eng_path}",
        "--skipInference",
    ]

    if precision == "fp16":
        cmd.append("--fp16")
    elif precision == "int8":
        cmd.append("--int8")

    cmd.extend(extra_args)

    print(f"[visual_perception:prepare_models] Building engine for {model_name}")
    rc = run_cmd(cmd)
    if rc != 0:
        print(f"[visual_perception:prepare_models] Engine build failed for {model_name}", file=sys.stderr)
        return rc

    if not eng_path.exists():
        print(f"[visual_perception:prepare_models] Build command succeeded but engine missing: {eng_path}", file=sys.stderr)
        return 1

    print(f"[visual_perception:prepare_models] Engine created: {eng_path}")
    return 0


def export_torchreid_to_onnx(model_name: str, model_cfg: dict, force: bool) -> int:
    build_cfg = model_cfg.get("build", {})

    checkpoint_rel = build_cfg.get("source_pth")
    onnx_rel = build_cfg.get("source_onnx")
    input_size = build_cfg.get("input_size", [256, 128])

    if not checkpoint_rel or not onnx_rel:
        print(f"[visual_perception:prepare_models] Missing source_pth/source_onnx for {model_name}", file=sys.stderr)
        return 1

    checkpoint_path = artifact_path(model_cfg, checkpoint_rel)
    onnx_path = artifact_path(model_cfg, onnx_rel)
    export_script = artifact_path(model_cfg, "export_to_onnx.py")

    if not checkpoint_path.exists():
        print(f"[visual_perception:prepare_models] Checkpoint missing: {checkpoint_path}", file=sys.stderr)
        return 1

    if not export_script.exists():
        print(f"[visual_perception:prepare_models] Export script missing: {export_script}", file=sys.stderr)
        return 1

    if onnx_path.exists() and not force:
        print(f"[visual_perception:prepare_models] ONNX already exists: {onnx_path}")
        return 0

    height, width = input_size

    cmd = [
        "python3",
        str(export_script),
        "--checkpoint",
        str(checkpoint_path),
        "--onnx",
        str(onnx_path),
        "--height",
        str(height),
        "--width",
        str(width),
    ]

    print(f"[visual_perception:prepare_models] Exporting ONNX for {model_name}")
    rc = run_cmd(cmd)
    if rc != 0:
        print(f"[visual_perception:prepare_models] ONNX export failed for {model_name}", file=sys.stderr)
        return rc

    if not onnx_path.exists():
        print(f"[visual_perception:prepare_models] Export command succeeded but ONNX missing: {onnx_path}", file=sys.stderr)
        return 1

    print(f"[visual_perception:prepare_models] ONNX created: {onnx_path}")
    return 0


def build_torchreid_onnx_trtexec(model_name: str, model_cfg: dict, force: bool) -> int:
    build_cfg = model_cfg.get("build", {})
    onnx_rel = build_cfg.get("source_onnx")
    engine_rel = build_cfg.get("engine")
    precision = build_cfg.get("precision", "fp16")

    if not onnx_rel or not engine_rel:
        print(f"[visual_perception:prepare_models] Missing source_onnx/engine for {model_name}", file=sys.stderr)
        return 1

    eng_path = artifact_path(model_cfg, engine_rel)
    if eng_path.exists() and not force:
        print(f"[visual_perception:prepare_models] Engine already exists: {eng_path}")
        return 0

    rc = export_torchreid_to_onnx(model_name, model_cfg, force=force)
    if rc != 0:
        return rc

    trtexec = find_trtexec()
    if not trtexec:
        print("[visual_perception:prepare_models] trtexec not found", file=sys.stderr)
        return 1

    onnx_path = artifact_path(model_cfg, onnx_rel)

    cmd = [
        trtexec,
        f"--onnx={onnx_path}",
        f"--saveEngine={eng_path}",
        "--skipInference",
    ]

    if precision == "fp16":
        cmd.append("--fp16")
    elif precision == "int8":
        cmd.append("--int8")

    print(f"[visual_perception:prepare_models] Building engine for {model_name}")
    rc = run_cmd(cmd)
    if rc != 0:
        print(f"[visual_perception:prepare_models] Engine build failed for {model_name}", file=sys.stderr)
        return rc

    if not eng_path.exists():
        print(f"[visual_perception:prepare_models] Build command succeeded but engine missing: {eng_path}", file=sys.stderr)
        return 1

    print(f"[visual_perception:prepare_models] Engine created: {eng_path}")
    return 0


def build_model(model_name: str, model_cfg: dict, force: bool) -> int:
    builder = model_cfg.get("build", {}).get("builder")

    if builder == "trtexec":
        return build_with_trtexec(model_name, model_cfg, force)

    if builder == "torchreid_onnx_trtexec":
        return build_torchreid_onnx_trtexec(model_name, model_cfg, force)

    print(f"[visual_perception:prepare_models] Unsupported builder '{builder}' for {model_name}", file=sys.stderr)
    return 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--list", action="store_true", help="List discovered models")
    parser.add_argument("--check", action="store_true", help="Check model artifacts and tools")
    parser.add_argument("--build", metavar="MODEL", help="Build engine for one model or 'all'")
    parser.add_argument("--force", action="store_true", help="Rebuild engine even if it already exists")
    args = parser.parse_args()

    registry = load_model_registry()
    if not registry:
        print("[visual_perception:prepare_models] No model.yaml files found under /home/ros/models", file=sys.stderr)
        return 1

    if args.list:
        for model_name in sorted(registry.keys()):
            print(model_name)
        return 0

    if args.check:
        rc = 0
        for model_name, model_cfg in sorted(registry.items()):
            rc |= check_model(model_name, model_cfg)
        return rc

    if args.build:
        if args.build == "all":
            rc = 0
            for model_name, model_cfg in sorted(registry.items()):
                rc |= build_model(model_name, model_cfg, args.force)
            return rc

        if args.build not in registry:
            print(f"[visual_perception:prepare_models] Unknown model: {args.build}", file=sys.stderr)
            return 1

        return build_model(args.build, registry[args.build], args.force)

    parser.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())