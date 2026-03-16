#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import torch


def build_osnet_model():
    """
    Build OSNet x0.25 using torchreid.
    Supports different torchreid package layouts.
    """
    try:
        from torchreid.reid.models import build_model
    except ImportError:
        try:
            from torchreid import models
            build_model = models.build_model
        except Exception as exc:
            raise RuntimeError(
                "torchreid is installed, but build_model could not be imported "
                "from the available package layout."
            ) from exc

    model = build_model(
        name="osnet_x0_25",
        num_classes=1,
        pretrained=False,
        use_gpu=False,
    )
    return model


def load_checkpoint_weights(model: torch.nn.Module, checkpoint_path: Path) -> None:
    """
    Load torch checkpoint into OSNet model.

    We accept either:
    - raw state_dict
    - dict with 'state_dict'
    """
    ckpt = torch.load(str(checkpoint_path), map_location="cpu")

    if isinstance(ckpt, dict) and "state_dict" in ckpt:
        state_dict = ckpt["state_dict"]
    else:
        state_dict = ckpt

    # Remove possible 'module.' prefixes from DataParallel checkpoints
        cleaned_state_dict = {}
    for k, v in state_dict.items():
        # Remove possible 'module.' prefixes from DataParallel checkpoints
        if k.startswith("module."):
            k = k[len("module."):]

        # Drop classifier head weights: for ReID export we only need embeddings
        if k in ("classifier.weight", "classifier.bias"):
            continue

        cleaned_state_dict[k] = v

    missing, unexpected = model.load_state_dict(cleaned_state_dict, strict=False)

    print("[visual_perception:osnet_export] Loaded checkpoint")
    if missing:
        print(f"[visual_perception:osnet_export] Missing keys: {len(missing)}")
    if unexpected:
        print(f"[visual_perception:osnet_export] Unexpected keys: {len(unexpected)}")


def export_onnx(checkpoint_path: Path, onnx_path: Path, input_height: int = 256, input_width: int = 128, opset: int = 18, ) -> None:
    model = build_osnet_model()
    load_checkpoint_weights(model, checkpoint_path)

    model.eval()

    dummy = torch.randn(1, 3, input_height, input_width, dtype=torch.float32)

    onnx_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"[visual_perception:osnet_export] Exporting ONNX to {onnx_path}")

    with torch.no_grad():
        torch.onnx.export(
            model,
            dummy,
            str(onnx_path),
            input_names=["images"],
            output_names=["embeddings"],
            opset_version=opset,
            do_constant_folding=True,
            dynamic_axes=None,
        )

    print("[visual_perception:osnet_export] Export complete")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True, help="Path to .pth checkpoint")
    parser.add_argument("--onnx", required=True, help="Output path for .onnx file")
    parser.add_argument("--height", type=int, default=256, help="Input height")
    parser.add_argument("--width", type=int, default=128, help="Input width")
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset")
    args = parser.parse_args()

    checkpoint_path = Path(args.checkpoint)
    onnx_path = Path(args.onnx)

    if not checkpoint_path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

    export_onnx(
        checkpoint_path=checkpoint_path,
        onnx_path=onnx_path,
        input_height=args.height,
        input_width=args.width,
        opset=args.opset,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())