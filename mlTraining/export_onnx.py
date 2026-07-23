"""
Export a trained regression checkpoint (assist or turn) to ONNX for Pi inference.

What this does:
1. Loads checkpoints/{model}_best.pt
2. Traces the model with a dummy input of shape (1, NUM_CHANNELS, window_size)
3. Writes checkpoints/{model}_best.onnx
4. Writes checkpoints/{model}_onnx_meta.json — a sidecar that describes the
   model's input shape, sample rate, and label normalization scale so the Pi
   can load the model without duplicating training-side constants
5. Verifies the ONNX model produces the same output as the torch model
   (max abs diff) across a batch of random inputs

Usage:
    python export_onnx.py --model assist
    python export_onnx.py --model turn

Notes on the ONNX graph:
- Input name:  "imu_window"   shape (batch, 6, window_size)   dtype float32
- Output name: "prediction"   shape (batch,)                  dtype float32
  The output is the regression scalar directly — no softmax, no post-processing.
  It is in *normalized label units*: multiply by label_norm_scale (from the
  metadata sidecar) to recover physical units (m/s^2 forward accel for assist,
  yaw rate for turn).
- Batch dim is marked dynamic so the same graph works for single-sample
  inference on the Pi or batched offline evaluation.
"""

from __future__ import annotations

import argparse
import json

import numpy as np
import torch

from config import (
    CHANNEL_NAMES,
    CHECKPOINT_DIR,
    IDLE_GATE_ENTER_THRESHOLD,
    IDLE_GATE_EXIT_THRESHOLD,
    IDLE_GATE_WINDOW_S,
    LABEL_BLEND_BOAT_TURN_HIGH,
    LABEL_BLEND_BOAT_TURN_LOW,
    LABEL_BLEND_PADDLE_ENERGY_HIGH,
    LABEL_BLEND_PADDLE_ENERGY_LOW,
    MODEL_TYPE_ASSIST,
    MODEL_TYPE_TURN,
    MODEL_TYPES,
    NUM_CHANNELS,
    SAMPLE_RATE_HZ,
)
from model import build_model, input_window_size


def _artifact_paths(model_type: str) -> dict:
    """Everything export reads and writes for one model type."""
    return {
        "checkpoint": CHECKPOINT_DIR / f"{model_type}_best.pt",
        "train_meta": CHECKPOINT_DIR / f"{model_type}_meta.json",
        "onnx": CHECKPOINT_DIR / f"{model_type}_best.onnx",
        "onnx_meta": CHECKPOINT_DIR / f"{model_type}_onnx_meta.json",
    }


def main():
    parser = argparse.ArgumentParser(
        description="Export a trained regression model to ONNX.")
    parser.add_argument("--model", choices=MODEL_TYPES, required=True,
                        help="Which trained model to export ('assist' or 'turn').")
    args = parser.parse_args()

    paths = _artifact_paths(args.model)
    if not paths["checkpoint"].exists():
        raise FileNotFoundError(
            f"No checkpoint at {paths['checkpoint']}. "
            f"Train the model first: python train.py --model {args.model} --log-dirs <dirs>"
        )

    # Load on CPU — export is device-agnostic and CPU keeps the exported graph clean
    device = torch.device("cpu")
    checkpoint = torch.load(paths["checkpoint"], map_location=device, weights_only=True)

    model = build_model(args.model).to(device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()

    print(f"Loaded {args.model} checkpoint from epoch {checkpoint['epoch']} "
          f"(validation_loss={checkpoint['val_loss']:.4f}, "
          f"validation_mae={checkpoint['val_mae']:.4f}, "
          f"validation_r2={checkpoint['val_r2']:.4f})")

    # Training metadata sidecar carries the LabelConfig this checkpoint was
    # trained against — including the label normalization scale the Pi needs
    # to convert predictions back to physical units.
    label_config = None
    if paths["train_meta"].exists():
        with open(paths["train_meta"]) as f:
            label_config = json.load(f).get("label_config")
    if label_config is None:
        raise FileNotFoundError(
            f"Training metadata missing or has no label_config: {paths['train_meta']}. "
            "Re-run train.py so the label normalization scale is recorded — the Pi "
            "cannot interpret predictions without it."
        )
    norm_scale_key = ("assist_norm_scale" if args.model == MODEL_TYPE_ASSIST
                      else "turn_norm_scale")
    label_norm_scale = float(label_config[norm_scale_key])
    print(f"Label normalization scale ({norm_scale_key}): {label_norm_scale}")

    window_size = input_window_size(args.model)

    # Dummy input used to trace the graph. Content doesn't matter, only shape/dtype.
    dummy_input = torch.randn(1, NUM_CHANNELS, window_size, dtype=torch.float32)

    torch.onnx.export(
        model,
        dummy_input,
        paths["onnx"].as_posix(),
        input_names=["imu_window"],
        output_names=["prediction"],
        dynamic_axes={
            "imu_window": {0: "batch"},
            "prediction": {0: "batch"},
        },
        opset_version=18,
        dynamo=False,   # legacy exporter → single self-contained .onnx file
    )
    print(f"Exported ONNX model to {paths['onnx']}")

    # Parity check: run the same inputs through torch and onnxruntime and
    # compare. A batch > 1 also exercises the dynamic batch axis.
    import onnxruntime as ort

    batch_input = torch.randn(8, NUM_CHANNELS, window_size, dtype=torch.float32)
    with torch.no_grad():
        torch_out = model(batch_input).cpu().numpy()

    session = ort.InferenceSession(paths["onnx"].as_posix(),
                                   providers=["CPUExecutionProvider"])
    (onnx_out,) = session.run(None, {"imu_window": batch_input.numpy()})

    max_abs_diff = float(np.max(np.abs(torch_out - onnx_out)))
    print(f"torch predictions: {np.round(torch_out.ravel(), 4)}")
    print(f"onnx  predictions: {np.round(onnx_out.ravel(), 4)}")
    print(f"Max abs diff:      {max_abs_diff:.3e}")

    if max_abs_diff > 1e-4:
        raise RuntimeError(
            f"ONNX / torch outputs diverged (max_abs_diff={max_abs_diff:.3e}). "
            "Investigate before shipping to the Pi."
        )
    print("Parity check PASSED.")

    # Sidecar metadata — Pi-side code reads this so training-only constants
    # (window size, sample rate, label scaling) don't get duplicated on the Pi.
    metadata = {
        "model_type": args.model,
        "input_name": "imu_window",
        "output_name": "prediction",
        "num_channels": NUM_CHANNELS,
        "channel_names": CHANNEL_NAMES,
        "window_size": window_size,
        "sample_rate_hz": SAMPLE_RATE_HZ,
        # Multiply the model's output by this to get physical units
        # (forward accel in m/s^2 for assist, yaw rate for turn).
        "label_norm_scale": label_norm_scale,
        "label_config": label_config,
        # Paddle idle gate the Pi runtime mirrors. Since idle label
        # blending, the model IS trained on idle windows (labels forced to
        # zero) and ramps assist down natively — the runtime gate is a
        # deterministic backstop, not the primary idle response.
        # Reference implementation: idle_gate.py.
        "idle_gate": {
            "signal": "rolling_std_of_gyro_magnitude",
            "window_seconds": IDLE_GATE_WINDOW_S,
            "enter_threshold": IDLE_GATE_ENTER_THRESHOLD,
            "exit_threshold": IDLE_GATE_EXIT_THRESHOLD,
        },
        # Training-only, recorded for provenance — the Pi does not act on
        # these. Labels were scaled toward zero by smoothstep blends; changing
        # them requires a retrain to take effect. The paddle-energy blend
        # (config.py "Paddle-energy label blending") applies to every model;
        # the boat-turn deadband ("Boat-turn label blending") applies only to
        # the turn model, so it is recorded only there.
        "label_blend": {
            "paddle_energy_low": LABEL_BLEND_PADDLE_ENERGY_LOW,
            "paddle_energy_high": LABEL_BLEND_PADDLE_ENERGY_HIGH,
            **({"boat_turn_low": LABEL_BLEND_BOAT_TURN_LOW,
                "boat_turn_high": LABEL_BLEND_BOAT_TURN_HIGH}
               if args.model == MODEL_TYPE_TURN else {}),
        },
        "checkpoint_epoch": int(checkpoint["epoch"]),
        "checkpoint_validation_loss": float(checkpoint["val_loss"]),
        "checkpoint_validation_mae": float(checkpoint["val_mae"]),
        "checkpoint_validation_r2": float(checkpoint["val_r2"]),
    }
    with open(paths["onnx_meta"], "w") as f:
        json.dump(metadata, f, indent=2)
    print(f"Wrote model metadata sidecar to {paths['onnx_meta']}")


if __name__ == "__main__":
    main()
