"""
Export the best trained StrokeCNN checkpoint to ONNX for Pi inference.

What this does:
1. Loads checkpoints/best_model.pt
2. Traces the model with a dummy input of shape (1, NUM_CHANNELS, WINDOW_SIZE)
3. Writes checkpoints/best_model.onnx
4. Verifies the ONNX model produces the same output as the torch model (max abs diff)

Usage:
    python export_onnx.py

Notes on the ONNX graph:
- Input name:  "imu_window"  shape (batch, 6, 40)    dtype float32
- Output name: "logits"      shape (batch, 2)        dtype float32
  Run softmax on the Pi side to get class probabilities.
- Batch dim is marked dynamic so the same graph works for single-sample inference
  or batched evaluation.
"""

import numpy as np
import torch

from config import CHECKPOINT_DIR, NUM_CHANNELS, WINDOW_SIZE
from model import StrokeCNN


CHECKPOINT_PATH = CHECKPOINT_DIR / "best_model.pt"
ONNX_PATH = CHECKPOINT_DIR / "best_model.onnx"


def main():
    if not CHECKPOINT_PATH.exists():
        raise FileNotFoundError(
            f"No checkpoint at {CHECKPOINT_PATH}. Train the model first with train.py."
        )

    # Load on CPU — export is device-agnostic and CPU keeps the exported graph clean
    device = torch.device("cpu")
    checkpoint = torch.load(CHECKPOINT_PATH, map_location=device, weights_only=True)

    model = StrokeCNN().to(device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()

    print(f"Loaded checkpoint from epoch {checkpoint['epoch']} "
          f"(validation_loss={checkpoint['val_loss']:.4f}, "
          f"validation_accuracy={checkpoint['val_acc']:.4f})")

    # Dummy input used to trace the graph. Content doesn't matter, only shape/dtype.
    dummy_input = torch.randn(1, NUM_CHANNELS, WINDOW_SIZE, dtype=torch.float32)

    torch.onnx.export(
        model,
        dummy_input,
        ONNX_PATH.as_posix(),
        input_names=["imu_window"],
        output_names=["logits"],
        dynamic_axes={
            "imu_window": {0: "batch"},
            "logits": {0: "batch"},
        },
        opset_version=18,
    )
    print(f"Exported ONNX model to {ONNX_PATH}")

    # Parity check: run the same input through torch and onnxruntime, compare outputs
    import onnxruntime as ort

    with torch.no_grad():
        torch_out = model(dummy_input).cpu().numpy()

    session = ort.InferenceSession(ONNX_PATH.as_posix(), providers=["CPUExecutionProvider"])
    (onnx_out,) = session.run(None, {"imu_window": dummy_input.numpy()})

    max_abs_diff = np.max(np.abs(torch_out - onnx_out))
    print(f"torch output:  {torch_out.ravel()}")
    print(f"onnx  output:  {onnx_out.ravel()}")
    print(f"Max abs diff:  {max_abs_diff:.3e}")

    if max_abs_diff > 1e-4:
        raise RuntimeError(
            f"ONNX / torch outputs diverged (max_abs_diff={max_abs_diff:.3e}). "
            "Investigate before shipping to the Pi."
        )
    print("Parity check PASSED.")


if __name__ == "__main__":
    main()
