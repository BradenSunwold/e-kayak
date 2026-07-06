"""
Verify a regression ONNX model (assist or turn) loads and runs on the Pi.

Runs the exported graph via onnxruntime on a dummy input and prints the
regression prediction plus per-inference timing. This is only a plumbing
check — it confirms the ONNX runtime is installed, the graph loads, the
normalization statistics and metadata sidecar load, and inference latency
is reasonable before wiring the real data path.

Usage (on the Pi, from pythonPi/):
    python test_scripts/verify_onnx.py --model assist
    python test_scripts/verify_onnx.py --model turn

Dependencies:
    pip install onnxruntime numpy

Artifacts referenced via relative path:
    ../mlTraining/checkpoints/{model}_best.onnx
    ../mlTraining/checkpoints/{model}_norm_stats.json
    ../mlTraining/checkpoints/{model}_onnx_meta.json
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort


CHECKPOINTS_DIR = Path(__file__).parent.parent.parent / "mlTraining" / "checkpoints"

NUM_WARMUP_RUNS = 3
NUM_TIMED_RUNS = 20


def main():
    parser = argparse.ArgumentParser(
        description="Verify a regression ONNX model runs via onnxruntime.")
    parser.add_argument("--model", choices=("assist", "turn"), required=True,
                        help="Which exported model to verify.")
    args = parser.parse_args()

    model_path = CHECKPOINTS_DIR / f"{args.model}_best.onnx"
    norm_stats_path = CHECKPOINTS_DIR / f"{args.model}_norm_stats.json"
    meta_path = CHECKPOINTS_DIR / f"{args.model}_onnx_meta.json"

    for path in (model_path, norm_stats_path, meta_path):
        if not path.exists():
            raise FileNotFoundError(
                f"Missing artifact: {path}. Run export_onnx.py in mlTraining/ first."
            )

    # Metadata sidecar: input shape, tensor names, and the label normalization
    # scale that converts predictions back to physical units.
    with open(meta_path) as f:
        meta = json.load(f)
    num_channels = meta["num_channels"]
    window_size = meta["window_size"]
    label_norm_scale = meta["label_norm_scale"]
    print(f"Loaded metadata from {meta_path.name}")
    print(f"  model_type={meta['model_type']} "
          f"num_channels={num_channels} window_size={window_size}")
    print(f"  label_norm_scale={label_norm_scale}")
    print(f"  trained epoch={meta['checkpoint_epoch']} "
          f"validation_loss={meta['checkpoint_validation_loss']:.4f} "
          f"validation_mean_absolute_error={meta['checkpoint_validation_mae']:.4f}")

    # Normalization statistics (per-channel mean and standard deviation)
    # computed from the training set. Must be applied to every input window
    # before inference — same preprocessing the model saw during training.
    with open(norm_stats_path) as f:
        stats = json.load(f)
    channel_means = np.array(stats["means"], dtype=np.float32)
    channel_standard_deviations = np.array(stats["stds"], dtype=np.float32)
    print(f"Loaded normalization statistics from {norm_stats_path.name}")
    print(f"  per-channel means:               {channel_means}")
    print(f"  per-channel standard deviations: {channel_standard_deviations}")

    # Load the ONNX graph into an inference session
    session = ort.InferenceSession(model_path.as_posix(),
                                   providers=["CPUExecutionProvider"])
    input_metadata = session.get_inputs()[0]
    output_metadata = session.get_outputs()[0]
    print(f"\nLoaded ONNX model from {model_path.name}")
    print(f"  input tensor name:  {input_metadata.name}  shape={input_metadata.shape}")
    print(f"  output tensor name: {output_metadata.name}  shape={output_metadata.shape}")

    # Build one dummy window: random floats shaped like a real oar IMU window.
    # Shape (1, channels, window) = (batch, channels, time_steps).
    raw_window = np.random.randn(num_channels, window_size).astype(np.float32)
    # Normalize per channel: (sample - channel_mean) / channel_standard_deviation
    # Means/stds are shape (channels,); broadcast along the time axis.
    normalized_window = ((raw_window - channel_means[:, None])
                         / channel_standard_deviations[:, None])
    model_input = normalized_window[None, :, :]   # add batch dimension

    # Warm-up runs (first-run cost of the runtime is higher; don't count it)
    for _ in range(NUM_WARMUP_RUNS):
        session.run(None, {input_metadata.name: model_input})

    # Timed inference
    latencies_milliseconds = []
    last_output = None
    for _ in range(NUM_TIMED_RUNS):
        start = time.perf_counter()
        (last_output,) = session.run(None, {input_metadata.name: model_input})
        latencies_milliseconds.append((time.perf_counter() - start) * 1000.0)

    latencies = np.array(latencies_milliseconds)
    print(f"\nInference latency over {NUM_TIMED_RUNS} runs "
          f"(after {NUM_WARMUP_RUNS} warmup runs):")
    print(f"  mean:    {latencies.mean():.3f} ms")
    print(f"  median:  {np.median(latencies):.3f} ms")
    print(f"  minimum: {latencies.min():.3f} ms")
    print(f"  maximum: {latencies.max():.3f} ms")

    # The model output is the regression scalar directly — no softmax.
    # It is in normalized label units; multiplying by label_norm_scale
    # recovers physical units (m/s^2 forward accel for assist, yaw rate
    # for turn).
    prediction = float(last_output[0])
    print(f"\nPrediction (normalized label units): {prediction:+.4f}")
    print(f"Prediction (physical units):         {prediction * label_norm_scale:+.4f}")
    print("\nVerification complete.")


if __name__ == "__main__":
    main()
