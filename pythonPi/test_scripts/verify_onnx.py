"""
Verify the ONNX stroke-detector model loads and runs on the Pi.

Runs the exported StrokeCNN graph via onnxruntime on a dummy input and prints
class probabilities plus per-inference timing. This is only a plumbing check —
it confirms the ONNX runtime is installed, the graph loads, normalization stats
load, and inference latency is reasonable before wiring the real data path.

Usage (on the Pi, from pythonPi/):
    python test_scripts/verify_onnx.py

Dependencies:
    pip install onnxruntime numpy

Artifacts referenced via relative path:
    ../mlTraining/checkpoints/best_model.onnx
    ../mlTraining/checkpoints/norm_stats.json
"""

import json
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort


# Model input shape must match mlTraining/config.py (NUM_CHANNELS=6, WINDOW_SIZE=40)
NUM_CHANNELS = 6
WINDOW_SIZE = 40
CLASS_NAMES = ["no_stroke", "stroke"]

CHECKPOINTS_DIR = Path(__file__).parent.parent.parent / "mlTraining" / "checkpoints"
MODEL_PATH = CHECKPOINTS_DIR / "best_model.onnx"
NORM_STATS_PATH = CHECKPOINTS_DIR / "norm_stats.json"

NUM_WARMUP_RUNS = 3
NUM_TIMED_RUNS = 20


def softmax(logits):
    """Convert raw model outputs into class probabilities that sum to 1."""
    shifted = logits - logits.max()
    exp = np.exp(shifted)
    return exp / exp.sum()


def main():
    if not MODEL_PATH.exists():
        raise FileNotFoundError(
            f"No ONNX model at {MODEL_PATH}. Run export_onnx.py in mlTraining/ first."
        )
    if not NORM_STATS_PATH.exists():
        raise FileNotFoundError(
            f"No normalization stats at {NORM_STATS_PATH}."
        )

    # Normalization statistics (per-channel mean and standard deviation)
    # computed from the training set. Must be applied to every input window
    # before inference — same preprocessing the model saw during training.
    with open(NORM_STATS_PATH) as f:
        stats = json.load(f)
    channel_means = np.array(stats["means"], dtype=np.float32)
    channel_standard_deviations = np.array(stats["stds"], dtype=np.float32)
    print(f"Loaded normalization statistics from {NORM_STATS_PATH.name}")
    print(f"  per-channel means:               {channel_means}")
    print(f"  per-channel standard deviations: {channel_standard_deviations}")

    # Load the ONNX graph into an inference session
    session = ort.InferenceSession(MODEL_PATH.as_posix(),
                                   providers=["CPUExecutionProvider"])
    input_metadata = session.get_inputs()[0]
    output_metadata = session.get_outputs()[0]
    print(f"\nLoaded ONNX model from {MODEL_PATH.name}")
    print(f"  input tensor name:  {input_metadata.name}  shape={input_metadata.shape}")
    print(f"  output tensor name: {output_metadata.name}  shape={output_metadata.shape}")

    # Build one dummy window: random floats shaped like a real oar IMU window.
    # Shape (1, 6, 40) = (batch, channels, time_steps).
    raw_window = np.random.randn(NUM_CHANNELS, WINDOW_SIZE).astype(np.float32)
    # Normalize per channel: (sample - channel_mean) / channel_standard_deviation
    # Means/stds are shape (6,); broadcast against (6, 40) along the time axis.
    normalized_window = ((raw_window - channel_means[:, None])
                         / channel_standard_deviations[:, None])
    model_input = normalized_window[None, :, :]   # add batch dimension → (1, 6, 40)

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

    # The raw model output ("logits") is a vector of unnormalized scores — one
    # per class. Softmax turns those into probabilities in [0, 1] that sum to 1.
    logits = last_output[0]
    probabilities = softmax(logits)
    print(f"\nRaw output (logits):    {logits}")
    print("Class probabilities (after softmax):")
    for class_name, probability in zip(CLASS_NAMES, probabilities):
        print(f"  {class_name:<10} {probability:.4f}")

    predicted_class_index = int(np.argmax(logits))
    print(f"\nPredicted class: {CLASS_NAMES[predicted_class_index]} "
          f"(probability={probabilities[predicted_class_index]:.4f})")
    print("\nVerification complete.")


if __name__ == "__main__":
    main()
