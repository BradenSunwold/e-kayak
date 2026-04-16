"""
Central configuration for paddle stroke detection model training.

All constants live here so they're easy to find and change in one place.
When you move to Phase 2 (left/right/no_stroke), you'll just update LABEL_MAP here.
"""

from pathlib import Path

# ── Paths ──────────────────────────────────────────────────────────────────
BASE_DIR = Path(__file__).parent
DATA_DIR = BASE_DIR / "data"
RAW_DIR = DATA_DIR / "raw"
CHECKPOINT_DIR = BASE_DIR / "checkpoints"

# ── IMU Sensor Config ─────────────────────────────────────────────────────
SAMPLE_RATE_HZ = 20          # Oar IMU rate over RF link
CHANNEL_NAMES = [
    "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z",
]
NUM_CHANNELS = len(CHANNEL_NAMES)

# ── Windowing ──────────────────────────────────────────────────────────────
# A full paddle stroke cycle takes roughly 1-2 seconds.  A 2-second window
# comfortably captures one full stroke.
WINDOW_DURATION_SEC = 2.0
WINDOW_STRIDE_SEC = 0.5      # 75% overlap between consecutive windows

WINDOW_SIZE = int(SAMPLE_RATE_HZ * WINDOW_DURATION_SEC)    # 40 samples
WINDOW_STRIDE = int(SAMPLE_RATE_HZ * WINDOW_STRIDE_SEC)    # 10 samples

# ── Labels ─────────────────────────────────────────────────────────────────
# Phase 1: binary stroke / no-stroke
# Phase 2: change to {"no_stroke": 0, "left_stroke": 1, "right_stroke": 2}
LABEL_MAP = {
    "no_stroke": 0,
    "stroke": 1,
}
NUM_CLASSES = len(LABEL_MAP)

# ── Training Hyperparameters ──────────────────────────────────────────────
# How many windows the model processes at once before updating its weights.
# Bigger = more stable gradient estimates but uses more memory.
# 32 is a common starting point; try 16 if you run out of memory, 64 if training is noisy.
BATCH_SIZE = 32

# How big of a step the optimizer takes when adjusting weights each batch.
# Too high = overshoots and never converges. Too low = learns very slowly.
# 1e-3 (0.001) is the default for Adam and a good starting point.
LEARNING_RATE = 1e-3

# Penalizes large weights to prevent overfitting (L2 regularization).
# The optimizer slightly shrinks all weights each step by this factor.
# 1e-4 is mild -- increase if the model overfits, decrease/zero if it underfits.
WEIGHT_DECAY = 1e-4

# Number of complete passes through the full training dataset.
# More epochs = more chances to learn, but too many = overfitting.
# We save the best model (lowest validation loss) so extra epochs are safe.
EPOCHS = 50
VAL_SPLIT = 0.2              # 20% of *files* held out for validation
DROPOUT = 0.3                # Fraction of neurons randomly disabled during
                             # training to prevent overfitting
RANDOM_SEED = 42
