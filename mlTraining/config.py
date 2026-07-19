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
# Both paddle IMU (over RF) and kayak IMU (local on Pi) sample at this rate.
# Keep them aligned -- the regression pipeline pairs samples by timestamp
# and the math assumes a single shared period.
SAMPLE_RATE_HZ = 20
CHANNEL_NAMES = [
    "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z",
]
NUM_CHANNELS = len(CHANNEL_NAMES)

# ── Kayak body-frame axes ─────────────────────────────────────────────────
# After the IMU remap commit, kayak accel_x is forward, gyro_z is yaw rate.
# Pulled out as constants so labels.py / dataset.py don't hardcode strings.
KAYAK_FORWARD_ACCEL_COLUMN = "accel_x"
KAYAK_YAW_RATE_COLUMN = "gyro_z"

# ── Regression-label generation ───────────────────────────────────────────
# For each kayak IMU sample at time t, the label is the mean of a future
# window of kayak signal starting `delay` ms ahead and lasting `window` ms.
# At 20 Hz: 100 ms = 2 samples, 600 ms = 12 samples, 2000 ms = 40 samples.
LABEL_ASSIST_DELAY_MS = 100      # head start so model predicts future, not past
LABEL_ASSIST_WINDOW_MS = 600     # wide enough to absorb session-to-session jitter
LABEL_TURN_DELAY_MS = 200        # yaw response is slower than surge
LABEL_TURN_WINDOW_MS = 2000      # longer integration window for smoother turn target

# Divisors that bring labels into roughly unit scale before training.
# Start at 1.0; once you have data, replace with ~95th percentile of |label|.
LABEL_ASSIST_NORM_SCALE = 0.383     # recomputed 7-14-26, idle-gated, over Ongoing_Training_Sessions (6 sessions, 61034 samples)
LABEL_TURN_NORM_SCALE = 0.122       # recomputed 7-14-26, idle-gated (new sessions have far more turning than 7-4 corpus)

# ── Paddle input windows (per-model) ──────────────────────────────────────
# Each model gets a fixed window of paddle IMU history at every prediction.
# Sizes are tuned to the timescale of what the model needs to detect:
#   Assist  — short window: stroke onset shows up in 200-300 ms of paddle data.
#   Turn    — long window: turn intent often needs a full stroke or two of
#             context to disambiguate from straight paddling.
# Stride is always 1 sample — every sample becomes a training example, which
# matches the streaming inference pattern on the Pi (one inference per IMU
# packet). The 75% stride from Phase 1 is obsolete here.
WINDOW_SIZE_ASSIST = 5   # 250 ms at 20 Hz
WINDOW_SIZE_TURN = 40    # 2 s at 20 Hz

# ── Paddle idle gate ──────────────────────────────────────────────────────
# Detects "not paddling" from paddle IMU motion energy: the rolling standard
# deviation of gyro magnitude sqrt(gx²+gy²+gz²) over a trailing window. Gyro
# (not accel) because a resting paddle reads ~0 on gyros regardless of
# orientation, while accelerometers always see gravity.
#
# Used in two places that MUST share behavior (these constants are the single
# source of truth; export_onnx.py copies them into the ONNX metadata sidecar
# so the Pi runtime gate reads the same values):
#   1. Training: dataset.py drops paddle-idle samples — their labels are wind
#      drift / coast-down the paddle input cannot explain.
#   2. Runtime (Pi): MlManager gates assist to zero while paddle-idle.
#
# Hysteresis: enter idle when energy drops below ENTER, exit as soon as it
# rises above EXIT. The band between them holds the previous state so the
# gate doesn't chatter at the boundary. Session data (7-11-26) shows idle
# energy < 0.2 and active paddling at 0.7-1.4, so the thresholds sit in a
# roughly order-of-magnitude gap. An incomplete window (startup) counts as
# idle — the safe state: no assist until paddling is confirmed.
IDLE_GATE_WINDOW_S = 3.0
IDLE_GATE_ENTER_THRESHOLD = 0.2
IDLE_GATE_EXIT_THRESHOLD = 0.4

# ── Idle label blending (training only) ───────────────────────────────────
# Instead of dropping paddle-idle samples from training, keep them and blend
# their labels toward zero based on paddle motion energy (same rolling-std
# signal the idle gate uses):
#   energy <= LOW            → label forced to 0 (paddle is idle; the correct
#                              assist is zero, and the IMU-derived label is
#                              pure wind/current/coast-down noise anyway)
#   energy >= HIGH           → IMU-derived label trusted fully
#   LOW < energy < HIGH      → label scaled by smoothstep between the two
# This teaches the model a true 0→max output range with a smooth taper, so
# at inference it ramps assist down toward zero on its own instead of
# floating just above the deadband until the runtime idle gate cuts it.
# The runtime gate (constants above) is unchanged and becomes a backstop.
#
# LOW sits at the idle-gate enter threshold (idle energy < 0.2 in session
# data). HIGH sits just under the weakest genuine strokes (active paddling
# measured at 0.7-1.4 on 7-11-26 sessions). Labels in the band are
# deliberately attenuated — light paddling gets light assist — and the band
# is also where kayak-IMU labels are least trustworthy (low signal-to-noise),
# so attenuation doubles as label-noise suppression.
LABEL_BLEND_ENERGY_LOW = IDLE_GATE_ENTER_THRESHOLD
LABEL_BLEND_ENERGY_HIGH = 0.7

# Model identifiers. Used as the string passed to build_model() and as the
# key for selecting which label column a SessionDataset returns.
MODEL_TYPE_ASSIST = "assist"
MODEL_TYPE_TURN = "turn"
MODEL_TYPES = (MODEL_TYPE_ASSIST, MODEL_TYPE_TURN)

# ── Evaluation ────────────────────────────────────────────────────────────
# When computing direction accuracy for the turn model, samples with
# |label| below this cutoff are considered neutral (boat effectively not
# turning) and skipped — sign() of ~0 is not a meaningful "left vs right"
# call. Kept centralized so train.py and evaluate.py stay consistent.
DIRECTION_ACCURACY_THRESHOLD = 0.05

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
