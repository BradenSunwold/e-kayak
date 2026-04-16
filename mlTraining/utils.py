"""
Utilities: device selection, reproducibility seeding, and normalization stats I/O.
"""

import json
import random

import numpy as np
import torch

from config import CHECKPOINT_DIR, RANDOM_SEED

NORM_STATS_PATH = CHECKPOINT_DIR / "norm_stats.json"


def get_device():
    """Pick the best available device: MPS (Apple Silicon) > CUDA > CPU."""
    if torch.backends.mps.is_available():
        return torch.device("mps")
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


def seed_everything(seed=RANDOM_SEED):
    """Set seeds for Python, NumPy, and PyTorch so results are reproducible."""
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)


def save_norm_stats(means, stds):
    """Save per-channel mean/std to JSON so inference can use the same normalization."""
    CHECKPOINT_DIR.mkdir(parents=True, exist_ok=True)
    with open(NORM_STATS_PATH, "w") as f:
        json.dump({"means": means.tolist(), "stds": stds.tolist()}, f, indent=2)


def load_norm_stats():
    """Load per-channel mean/std from JSON. Returns (means, stds) as numpy arrays."""
    with open(NORM_STATS_PATH) as f:
        data = json.load(f)
    return np.array(data["means"]), np.array(data["stds"])
