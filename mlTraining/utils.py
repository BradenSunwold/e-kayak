"""
Utilities: device selection, reproducibility seeding, and normalization stats I/O.
"""

from __future__ import annotations

import json
import random
from pathlib import Path

import numpy as np
import torch

from config import RANDOM_SEED


def get_device():
    """Pick the best available device: MPS (Apple Silicon) > CUDA > CPU."""
    if torch.backends.mps.is_available():
        return torch.device("mps")
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


def seed_everything(seed: int = RANDOM_SEED) -> None:
    """Set seeds for Python, NumPy, and PyTorch so results are reproducible."""
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)


def save_norm_stats(path: Path, means: np.ndarray, stds: np.ndarray) -> None:
    """Save per-channel paddle mean/std to JSON so inference can apply the
    same normalization the model saw during training."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump({"means": means.tolist(), "stds": stds.tolist()}, f, indent=2)


def load_norm_stats(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load per-channel mean/std from JSON. Returns (means, stds) as float32."""
    with open(path) as f:
        data = json.load(f)
    return (np.asarray(data["means"], dtype=np.float32),
            np.asarray(data["stds"], dtype=np.float32))
