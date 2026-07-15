"""
Paddle idle detection shared by the training pipeline and (via the ONNX
metadata sidecar) the Pi runtime gate.

The signal is the rolling standard deviation of paddle gyro magnitude over
a trailing window — see the "Paddle idle gate" block in config.py for the
rationale and the threshold values. This module is the reference
implementation of the gate: dataset.py and create_label_norm_scales.py call
it directly, and the Pi-side rolling implementation must match it
sample-for-sample (same window, same thresholds, same hysteresis rules).

State machine per sample, in order:
  incomplete window (fewer than window_samples seen)  → idle
  currently idle   and energy > IDLE_GATE_EXIT_THRESHOLD   → active
  currently active and energy < IDLE_GATE_ENTER_THRESHOLD  → idle
  otherwise → hold previous state (hysteresis band)

The initial state is idle: on the water that means no assist until paddling
is positively detected, and in training it drops the ambiguous session-start
samples.
"""

from __future__ import annotations

import numpy as np
import pandas as pd

from config import (
    IDLE_GATE_ENTER_THRESHOLD,
    IDLE_GATE_EXIT_THRESHOLD,
    IDLE_GATE_WINDOW_S,
    SAMPLE_RATE_HZ,
)

GYRO_COLUMNS = ["gyro_x", "gyro_y", "gyro_z"]


def paddle_motion_energy(gyro_xyz: np.ndarray,
                         sample_rate_hz: float = SAMPLE_RATE_HZ) -> np.ndarray:
    """Rolling std of gyro magnitude over the trailing gate window.

    Args:
        gyro_xyz: shape (n_samples, 3) paddle gyro readings.

    Returns:
        shape (n_samples,) energy; NaN where the trailing window is not yet
        full (the first window_samples - 1 entries).
    """
    window_samples = int(round(IDLE_GATE_WINDOW_S * sample_rate_hz))
    magnitude = np.sqrt((np.asarray(gyro_xyz, dtype=float) ** 2).sum(axis=1))
    return (pd.Series(magnitude)
            .rolling(window_samples, min_periods=window_samples)
            .std()
            .to_numpy())


def compute_idle_mask(gyro_xyz: np.ndarray,
                      sample_rate_hz: float = SAMPLE_RATE_HZ) -> np.ndarray:
    """Boolean mask over paddle samples: True where the paddle is idle.

    Causal — each sample's state depends only on samples at or before it,
    so the offline mask matches what a streaming implementation on the Pi
    would decide at the same sample.
    """
    energy = paddle_motion_energy(gyro_xyz, sample_rate_hz)
    idle = np.empty(energy.shape[0], dtype=bool)
    state = True  # gate starts closed
    for i, e in enumerate(energy):
        if np.isnan(e):
            state = True
        elif state and e > IDLE_GATE_EXIT_THRESHOLD:
            state = False
        elif not state and e < IDLE_GATE_ENTER_THRESHOLD:
            state = True
        idle[i] = state
    return idle
