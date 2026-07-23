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
    LABEL_BLEND_BOAT_TURN_HIGH,
    LABEL_BLEND_BOAT_TURN_LOW,
    LABEL_BLEND_PADDLE_ENERGY_HIGH,
    LABEL_BLEND_PADDLE_ENERGY_LOW,
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


def compute_label_blend_weights(gyro_xyz: np.ndarray,
                                sample_rate_hz: float = SAMPLE_RATE_HZ
                                ) -> np.ndarray:
    """Per-sample weight in [0, 1] that scales training labels toward zero
    as paddle motion energy falls (see config.py "Paddle-energy label
    blending"). Applies to both the assist and turn labels.

    Training-only — the runtime gate on the Pi stays the binary hysteresis
    state machine above. Unlike the gate, this is a memoryless function of
    the current energy value: smoothstep from 0 at LABEL_BLEND_PADDLE_ENERGY_LOW
    to 1 at LABEL_BLEND_PADDLE_ENERGY_HIGH. Smoothstep (a²(3-2a)) rather than a
    linear ramp so the label-vs-energy curve has no slope discontinuities
    at the two thresholds — a smooth target is easier for the network to
    represent than one with corners.

    NaN energy (incomplete trailing window at session start) maps to
    weight 0, matching the gate's "incomplete window counts as idle" rule.
    """
    energy = paddle_motion_energy(gyro_xyz, sample_rate_hz)
    a = ((energy - LABEL_BLEND_PADDLE_ENERGY_LOW)
         / (LABEL_BLEND_PADDLE_ENERGY_HIGH - LABEL_BLEND_PADDLE_ENERGY_LOW))
    a = np.clip(np.nan_to_num(a, nan=0.0), 0.0, 1.0)
    return a * a * (3.0 - 2.0 * a)


def compute_turn_deadband_weights(turn_label: np.ndarray) -> np.ndarray:
    """Per-sample weight in [0, 1] that blends the turn label toward zero for
    small boat rotations (see config.py "Boat-turn label blending").

    Training-only, turn model only. Smoothstep on |turn_label|: 0 at or below
    LABEL_BLEND_BOAT_TURN_LOW, 1 at or above LABEL_BLEND_BOAT_TURN_HIGH, smooth
    between — so straight-line wobble is zeroed and real turns pass through,
    with no slope discontinuity for the network to fit around. Same smoothstep
    shape as compute_label_blend_weights, keyed on the label magnitude (boat
    yaw) instead of paddle motion energy.

    NaN labels (end-of-session future window) propagate to NaN weight so the
    caller's validity check still drops them — unlike the paddle-energy blend,
    a NaN turn label is not known to be zero. If HIGH <= LOW the deadband is
    disabled and every finite label gets weight 1.
    """
    absv = np.abs(np.asarray(turn_label, dtype=float))
    if LABEL_BLEND_BOAT_TURN_HIGH <= LABEL_BLEND_BOAT_TURN_LOW:
        return np.ones_like(absv)
    a = ((absv - LABEL_BLEND_BOAT_TURN_LOW)
         / (LABEL_BLEND_BOAT_TURN_HIGH - LABEL_BLEND_BOAT_TURN_LOW))
    a = np.clip(a, 0.0, 1.0)
    return a * a * (3.0 - 2.0 * a)
