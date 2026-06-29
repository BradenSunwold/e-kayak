"""
Integrate kayak IMU into a 2D top-down trajectory for playback visualization.

This is a *visualization* trajectory, not a navigation estimate. We do not
fuse GPS, do not use a Kalman filter, do not try to be metric-accurate. We
just want the path drawn on screen to feel like the boat moving — surging
during strokes, turning on yaw input, coasting between strokes.

Three integrators, each with a known compromise:

  Heading from gyro_z:
    Bias-corrected with the first ~2 s of data (assumes the boat is roughly
    stationary at session start). Cumulative integration drifts slowly —
    expect a few degrees per minute of error, which is fine for visual
    playback over single sessions.

  Forward velocity from accel_x (forward axis):
    Pure integration is unbounded — sensor bias of 0.05 m/s^2 turns into
    a phantom 1.5 m/s after 30 s. To mask this we apply a first-order
    velocity decay each step (velocity *= exp(-dt/tau)), modeling water
    drag. Bias drift gets absorbed into the same decay so it doesn't
    accumulate. Not physically faithful (real drag is quadratic), but
    perceptually correct.

  Position from velocity + heading:
    Just dead-reckoning: x += v*cos(heading)*dt, y += v*sin(heading)*dt.
    The shape of the path is informative even if the absolute positions
    drift over minutes.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd

from config import KAYAK_FORWARD_ACCEL_COLUMN, KAYAK_YAW_RATE_COLUMN


@dataclass
class TrajectoryConfig:
    """Knobs for the visualization-trajectory integrator."""

    # First N seconds of data used to estimate gyro_z zero bias. Assumes the
    # boat is stationary in that window — if it's drifting at session start,
    # this injects error rather than removing it. Set to 0 to skip bias
    # correction entirely (the default — heading will drift more slowly
    # than the bias is wrong-guessed).
    bias_estimation_seconds: float = 0.0

    # Velocity decay time constant in seconds. Smaller = faster decay
    # (boat coasts to a stop quickly). 3-5 s feels about right for kayaks
    # but tune to taste based on how the playback looks.
    drag_tau_seconds: float = 4.0

    # Which IMU columns to use. Defaults to the body-frame mapping
    # established in config.py.
    forward_accel_column: str = KAYAK_FORWARD_ACCEL_COLUMN
    yaw_rate_column: str = KAYAK_YAW_RATE_COLUMN

    # Sign multipliers. If the boat turns the wrong way on screen, flip
    # yaw_sign to -1. If it surges backward when paddling forward, flip
    # forward_sign. Depends on IMU mounting orientation.
    yaw_sign: float = 1.0
    forward_sign: float = 1.0


def compute_trajectory(kayak_df: pd.DataFrame, config: TrajectoryConfig) -> pd.DataFrame:
    """Integrate kayak IMU into a 2D trajectory.

    Returns DataFrame aligned with kayak_df, columns:
      timestamp, x, y, heading_rad, velocity
    """
    if len(kayak_df) < 2:
        return pd.DataFrame(columns=["timestamp", "x", "y", "heading_rad", "velocity"])

    timestamps = kayak_df["timestamp"].reset_index(drop=True)
    dt_seconds = timestamps.diff().dt.total_seconds().fillna(0.0).to_numpy()

    yaw_rate_raw = kayak_df[config.yaw_rate_column].to_numpy(dtype=float)
    if config.bias_estimation_seconds > 0:
        # Estimate gyro zero bias from the leading "stationary" window.
        bias_samples = max(1, int(config.bias_estimation_seconds /
                                  max(np.median(dt_seconds[1:]), 1e-3)))
        gyro_bias = float(yaw_rate_raw[:bias_samples].mean())
    else:
        gyro_bias = 0.0
    yaw_rate = config.yaw_sign * (yaw_rate_raw - gyro_bias)

    forward_accel = config.forward_sign * kayak_df[config.forward_accel_column].to_numpy(dtype=float)

    # Heading: cumulative integral of yaw rate. Starts at 0 rad.
    heading_rad = np.cumsum(yaw_rate * dt_seconds)

    # Velocity with first-order drag. Loop is short enough at session length
    # that a plain Python loop is fine; vectorizing would obscure the recurrence.
    velocity = np.zeros(len(kayak_df))
    for i in range(1, len(kayak_df)):
        dt = dt_seconds[i]
        # Exponential decay (closed-form solution to v' = -v/tau)
        decay = np.exp(-dt / config.drag_tau_seconds) if dt > 0 else 1.0
        velocity[i] = (velocity[i - 1] + forward_accel[i] * dt) * decay

    # Position: dead-reckoning along the heading vector.
    x = np.cumsum(velocity * np.cos(heading_rad) * dt_seconds)
    y = np.cumsum(velocity * np.sin(heading_rad) * dt_seconds)

    return pd.DataFrame({
        "timestamp": timestamps,
        "x": x,
        "y": y,
        "heading_rad": heading_rad,
        "velocity": velocity,
    })
