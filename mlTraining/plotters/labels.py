"""
Compute regression training labels from kayak IMU data.

The training signal for the new ML pipeline is "what does the kayak do in
the near future." For every kayak IMU sample at time t, we look ahead by
delay_ms, average window_ms worth of samples, and emit that scalar as the
label at t. Two labels are produced:

  assist_label  — future-window mean of kayak forward linear acceleration.
                  Target for Model A (assist percentage).
  turn_label    — future-window mean of kayak yaw rate (gyro Z).
                  Target for Model C (turn rate).

Why a future window:
  Paddle motion at time t causes kayak response a fraction of a second
  later. Labeling paddle inputs with present kayak state would teach the
  model to predict the past. A future window teaches it to predict the
  kayak's near-future response — what the motor controller actually needs.

Why a wide mean:
  Session-to-session variation in paddle force / water conditions shifts
  the exact response delay by tens of ms. A wide future window absorbs
  that variation: the mean of a 600 ms window changes very little whether
  the response peak lands at +150 ms or +400 ms.

This is a pure function: same kayak DataFrame + same LabelConfig always
produces the same labels DataFrame. No I/O, no model, no global state.
Defaults live in config.py; LabelConfig instances are the per-run object
that gets passed around and saved alongside trained checkpoints.
"""

from __future__ import annotations

from dataclasses import dataclass

import pandas as pd

from config import (
    KAYAK_FORWARD_ACCEL_COLUMN,
    KAYAK_YAW_RATE_COLUMN,
    LABEL_ASSIST_DELAY_MS,
    LABEL_ASSIST_NORM_SCALE,
    LABEL_ASSIST_WINDOW_MS,
    LABEL_TURN_DELAY_MS,
    LABEL_TURN_NORM_SCALE,
    LABEL_TURN_WINDOW_MS,
    SAMPLE_RATE_HZ,
)


@dataclass
class LabelConfig:
    """All parameters that define what 'label[t]' means.

    Defaults pull from config.py so there's a single source of truth.
    YAML configs can override any field per session. The whole object
    gets serialized into the trained checkpoint metadata so we always
    know what scalar the model was trained to predict.
    """

    # How far into the future the averaging window starts, per output.
    # Larger = later prediction horizon.
    assist_delay_ms: float = LABEL_ASSIST_DELAY_MS
    turn_delay_ms: float = LABEL_TURN_DELAY_MS

    # How wide the future averaging window is, per output. Larger = more
    # robust to per-session timing jitter, smoother labels, but also smoother
    # predictions (less able to track sharp transients).
    assist_window_ms: float = LABEL_ASSIST_WINDOW_MS
    turn_window_ms: float = LABEL_TURN_WINDOW_MS

    # Which kayak IMU columns are the forward axis and the yaw rate.
    forward_accel_column: str = KAYAK_FORWARD_ACCEL_COLUMN
    yaw_rate_column: str = KAYAK_YAW_RATE_COLUMN

    # Divisors so the model trains against roughly unit-scale targets.
    # Tune to ~95th percentile of |label| once you have enough data.
    assist_norm_scale: float = LABEL_ASSIST_NORM_SCALE
    turn_norm_scale: float = LABEL_TURN_NORM_SCALE


def _samples_for_duration(duration_ms: float) -> int:
    """Convert a duration in ms to a sample count at SAMPLE_RATE_HZ.

    Floors to at least 1 so a 0 ms window still degenerates to "use the
    single sample at the offset" rather than an empty window.
    """
    return max(1, int(round(duration_ms / 1000.0 * SAMPLE_RATE_HZ)))


def _future_window_mean(series: pd.Series, delay_samples: int, window_samples: int) -> pd.Series:
    """Mean of a future window of `series`, anchored at the current sample.

    For each index t in the output: result[t] = mean(series[t+delay ..
    t+delay+window-1]).

    Implementation note: pandas' rolling(N).mean() is *backward*-looking
    (at index i, returns mean over [i-N+1 .. i]). A forward-looking mean at
    t is identical to a backward-looking mean at t + (delay + window - 1)
    — same N samples, just named by their endpoint instead of their start.
    So we compute the backward rolling mean, then shift the result earlier
    in time by (delay + window - 1) so each value lands at the timestamp
    we wanted to label.

    The last (delay + window - 1) samples receive NaN — the future window
    would run off the end of the data.
    """
    shift_samples = -(delay_samples + window_samples - 1)
    return series.rolling(window_samples).mean().shift(shift_samples)


def compute_labels(kayak_df: pd.DataFrame, config: LabelConfig) -> pd.DataFrame:
    """Compute per-sample regression labels from kayak IMU data.

    Returns a DataFrame with the same length as ``kayak_df`` and columns:
      timestamp, assist_label, turn_label

    Samples near the end of the session — where the future window would run
    off the end of the data — receive NaN labels. Drop those before training.
    """
    if len(kayak_df) < 2:
        return pd.DataFrame(columns=["timestamp", "assist_label", "turn_label"])

    forward = kayak_df[config.forward_accel_column].astype(float)
    yaw_rate = kayak_df[config.yaw_rate_column].astype(float)

    assist_delay_samples = _samples_for_duration(config.assist_delay_ms)
    assist_window_samples = _samples_for_duration(config.assist_window_ms)
    turn_delay_samples = _samples_for_duration(config.turn_delay_ms)
    turn_window_samples = _samples_for_duration(config.turn_window_ms)

    assist = (_future_window_mean(forward, assist_delay_samples, assist_window_samples)
              / config.assist_norm_scale)
    turn = (_future_window_mean(yaw_rate, turn_delay_samples, turn_window_samples)
            / config.turn_norm_scale)

    return pd.DataFrame({
        "timestamp": kayak_df["timestamp"].reset_index(drop=True),
        "assist_label": assist.reset_index(drop=True),
        "turn_label": turn.reset_index(drop=True),
    })
