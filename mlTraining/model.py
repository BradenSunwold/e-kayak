"""
Two 1D-CNN regression models, one per output target.

  AssistCNN — predicts assist scalar from a 250 ms paddle window (5 samples).
              Small architecture matching the short input.
  TurnCNN   — predicts turn scalar from a 2 s paddle window (40 samples).
              Same 3-block layout as the previous classifier, regression head.

Both consume the same 6-channel paddle IMU (raw accel + gyro), produce a
single scalar prediction per window, and are trained with SmoothL1Loss
(Huber) against the future-window-mean labels computed in plotters.labels.

Use build_model(model_type) to construct the right one from a string —
this is the public API the training, evaluation, and export scripts hit.
"""

from __future__ import annotations

import torch
import torch.nn as nn

from config import (
    DROPOUT,
    MODEL_TYPE_ASSIST,
    MODEL_TYPE_TURN,
    NUM_CHANNELS,
    TURN_CNN_CHANNELS,
    WINDOW_SIZE_ASSIST,
    WINDOW_SIZE_TURN,
)


class AssistCNN(nn.Module):
    """Compact CNN for the assist (forward thrust) regression target.

    Input:  (batch, 6 channels, 5 samples)  — 250 ms of paddle IMU @ 20 Hz
    Output: (batch,)                         — single scalar per window

    Layer-by-layer shapes:
        Conv1d(6 → 16, k=3, pad=1) → BN → ReLU         (batch, 16, 5)
        Conv1d(16 → 32, k=3, pad=1) → BN → ReLU        (batch, 32, 5)
        AdaptiveAvgPool1d(1)                            (batch, 32, 1)
        squeeze last dim                                (batch, 32)
        Dropout → Linear(32 → 1)                        (batch, 1)
        squeeze last dim                                (batch,)

    No pooling between conv blocks: a 5-sample window can't survive two
    halvings, and the receptive field of two stacked kernel=3 convs is
    already 5 — covering the whole window in one pass.

    ~2.5K parameters.
    """

    def __init__(self):
        super().__init__()
        self.block1 = nn.Sequential(
            nn.Conv1d(NUM_CHANNELS, 16, kernel_size=3, padding=1),
            nn.BatchNorm1d(16),
            nn.ReLU(),
        )
        self.block2 = nn.Sequential(
            nn.Conv1d(16, 32, kernel_size=3, padding=1),
            nn.BatchNorm1d(32),
            nn.ReLU(),
        )
        # Collapse the remaining 5 time steps into one feature vector by
        # averaging — robust to small input-window length changes.
        self.pool = nn.AdaptiveAvgPool1d(1)
        self.head = nn.Sequential(
            nn.Dropout(DROPOUT),
            nn.Linear(32, 1),
        )

    def forward(self, x):
        # x: (batch, 6, 5)
        x = self.block1(x)         # (batch, 16, 5)
        x = self.block2(x)         # (batch, 32, 5)
        x = self.pool(x)           # (batch, 32, 1)
        x = x.squeeze(-1)          # (batch, 32)
        x = self.head(x)           # (batch, 1)
        return x.squeeze(-1)       # (batch,) — match label tensor shape


class TurnCNN(nn.Module):
    """CNN for the turn-rate regression target.

    Input:  (batch, 6 channels, 40 samples)  — 2 s of paddle IMU @ 20 Hz
    Output: (batch,)                          — single scalar per window

    Layer-by-layer shapes, with (c1, c2, c3) = TURN_CNN_CHANNELS:
        Conv1d(6 → c1, k=5, pad=2) → BN → ReLU → MaxPool(2)    (batch, c1, 20)
            first layer learns simple patterns (spikes, slopes, oscillations)

        Conv1d(c1 → c2, k=5, pad=2) → BN → ReLU → MaxPool(2)   (batch, c2, 10)
            combines simple patterns into stroke-shaped features

        Conv1d(c2 → c3, k=3, pad=1) → BN → ReLU → AdaptiveAvgPool(1)
                                                                (batch, c3, 1)
            higher-level patterns, then collapse time to one feature vector

        squeeze → Dropout → Linear(c3 → 1)                      (batch, 1)
        squeeze last dim                                        (batch,)

    Channel widths come from config.TURN_CNN_CHANNELS so capacity can be swept
    without editing this file — see that constant for why. The layout is
    unchanged from the original (32, 64, 128) backbone inherited from the
    binary classifier; only the widths are tunable.
    """

    def __init__(self, channels: tuple[int, int, int] = TURN_CNN_CHANNELS):
        super().__init__()
        if len(channels) != 3:
            raise ValueError(
                f"TurnCNN expects exactly 3 conv channel widths, got {channels!r}")
        c1, c2, c3 = channels
        self.block1 = nn.Sequential(
            nn.Conv1d(NUM_CHANNELS, c1, kernel_size=5, padding=2),
            nn.BatchNorm1d(c1),
            nn.ReLU(),
            nn.MaxPool1d(2),       # 40 → 20
        )
        self.block2 = nn.Sequential(
            nn.Conv1d(c1, c2, kernel_size=5, padding=2),
            nn.BatchNorm1d(c2),
            nn.ReLU(),
            nn.MaxPool1d(2),       # 20 → 10
        )
        self.block3 = nn.Sequential(
            nn.Conv1d(c2, c3, kernel_size=3, padding=1),
            nn.BatchNorm1d(c3),
            nn.ReLU(),
            nn.AdaptiveAvgPool1d(1),   # 10 → 1
        )
        self.head = nn.Sequential(
            nn.Dropout(DROPOUT),
            nn.Linear(c3, 1),
        )

    def forward(self, x):
        # x: (batch, 6, 40)
        x = self.block1(x)         # (batch, c1, 20)
        x = self.block2(x)         # (batch, c2, 10)
        x = self.block3(x)         # (batch, c3, 1)
        x = x.squeeze(-1)          # (batch, c3)
        x = self.head(x)           # (batch, 1)
        return x.squeeze(-1)       # (batch,) — match label tensor shape


_MODEL_CLASS_FOR = {
    MODEL_TYPE_ASSIST: AssistCNN,
    MODEL_TYPE_TURN: TurnCNN,
}

_WINDOW_SIZE_FOR = {
    MODEL_TYPE_ASSIST: WINDOW_SIZE_ASSIST,
    MODEL_TYPE_TURN: WINDOW_SIZE_TURN,
}


def build_model(model_type: str) -> nn.Module:
    """Construct a fresh (untrained) model for the given type.

    Args:
        model_type: 'assist' or 'turn' — must be one of MODEL_TYPES.

    Returns:
        An instance of AssistCNN or TurnCNN.
    """
    if model_type not in _MODEL_CLASS_FOR:
        raise ValueError(
            f"model_type must be one of {tuple(_MODEL_CLASS_FOR)}, got {model_type!r}")
    return _MODEL_CLASS_FOR[model_type]()


def input_window_size(model_type: str) -> int:
    """Return the expected paddle-input window length (in samples) for the
    given model type. Useful for shape checks and ONNX export."""
    if model_type not in _WINDOW_SIZE_FOR:
        raise ValueError(
            f"model_type must be one of {tuple(_WINDOW_SIZE_FOR)}, got {model_type!r}")
    return _WINDOW_SIZE_FOR[model_type]


def count_parameters(model: nn.Module) -> int:
    """Total number of trainable parameters. Handy for sanity-checking
    that the model is the size you expect."""
    return sum(p.numel() for p in model.parameters() if p.requires_grad)
