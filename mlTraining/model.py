"""
1D CNN for paddle stroke classification.

Architecture summary:
    Input: (batch, 6, 40)  -- 6 IMU channels, 40 time steps (2s at 20Hz)

    Conv1d(6 → 32, kernel=5, pad=2) → BatchNorm → ReLU → MaxPool(2)
        Output: (batch, 32, 20)
        First layer learns simple patterns: spikes, slopes, oscillations

    Conv1d(32 → 64, kernel=5, pad=2) → BatchNorm → ReLU → MaxPool(2)
        Output: (batch, 64, 10)
        Combines simple patterns into stroke-shaped features

    Conv1d(64 → 128, kernel=3, pad=1) → BatchNorm → ReLU → AdaptiveAvgPool(1)
        Output: (batch, 128)
        Captures higher-level patterns, then averages over time into one vector

    Dropout(0.3) → Linear(128 → NUM_CLASSES)
        Output: (batch, 2)
        Final classification scores (one per class)

Why 1D CNN over LSTM/Transformer:
    - Simpler to implement and debug
    - Faster to train on small datasets
    - Easy to export to ONNX for Pi inference
    - Performs equally well for fixed-window IMU classification

~75K trainable parameters.
"""

import torch.nn as nn

from config import DROPOUT, NUM_CHANNELS, NUM_CLASSES


class StrokeCNN(nn.Module):
    def __init__(self):
        super().__init__()

        # Each block: Conv1d → BatchNorm → ReLU → Pool
        # BatchNorm stabilizes training by normalizing each layer's output.
        # padding="same" equivalent: pad = kernel_size // 2

        self.block1 = nn.Sequential(
            nn.Conv1d(NUM_CHANNELS, 32, kernel_size=5, padding=2),
            nn.BatchNorm1d(32),
            nn.ReLU(),
            nn.MaxPool1d(2),  # halves the time dimension: 40 → 20
        )

        self.block2 = nn.Sequential(
            nn.Conv1d(32, 64, kernel_size=5, padding=2),
            nn.BatchNorm1d(64),
            nn.ReLU(),
            nn.MaxPool1d(2),  # 20 → 10
        )

        self.block3 = nn.Sequential(
            nn.Conv1d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm1d(128),
            nn.ReLU(),
            nn.AdaptiveAvgPool1d(1),  # 10 → 1 (average over remaining time)
        )

        self.classifier = nn.Sequential(
            nn.Dropout(DROPOUT),
            nn.Linear(128, NUM_CLASSES),
        )

    def forward(self, x):
        # x shape: (batch, 6, 40)
        x = self.block1(x)   # (batch, 32, 20)
        x = self.block2(x)   # (batch, 64, 10)
        x = self.block3(x)   # (batch, 128, 1)
        x = x.squeeze(-1)    # (batch, 128) -- remove the trailing time dim
        x = self.classifier(x)  # (batch, NUM_CLASSES)
        return x
