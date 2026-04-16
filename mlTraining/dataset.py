"""
PyTorch Dataset that loads labeled CSVs and produces windowed IMU tensors.

How it works:
1. Scan DATA_DIR for CSV files matching known label prefixes (e.g. stroke_*.csv)
2. Load each CSV into a numpy array of shape (num_samples, 6)
3. Slide a window across each file to produce fixed-size chunks
4. Apply z-score normalization (computed from training set only)

The label comes from the filename prefix -- files starting with "stroke_" get
label 1, files starting with "no_stroke_" get label 0 (per LABEL_MAP in config).
"""

import csv

import numpy as np
import torch
from torch.utils.data import Dataset

from config import (
    CHANNEL_NAMES,
    DATA_DIR,
    LABEL_MAP,
    WINDOW_SIZE,
    WINDOW_STRIDE,
)


def discover_csv_files():
    """
    Find all labeled CSV files in DATA_DIR.

    Returns list of (path, label_int) tuples. The label is determined by
    matching the filename prefix against LABEL_MAP keys.
    E.g. "stroke_session1.csv" matches label "stroke" -> 1
    """
    files = []
    for csv_path in sorted(DATA_DIR.glob("*.csv")):
        name = csv_path.stem  # e.g. "stroke_session1"
        for label_name, label_int in LABEL_MAP.items():
            if name.startswith(label_name):
                files.append((csv_path, label_int))
                break
    return files


def load_csv(csv_path):
    """
    Load a CSV file into a numpy array of shape (num_samples, 6).

    Skips the header row and timestamp column -- only loads the 6 IMU channels.
    Uses Python's csv module which handles quoted fields correctly.
    """
    rows = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append([float(row[ch]) for ch in CHANNEL_NAMES])
    return np.array(rows)  # (num_samples, 6)


def extract_windows(data, window_size=WINDOW_SIZE, stride=WINDOW_STRIDE):
    """
    Slide a window across the data array and return stacked windows.

    Args:
        data: numpy array of shape (num_samples, num_channels)
        window_size: number of samples per window
        stride: samples between window starts

    Returns:
        numpy array of shape (num_windows, num_channels, window_size)
        Channels on axis 1, time on axis 2 -- the format Conv1d expects.
    """
    num_samples = data.shape[0]
    windows = []
    for start in range(0, num_samples - window_size + 1, stride):
        window = data[start : start + window_size]  # (window_size, channels)
        windows.append(window.T)  # transpose to (channels, window_size)
    return np.array(windows)  # (num_windows, channels, window_size)


def compute_norm_stats(file_list):
    """
    Compute per-channel mean and std from a list of (csv_path, label) tuples.

    Only call this on the TRAINING files -- never include validation files,
    or the model gets a sneak peek at test data (data leakage).

    Returns:
        means: numpy array of shape (6,)
        stds: numpy array of shape (6,)
    """
    all_data = []
    for csv_path, _ in file_list:
        all_data.append(load_csv(csv_path))
    combined = np.concatenate(all_data, axis=0)  # (total_samples, 6)
    means = combined.mean(axis=0)
    stds = combined.std(axis=0)
    # Prevent division by zero if a channel is constant
    stds[stds == 0] = 1.0
    return means, stds


class StrokeDataset(Dataset):
    """
    PyTorch Dataset for windowed IMU data.

    Each item is (tensor, label):
        tensor: float32, shape (6, 40) -- 6 IMU channels x 40 time steps
        label:  int, 0 or 1

    Args:
        file_list: list of (csv_path, label_int) from discover_csv_files()
        means: per-channel means for z-score normalization (from training set)
        stds: per-channel stds for z-score normalization (from training set)
    """

    def __init__(self, file_list, means, stds):
        self.windows = []  # list of numpy arrays, each (channels, window_size)
        self.labels = []   # list of ints

        for csv_path, label in file_list:
            data = load_csv(csv_path)  # (num_samples, 6)

            # Normalize: subtract mean, divide by std (per channel)
            data = (data - means) / stds

            windows = extract_windows(data)  # (num_windows, 6, 40)
            for w in windows:
                self.windows.append(w)
                self.labels.append(label)

        self.windows = np.array(self.windows, dtype=np.float32)
        self.labels = np.array(self.labels, dtype=np.int64)

    def __len__(self):
        return len(self.labels)

    def __getitem__(self, idx):
        return torch.from_numpy(self.windows[idx]), self.labels[idx]
