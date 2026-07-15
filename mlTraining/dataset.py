"""
Per-session paired (paddle_window, label_scalar) datasets for regression.

How it works:
  1. A *session* is one paddling recording — a pair of log files
     imuLog_<session>.log (kayak BNO055) and rfLog_<session>.log (paddle).
  2. discover_sessions() scans one or more log directories for sessions
     that have both files.
  3. SessionDataset loads one session: parses paddle + kayak, computes
     regression labels from the kayak signal, aligns paddle samples to
     labels by timestamp, and produces (paddle_window, label) pairs at
     stride=1 (every paddle sample becomes a training example).
  4. Train/val split happens at the *session* level via split_sessions —
     this is important. Adjacent samples within a session share 99% of
     their paddle window with their neighbors, so splitting per-sample
     would leak training data into validation.
  5. ConcatDataset stitches per-session datasets together for the
     DataLoader.

Per-channel z-score normalization is computed from the training sessions
only and saved to checkpoints/norm_stats.json so the Pi can apply the
same preprocessing at inference time.
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd
import torch
from torch.utils.data import ConcatDataset, Dataset

from config import (
    CHANNEL_NAMES,
    MODEL_TYPE_ASSIST,
    MODEL_TYPE_TURN,
    MODEL_TYPES,
    RANDOM_SEED,
    VAL_SPLIT,
    WINDOW_SIZE_ASSIST,
    WINDOW_SIZE_TURN,
)
from idle_gate import GYRO_COLUMNS, compute_idle_mask
from plotters.labels import LabelConfig, compute_labels
from plotters.parsers import parse_imu_log, parse_motor_log, parse_rf_log


_WINDOW_SIZE_FOR = {
    MODEL_TYPE_ASSIST: WINDOW_SIZE_ASSIST,
    MODEL_TYPE_TURN: WINDOW_SIZE_TURN,
}
_LABEL_COLUMN_FOR = {
    MODEL_TYPE_ASSIST: "assist_label",
    MODEL_TYPE_TURN: "turn_label",
}

# How close a paddle sample must be (in time) to its assigned kayak label
# to be considered a valid pair. Gaps larger than this — usually from RF
# packet loss — are dropped instead of paired across a hole.
_ALIGNMENT_TOLERANCE = pd.Timedelta(milliseconds=100)

# The motor mode string in the motor log that corresponds to "motor off,
# fin centered, purely paddle-driven kayak IMU response." This is the
# only mode where the kayak IMU signal cleanly reflects paddle input.
_TRAINING_MODE_NAME = "TRAINING"


def _compute_training_mode_mask(paddle_timestamps: np.ndarray,
                                motor_log_path: Path) -> np.ndarray | None:
    """Return a boolean mask (same length as paddle_timestamps) that is True
    where the paddle sample falls inside a TRAINING-mode motor-log window.

    Returns None if the motor log is missing or has no mode entries — the
    caller decides whether that's a warning-and-continue or a hard error.

    Approach: motor log records the active mode roughly every write cycle
    (a step function). For each paddle timestamp we look up the most
    recent motor-mode entry via merge_asof(direction='backward'). Paddle
    samples that predate the first motor-mode entry get NaN, which
    compares False against 'TRAINING' — the safe default (assume unknown
    mode is not TRAINING).
    """
    if not motor_log_path.exists():
        return None
    motor_df = parse_motor_log(motor_log_path)
    if motor_df.empty or "mode" not in motor_df.columns:
        return None

    mode_df = (motor_df[["timestamp", "mode"]]
               .dropna(subset=["mode"])
               .sort_values("timestamp")
               .reset_index(drop=True))
    if mode_df.empty:
        return None

    paddle_df = pd.DataFrame({"timestamp": paddle_timestamps})
    joined = pd.merge_asof(paddle_df, mode_df, on="timestamp", direction="backward")
    return (joined["mode"] == _TRAINING_MODE_NAME).to_numpy()


@dataclass
class SessionMeta:
    """Lightweight pointer to a session: directory + filename suffix."""
    log_dir: Path
    name: str

    @property
    def imu_log(self) -> Path:
        return self.log_dir / f"imuLog_{self.name}.log"

    @property
    def rf_log(self) -> Path:
        return self.log_dir / f"rfLog_{self.name}.log"

    @property
    def motor_log(self) -> Path:
        return self.log_dir / f"motorLog_{self.name}.log"


def discover_sessions(log_dirs: list[Path]) -> list[SessionMeta]:
    """Return every session in the given directories that has BOTH a kayak
    IMU log and a paddle RF log present. Sessions are matched by filename
    suffix (the timestamp after the source prefix)."""
    sessions: list[SessionMeta] = []
    for log_dir in log_dirs:
        if not log_dir.is_dir():
            continue
        kayak = {p.stem.removeprefix("imuLog_") for p in log_dir.glob("imuLog_*.log")}
        paddle = {p.stem.removeprefix("rfLog_") for p in log_dir.glob("rfLog_*.log")}
        for name in sorted(kayak & paddle):
            sessions.append(SessionMeta(log_dir=log_dir, name=name))
    return sessions


def split_sessions(sessions: list[SessionMeta],
                   val_split: float = VAL_SPLIT,
                   random_seed: int = RANDOM_SEED
                   ) -> tuple[list[SessionMeta], list[SessionMeta]]:
    """Random session-level split. Returns (train_sessions, val_sessions).

    val_split is the fraction of *sessions* — not samples — held out. At
    least one session always goes to val so validation can run at all,
    even if val_split rounds down to zero.
    """
    rng = random.Random(random_seed)
    shuffled = sessions.copy()
    rng.shuffle(shuffled)
    n_val = max(1, int(round(len(shuffled) * val_split)))
    return shuffled[n_val:], shuffled[:n_val]


class SessionDataset(Dataset):
    """One paddling session, paired (paddle_window, label_scalar) samples.

    Eager construction — all sample windows and labels are precomputed.
    __getitem__ then just slices and (optionally) normalizes. For a 30 min
    session at 20 Hz this is under 1 MB of float32 data, so memory is
    a non-issue.

    Args:
        meta: SessionMeta pointing at the session's log files.
        model_type: 'assist' or 'turn' — picks the input window size and
                    which label column becomes the regression target.
        label_config: LabelConfig used to compute labels from kayak IMU.
        norm_stats: Optional (means, stds) tuple, each shape (6,). When
                    set, paddle windows are z-score normalized in
                    __getitem__. Computed from train sessions only.
    """

    def __init__(self,
                 meta: SessionMeta,
                 model_type: str,
                 label_config: LabelConfig,
                 norm_stats: tuple[np.ndarray, np.ndarray] | None = None,
                 filter_training_mode_only: bool = True,
                 filter_idle: bool = True,
                 sample_fraction_range: tuple[float, float] = (0.0, 1.0)):
        """
        Args:
            meta, model_type, label_config, norm_stats — see class docstring.

            filter_training_mode_only:
                When True (default), reads the session's motor log and keeps
                only paddle samples that fall inside TRAINING-mode windows.
                In MANUAL/AUTO modes the motor contributes forward thrust and
                fin steering which contaminate the kayak IMU signal, so those
                samples produce bad labels. If the motor log is missing or
                empty, the filter is silently no-op'd and all samples pass
                through — the caller sees a print-line warning.

            filter_idle:
                When True (default), drops samples where the paddle is idle
                per the idle gate (see config.py "Paddle idle gate"). While
                the paddle is still, the labels are wind drift and coast-down
                deceleration — boat responses the paddle input cannot
                explain, so keeping them just teaches the model noise. The
                runtime gate on the Pi zeroes assist in the same state, so
                the model is never consulted there anyway.

            sample_fraction_range:
                Sub-slice of the surviving valid_indices to actually use,
                given as (lo, hi) fractions of the whole. Defaults to
                (0.0, 1.0) — use everything. Used by the single-session
                temporal split to carve one session into a train and a val
                half, e.g. train=(0.0, 0.8), val=(0.8, 1.0).
        """
        if model_type not in MODEL_TYPES:
            raise ValueError(f"model_type must be one of {MODEL_TYPES}, got {model_type!r}")

        self.meta = meta
        self.model_type = model_type
        self.window_size = _WINDOW_SIZE_FOR[model_type]
        self.label_column = _LABEL_COLUMN_FOR[model_type]

        # ── Parse logs ────────────────────────────────────────────────────
        kayak_df = parse_imu_log(meta.imu_log)
        paddle_df = parse_rf_log(meta.rf_log)
        if kayak_df.empty or paddle_df.empty:
            raise ValueError(f"Session {meta.name} has empty kayak or paddle log.")

        # Drop paddle rows missing any IMU channel (incomplete RF packets).
        paddle_df = paddle_df.dropna(subset=CHANNEL_NAMES).reset_index(drop=True)
        if paddle_df.empty:
            raise ValueError(f"Session {meta.name} has no complete paddle packets.")

        # ── Compute labels from kayak IMU ─────────────────────────────────
        labels_df = compute_labels(kayak_df, label_config)
        labels_df = labels_df[["timestamp", self.label_column]]

        # ── Align paddle samples to labels by timestamp ───────────────────
        # merge_asof matches each paddle row to the nearest-in-time label.
        # Tolerance gates out samples in big RF gaps where no kayak label
        # is within 100 ms of the paddle timestamp.
        aligned = pd.merge_asof(
            paddle_df.sort_values("timestamp").reset_index(drop=True),
            labels_df.sort_values("timestamp").reset_index(drop=True),
            on="timestamp",
            direction="nearest",
            tolerance=_ALIGNMENT_TOLERANCE,
        )

        # ── Extract raw arrays ────────────────────────────────────────────
        self.timestamps = aligned["timestamp"].to_numpy()  # for eval/plot alignment
        self.raw_paddle = aligned[CHANNEL_NAMES].to_numpy(dtype=np.float32)
        self.targets = aligned[self.label_column].to_numpy(dtype=np.float32)

        # ── Find valid sample indices ─────────────────────────────────────
        # A sample at index i is valid if:
        #   (a) the paddle window [i-W+1 .. i] fits within the session
        #   (b) the label at i is not NaN (end-of-session future window
        #       ran off the data, or RF gap dropped the alignment)
        #   (c) optionally, the kayak was in TRAINING mode at that timestamp
        #   (d) optionally, the paddle was not idle at that timestamp
        valid = np.flatnonzero(~np.isnan(self.targets))
        valid = valid[valid >= self.window_size - 1]

        if filter_training_mode_only:
            mask = _compute_training_mode_mask(self.timestamps, meta.motor_log)
            if mask is None:
                print(f"[warn] {meta.log_dir.name}/{meta.name}: motor log missing "
                      f"or empty; skipping TRAINING-mode filter, using all samples.")
            else:
                valid = valid[mask[valid]]
                if valid.size == 0:
                    raise ValueError(
                        f"Session {meta.name} has no TRAINING-mode samples. "
                        f"Was the kayak ever in TRAINING mode during this session?")

        if filter_idle:
            idle = compute_idle_mask(aligned[GYRO_COLUMNS].to_numpy(dtype=float))
            kept = valid[~idle[valid]]
            dropped = valid.size - kept.size
            print(f"[gate] {meta.log_dir.name}/{meta.name}: dropped {dropped} of "
                  f"{valid.size} samples as paddle-idle "
                  f"({100.0 * dropped / max(valid.size, 1):.1f}%)")
            valid = kept
            if valid.size == 0:
                raise ValueError(
                    f"Session {meta.name} has no samples where the paddle "
                    f"was moving. Idle gate dropped everything.")

        # Apply optional sub-slice (used by --single-session-split).
        lo_frac, hi_frac = sample_fraction_range
        if not (0.0 <= lo_frac < hi_frac <= 1.0):
            raise ValueError(
                f"sample_fraction_range must satisfy 0 <= lo < hi <= 1, got {sample_fraction_range}")
        if valid.size and (lo_frac > 0.0 or hi_frac < 1.0):
            lo = int(round(lo_frac * valid.size))
            hi = int(round(hi_frac * valid.size))
            valid = valid[lo:hi]

        self.valid_indices = valid.astype(np.int64)

        self._set_norm_stats(norm_stats)

    def _set_norm_stats(self, norm_stats):
        if norm_stats is None:
            self._channel_means = None
            self._channel_stds = None
        else:
            means, stds = norm_stats
            self._channel_means = np.asarray(means, dtype=np.float32)
            self._channel_stds = np.asarray(stds, dtype=np.float32)

    def apply_norm_stats(self, means: np.ndarray, stds: np.ndarray) -> None:
        """Attach normalization stats after construction (computed from train
        sessions). Safe to call repeatedly."""
        self._set_norm_stats((means, stds))

    @property
    def num_raw_samples(self) -> int:
        """Number of paddle rows loaded for this session (before window
        filtering). Used by compute_paddle_norm_stats."""
        return self.raw_paddle.shape[0]

    def __len__(self) -> int:
        return self.valid_indices.shape[0]

    def __getitem__(self, idx: int):
        end = int(self.valid_indices[idx])
        start = end - self.window_size + 1
        window = self.raw_paddle[start:end + 1]  # (window_size, channels)

        if self._channel_means is not None:
            window = (window - self._channel_means) / self._channel_stds

        # Conv1d expects (channels, time). Copy ensures the tensor owns its
        # memory after the transpose (the slice + .T is a view otherwise).
        window = np.ascontiguousarray(window.T, dtype=np.float32)
        target = self.targets[end]
        return torch.from_numpy(window), torch.tensor(target, dtype=torch.float32)


def compute_paddle_norm_stats(train_datasets: list[SessionDataset]
                              ) -> tuple[np.ndarray, np.ndarray]:
    """Per-channel mean and std across the raw paddle samples in the
    training sessions. Uses raw samples (not windowed) so each sample
    contributes once — windowed data would over-weight the middle of
    every session because each raw sample appears in multiple windows.

    Channels with zero variance get std=1 (no-op normalization) so we
    don't divide by zero.
    """
    all_paddle = np.concatenate([ds.raw_paddle for ds in train_datasets], axis=0)
    means = all_paddle.mean(axis=0).astype(np.float32)
    stds = all_paddle.std(axis=0).astype(np.float32)
    stds[stds == 0] = 1.0
    return means, stds


def _build_session_datasets(session_metas: list[SessionMeta],
                            model_type: str,
                            label_config: LabelConfig,
                            filter_training_mode_only: bool,
                            filter_idle: bool = True,
                            sample_fraction_range: tuple[float, float] = (0.0, 1.0),
                            ) -> tuple[list[SessionDataset], list[SessionMeta]]:
    """Try to build a SessionDataset for each session. Skip any that fail
    with ValueError (session too short, RF parser found no complete
    packets, no valid windows, no TRAINING-mode samples). Returns the
    datasets that succeeded plus the SessionMeta list trimmed to match —
    the caller uses the trimmed list for logging."""
    datasets: list[SessionDataset] = []
    kept: list[SessionMeta] = []
    for meta in session_metas:
        try:
            ds = SessionDataset(
                meta, model_type, label_config,
                filter_training_mode_only=filter_training_mode_only,
                filter_idle=filter_idle,
                sample_fraction_range=sample_fraction_range,
            )
        except ValueError as e:
            print(f"[skip] {meta.log_dir.name}/{meta.name}: {e}")
            continue
        if len(ds) == 0:
            print(f"[skip] {meta.log_dir.name}/{meta.name}: 0 valid samples "
                  f"after filtering.")
            continue
        datasets.append(ds)
        kept.append(meta)
    return datasets, kept


def build_train_val_datasets(log_dirs: list[Path],
                             model_type: str,
                             label_config: LabelConfig | None = None,
                             val_split: float = VAL_SPLIT,
                             random_seed: int = RANDOM_SEED,
                             filter_training_mode_only: bool = True,
                             filter_idle: bool = True,
                             single_session_split: bool = False):
    """End-to-end constructor: discover, split, compute norm stats, return
    concatenated train and val datasets ready for a DataLoader.

    Sessions that fail to parse or produce zero valid samples are skipped
    with a warning. The returned session lists reflect what actually made
    it into training.

    Args:
        log_dirs, model_type, label_config: see SessionDataset.
        val_split: fraction of *sessions* held out for validation in the
            standard multi-session path, OR fraction of *samples* held
            out at the end of the session in single_session_split mode.
        random_seed: for reproducible session shuffling.
        filter_training_mode_only: propagated to every SessionDataset.
            Default True — only paddle samples where the kayak was in
            TRAINING mode are kept.
        filter_idle: propagated to every SessionDataset. Default True —
            paddle-idle samples are dropped (see config.py "Paddle idle
            gate").
        single_session_split: when True, expect exactly one usable session
            and split its samples temporally (first (1 - val_split) for
            train, last val_split for val). This is a smoke-test hack —
            samples from the same session are highly correlated so the
            resulting val metrics do not honestly measure generalization.

    Returns:
        train_dataset:   ConcatDataset over train SessionDatasets
        val_dataset:     ConcatDataset over val SessionDatasets
        channel_means:   shape (6,)
        channel_stds:    shape (6,)
        train_sessions:  list[SessionMeta] (only sessions that survived)
        val_sessions:    list[SessionMeta] (only sessions that survived)
    """
    cfg = label_config if label_config is not None else LabelConfig()
    sessions = discover_sessions(log_dirs)
    if not sessions:
        raise ValueError("No sessions found in the given log directories.")

    if single_session_split:
        # ── Single-session temporal split ─────────────────────────────
        # Find the one session that survives parsing, then slice its
        # samples into a train chunk and a val chunk.
        survivors, survivor_metas = _build_session_datasets(
            sessions, model_type, cfg, filter_training_mode_only, filter_idle,
            sample_fraction_range=(0.0, 1.0),
        )
        if not survivors:
            raise ValueError("No sessions survived parsing for single-session mode. "
                             "Check the skip warnings above.")
        if len(survivors) > 1:
            print(f"[warn] --single-session-split: {len(survivors)} usable sessions "
                  f"available; only using the first ({survivor_metas[0].name}). "
                  f"Drop the flag to use them all with a proper session-level split.")
        chosen_meta = survivor_metas[0]
        train_hi = 1.0 - val_split
        train_datasets, _ = _build_session_datasets(
            [chosen_meta], model_type, cfg, filter_training_mode_only, filter_idle,
            sample_fraction_range=(0.0, train_hi),
        )
        val_datasets, _ = _build_session_datasets(
            [chosen_meta], model_type, cfg, filter_training_mode_only, filter_idle,
            sample_fraction_range=(train_hi, 1.0),
        )
        train_meta = [chosen_meta]
        val_meta = [chosen_meta]

    else:
        # ── Standard session-level split ──────────────────────────────
        if len(sessions) < 2:
            raise ValueError(
                f"Need at least 2 sessions for a train/val split, found {len(sessions)}. "
                f"Pass --single-session-split to temporally split a single session for a smoke test.")

        train_meta, val_meta = split_sessions(sessions, val_split, random_seed)
        train_datasets, train_meta = _build_session_datasets(
            train_meta, model_type, cfg, filter_training_mode_only, filter_idle)
        val_datasets, val_meta = _build_session_datasets(
            val_meta, model_type, cfg, filter_training_mode_only, filter_idle)

    if not train_datasets:
        raise ValueError("No training samples survived filtering. "
                         "Check the skip warnings above.")
    if not val_datasets:
        raise ValueError("No validation samples survived filtering. "
                         "Consider adjusting the val split or adding more data.")

    means, stds = compute_paddle_norm_stats(train_datasets)
    for ds in train_datasets + val_datasets:
        ds.apply_norm_stats(means, stds)

    return (
        ConcatDataset(train_datasets),
        ConcatDataset(val_datasets),
        means,
        stds,
        train_meta,
        val_meta,
    )
