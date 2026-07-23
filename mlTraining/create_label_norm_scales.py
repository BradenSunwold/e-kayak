"""
Compute label normalization scales across a corpus of sessions.

For each session in the supplied log directories, parses the kayak IMU log,
computes the regression labels with norm_scale=1.0 (so we see raw magnitudes,
not already-normalized values), and aggregates absolute label values across
the entire corpus. Reports the 95th percentile of |assist_label| and
|turn_label| — the values to paste into config.py as the divisors.

Labels are idle-gated before the percentile: samples where the paddle was
idle (see config.py "Paddle idle gate") are excluded, matching the filtering
dataset.py applies during training. Sessions without an rfLog fall back to
using all samples, with a note in the per-session output line.

Why the 95th percentile and not max:
  Max is dominated by single outliers (one wave-funny stroke spikes the
  divisor and crams all normal labels into a sliver of the [-1, 1] range).
  The 95th percentile picks a value that 95% of labels fall under, so
  typical paddling lands roughly in [-1, 1] and the rare strong strokes
  exceed it as genuine outliers — which is what the model should see.

Usage:
  python compute_norm_scales.py /path/to/logs1 [/path/to/logs2 ...]

Sessions are discovered automatically by globbing imuLog_*.log files in
each directory. A session is skipped if the kayak IMU log can't be parsed.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

from config import LABEL_TURN_NORM_SCALE
from dataset import _ALIGNMENT_TOLERANCE
from idle_gate import GYRO_COLUMNS, compute_idle_mask
from plotters.labels import LabelConfig, compute_labels
from plotters.parsers import parse_imu_log, parse_rf_log


def find_sessions(log_dir: Path) -> list[str]:
    """Return session suffixes (e.g. '2026-06-27_17:41.35') for every imuLog
    file in the directory. Sorted for deterministic iteration order."""
    return [
        path.stem.removeprefix("imuLog_")
        for path in sorted(log_dir.glob("imuLog_*.log"))
    ]


def main():
    ap = argparse.ArgumentParser(
        description="Compute label normalization scales across a session corpus.")
    ap.add_argument("log_dirs", nargs="+", type=Path,
                    help="Directories containing imuLog_*.log files.")
    args = ap.parse_args()

    # Force unnormalized labels so we measure the raw magnitudes — otherwise
    # we'd be computing the percentile of already-normalized values, which
    # would silently double-normalize on the next config update.
    cfg = LabelConfig(assist_norm_scale=1.0, turn_norm_scale=1.0)

    all_assist = []
    all_turn = []
    total_sessions = 0
    total_samples = 0

    for log_dir in args.log_dirs:
        if not log_dir.is_dir():
            print(f"[warn] not a directory, skipping: {log_dir}")
            continue
        sessions = find_sessions(log_dir)
        if not sessions:
            print(f"[warn] no imuLog files in {log_dir}")
            continue

        for session in sessions:
            imu_path = log_dir / f"imuLog_{session}.log"
            try:
                kayak_df = parse_imu_log(imu_path)
            except Exception as e:
                print(f"[skip] {imu_path.name}: parse failed ({e})")
                continue
            if len(kayak_df) < 2:
                print(f"[skip] {imu_path.name}: too few samples ({len(kayak_df)})")
                continue

            labels_df = compute_labels(kayak_df, cfg)
            labels_df = labels_df.sort_values("timestamp").reset_index(drop=True)

            # Idle-gate the labels the same way dataset.py gates training
            # samples, so the percentiles describe the distribution the
            # model actually trains on. Kayak samples with no paddle match
            # within the alignment tolerance drop too — they never reach
            # training either.
            gate_note = "no rf log, idle gate skipped"
            rf_path = log_dir / f"rfLog_{session}.log"
            if rf_path.exists():
                paddle_df = (parse_rf_log(rf_path)
                             .dropna(subset=GYRO_COLUMNS)
                             .reset_index(drop=True))
                if not paddle_df.empty:
                    idle_df = pd.DataFrame({
                        "timestamp": paddle_df["timestamp"],
                        "idle": compute_idle_mask(
                            paddle_df[GYRO_COLUMNS].to_numpy(dtype=float)),
                    })
                    merged = pd.merge_asof(
                        labels_df[["timestamp"]], idle_df, on="timestamp",
                        direction="nearest", tolerance=_ALIGNMENT_TOLERANCE)
                    keep = (merged["idle"] == False).to_numpy()  # noqa: E712 — NaN (no match) drops too
                    gate_note = (f"idle gate dropped "
                                 f"{len(labels_df) - int(keep.sum())}/{len(labels_df)}")
                    labels_df = labels_df.loc[keep]

            assist = labels_df["assist_label"].dropna().to_numpy(dtype=float)
            turn = labels_df["turn_label"].dropna().to_numpy(dtype=float)

            if assist.size == 0:
                print(f"[skip] {imu_path.name}: all labels NaN")
                continue

            assist_p95 = float(np.percentile(np.abs(assist), 95))
            turn_p95 = float(np.percentile(np.abs(turn), 95))
            print(f"[ok]  {log_dir.name}/{session}: "
                  f"{assist.size:>6} samples, "
                  f"|assist| p95={assist_p95:7.3f}, |turn| p95={turn_p95:7.3f} "
                  f"({gate_note})")

            all_assist.append(assist)
            all_turn.append(turn)
            total_sessions += 1
            total_samples += assist.size

    if total_samples == 0:
        print("\nNo usable label samples found. Check log directories.")
        sys.exit(1)

    assist_concat = np.concatenate(all_assist)
    turn_concat = np.concatenate(all_turn)

    overall_assist_p95 = float(np.percentile(np.abs(assist_concat), 95))
    overall_turn_p95 = float(np.percentile(np.abs(turn_concat), 95))

    # Distribution context — handy for sanity-checking the chosen percentile.
    def _summary(name, arr):
        absvals = np.abs(arr)
        return (f"  |{name}|  "
                f"p50={np.percentile(absvals, 50):7.3f}  "
                f"p90={np.percentile(absvals, 90):7.3f}  "
                f"p95={np.percentile(absvals, 95):7.3f}  "
                f"p99={np.percentile(absvals, 99):7.3f}  "
                f"max={absvals.max():7.3f}")

    print(f"\nAcross {total_samples} samples from {total_sessions} session(s):")
    print(_summary("assist_label", assist_concat))
    print(_summary("turn_label  ", turn_concat))

    print("\nSuggested config.py update:")
    print(f"  LABEL_ASSIST_NORM_SCALE = {overall_assist_p95:.3f}")
    print(f"  LABEL_TURN_NORM_SCALE   = {overall_turn_p95:.3f}")

    # The turn deadband is specified in physical degrees
    # (LABEL_BLEND_BOAT_TURN_*_DEG) and converts to normalized units off the
    # turn norm scale automatically, so a scale change alone keeps it fixed in
    # degrees — no manual retune needed for renormalization. But if THIS run's
    # numbers moved because the corpus itself changed (new conditions, more
    # turning), the straight-vs-turning split those degrees encode may have
    # shifted too — re-run analyze_turn_distribution.py to reassess them.
    if abs(overall_turn_p95 - LABEL_TURN_NORM_SCALE) > 1e-3:
        print(f"\n[note] turn norm scale would change "
              f"{LABEL_TURN_NORM_SCALE:.3f} -> {overall_turn_p95:.3f}. The "
              f"deadband stays fixed in degrees automatically, but if the "
              f"corpus changed, re-run analyze_turn_distribution.py to "
              f"reassess LABEL_BLEND_BOAT_TURN_*_DEG.")


if __name__ == "__main__":
    main()
