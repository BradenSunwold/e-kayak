"""
Compute label normalization scales across a corpus of sessions.

For each session in the supplied log directories, parses the kayak IMU log,
computes the regression labels with norm_scale=1.0 (so we see raw magnitudes,
not already-normalized values), and aggregates absolute label values across
the entire corpus. Reports the 95th percentile of |assist_label| and
|turn_label| — the values to paste into config.py as the divisors.

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

from plotters.labels import LabelConfig, compute_labels
from plotters.parsers import parse_imu_log


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
            assist = labels_df["assist_label"].dropna().to_numpy(dtype=float)
            turn = labels_df["turn_label"].dropna().to_numpy(dtype=float)

            if assist.size == 0:
                print(f"[skip] {imu_path.name}: all labels NaN")
                continue

            assist_p95 = float(np.percentile(np.abs(assist), 95))
            turn_p95 = float(np.percentile(np.abs(turn), 95))
            print(f"[ok]  {log_dir.name}/{session}: "
                  f"{assist.size:>6} samples, "
                  f"|assist| p95={assist_p95:7.3f}, |turn| p95={turn_p95:7.3f}")

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


if __name__ == "__main__":
    main()
