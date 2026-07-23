"""
Characterize the turn-label distribution across a session corpus, to pick a
turn-magnitude deadband/blend range.

The turn label is the future-window mean of kayak yaw rate (gyro_z). Because
it is a *mean over a fixed window*, it has a clean physical reading:

    net heading change over the window = mean_yaw_rate * window_seconds

So every turn-label magnitude maps to "degrees the boat rotated across the
2 s window" — the intuitive way to ask "is this a real turn or just the
side-to-side wobble every forward stroke induces?"

What this script reports, over active-paddling samples only (idle-gated the
same way training is, so the distribution matches what the turn model sees):

  1. Percentiles of |turn_label| in normalized units AND physical units
     (deg/s of yaw, and deg of heading change per window).
  2. For a sweep of candidate deadband thresholds: the fraction of samples
     each would zero out, so you can see where the "wobble core" ends and the
     "real turn" tail begins.
  3. A histogram PNG (linear + log-count) so the core-vs-tail shape is visible.

Usage:
  python analyze_turn_distribution.py /path/to/logs [/path/to/logs2 ...]
  python analyze_turn_distribution.py /path/to/logs --plot turn_dist.png

Idle-gating and session discovery mirror create_label_norm_scales.py exactly.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

from config import LABEL_TURN_NORM_SCALE, LABEL_TURN_WINDOW_MS
from dataset import _ALIGNMENT_TOLERANCE
from idle_gate import GYRO_COLUMNS, compute_idle_mask
from plotters.labels import LabelConfig, compute_labels
from plotters.parsers import parse_imu_log, parse_rf_log

# Physical conversion constants, derived once from config.
_WINDOW_SECONDS = LABEL_TURN_WINDOW_MS / 1000.0
_RAD_TO_DEG = 180.0 / np.pi


def _norm_to_deg_per_window(norm_value: float) -> float:
    """Convert a normalized turn-label magnitude into the boat's net heading
    change (degrees) over the label window. norm -> rad/s -> rad over window
    -> deg. This is the physical meaning of the deadband threshold."""
    yaw_rate_rad_s = norm_value * LABEL_TURN_NORM_SCALE
    heading_change_rad = yaw_rate_rad_s * _WINDOW_SECONDS
    return heading_change_rad * _RAD_TO_DEG


def _norm_to_deg_per_sec(norm_value: float) -> float:
    """Convert a normalized turn-label magnitude into yaw rate in deg/s."""
    return norm_value * LABEL_TURN_NORM_SCALE * _RAD_TO_DEG


def find_sessions(log_dir: Path) -> list[str]:
    """Session suffixes for every imuLog file in the directory, sorted."""
    return [
        path.stem.removeprefix("imuLog_")
        for path in sorted(log_dir.glob("imuLog_*.log"))
    ]


def collect_turn_labels(log_dirs: list[Path]) -> np.ndarray:
    """Return all active-paddling turn labels (normalized units) across the
    corpus, idle-gated exactly as create_label_norm_scales.py does so the
    distribution matches the training target."""
    # turn_norm_scale from config (not 1.0) — we want the normalized label the
    # model actually regresses, since the deadband will be applied in those units.
    cfg = LabelConfig()

    all_turn = []
    total_sessions = 0

    for log_dir in log_dirs:
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

            # Idle-gate to active paddling only (same as training).
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
                    keep = (merged["idle"] == False).to_numpy()  # noqa: E712
                    gate_note = (f"idle gate dropped "
                                 f"{len(labels_df) - int(keep.sum())}/{len(labels_df)}")
                    labels_df = labels_df.loc[keep]

            turn = labels_df["turn_label"].dropna().to_numpy(dtype=float)
            if turn.size == 0:
                print(f"[skip] {imu_path.name}: all turn labels NaN")
                continue

            p50 = float(np.percentile(np.abs(turn), 50))
            p95 = float(np.percentile(np.abs(turn), 95))
            print(f"[ok]  {log_dir.name}/{session}: {turn.size:>6} samples, "
                  f"|turn| p50={p50:6.3f} p95={p95:6.3f} "
                  f"({_norm_to_deg_per_window(p50):4.1f}deg / "
                  f"{_norm_to_deg_per_window(p95):4.1f}deg per window)  [{gate_note}]")

            all_turn.append(turn)
            total_sessions += 1

    if not all_turn:
        print("\nNo usable turn-label samples found. Check log directories.")
        sys.exit(1)

    print(f"\nCollected {sum(a.size for a in all_turn)} samples "
          f"from {total_sessions} session(s).")
    return np.concatenate(all_turn)


def report_distribution(turn: np.ndarray) -> None:
    """Print percentiles and a candidate-threshold sweep."""
    absv = np.abs(turn)

    print("\n" + "=" * 72)
    print("DISTRIBUTION OF |turn_label|  (active paddling only)")
    print("=" * 72)
    print(f"{'pct':>5} | {'norm':>8} | {'deg/s yaw':>10} | {'deg / 2s window':>16}")
    print("-" * 50)
    for pct in (10, 25, 50, 60, 70, 75, 80, 90, 95, 99):
        v = float(np.percentile(absv, pct))
        print(f"{pct:>4}% | {v:>8.3f} | {_norm_to_deg_per_sec(v):>10.2f} | "
              f"{_norm_to_deg_per_window(v):>16.2f}")

    print(f"\nmean|turn|={absv.mean():.3f}  "
          f"({_norm_to_deg_per_window(absv.mean()):.1f} deg/window)   "
          f"max={absv.max():.3f}  "
          f"({_norm_to_deg_per_window(absv.max()):.1f} deg/window)")

    # Candidate-threshold sweep: for each physical heading-change threshold,
    # what fraction of active-paddling samples fall below it (would be zeroed
    # by a deadband LOW at that value)?
    print("\n" + "=" * 72)
    print("CANDIDATE DEADBAND SWEEP")
    print("  'below' = fraction of active-paddling samples this LOW would zero")
    print("=" * 72)
    print(f"{'deg/window':>11} | {'norm thresh':>12} | {'% below':>8}")
    print("-" * 38)
    for deg in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0):
        norm_thresh = deg / _norm_to_deg_per_window(1.0)
        frac_below = float((absv < norm_thresh).mean())
        print(f"{deg:>9.1f}   | {norm_thresh:>12.3f} | {100 * frac_below:>7.1f}%")


def save_histogram(turn: np.ndarray, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    absv = np.abs(turn)
    # Clip the extreme tail so the wobble core is visible; annotate the clip.
    hi = float(np.percentile(absv, 99.5))
    fig, (ax_lin, ax_log) = plt.subplots(1, 2, figsize=(13, 5))

    for ax in (ax_lin, ax_log):
        ax.hist(absv, bins=120, range=(0, hi), color="tab:blue", alpha=0.8)
        ax.set_xlabel("|turn_label| (normalized)")
        # Secondary axis in physical deg/window.
        secax = ax.secondary_xaxis(
            "top",
            functions=(_norm_to_deg_per_window,
                       lambda d: d / _norm_to_deg_per_window(1.0)))
        secax.set_xlabel("net heading change over 2 s window (deg)")
        for pct, style in ((50, ":"), (75, "--"), (90, "-.")):
            v = float(np.percentile(absv, pct))
            ax.axvline(v, color="tab:red", linestyle=style, linewidth=1.0,
                       label=f"p{pct} = {v:.2f} ({_norm_to_deg_per_window(v):.1f} deg)")
        ax.legend(fontsize=8)

    ax_lin.set_ylabel("sample count")
    ax_lin.set_title("Turn-label distribution (linear count)")
    ax_log.set_yscale("log")
    ax_log.set_ylabel("sample count (log)")
    ax_log.set_title("Same, log count — reveals the real-turn tail")

    fig.suptitle("Turn-label magnitude during active paddling "
                 "(wobble core near 0, real turns in the tail)", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"\n[ok] histogram written to {path}")


def main():
    ap = argparse.ArgumentParser(
        description="Characterize the turn-label distribution to pick a deadband.")
    ap.add_argument("log_dirs", nargs="+", type=Path,
                    help="Directories containing imuLog_*.log files.")
    ap.add_argument("--plot", type=Path, default=None,
                    help="Optional path to write a histogram PNG.")
    args = ap.parse_args()

    print(f"turn_norm_scale = {LABEL_TURN_NORM_SCALE}  "
          f"window = {_WINDOW_SECONDS:.1f}s  "
          f"(norm 1.0 -> {_norm_to_deg_per_window(1.0):.1f} deg heading change / window)")

    turn = collect_turn_labels(args.log_dirs)
    report_distribution(turn)
    if args.plot is not None:
        save_histogram(turn, args.plot)


if __name__ == "__main__":
    main()
