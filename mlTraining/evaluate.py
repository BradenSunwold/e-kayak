"""
Evaluate a trained regression model on held-out sessions.

Loads the best checkpoint for a given model type (assist or turn), runs
predictions across the validation sessions the model was trained against
(recorded in the metadata sidecar), and reports:

  - Per-session metrics table (MAE, MAE%, R², bias, direction accuracy).
  - Overall aggregate metrics across all evaluated sessions.
  - Per-session diagnostic figure (time-series overlay, prediction-vs-target
    scatter, residual histogram) displayed via plt.show(). Not saved to
    disk — close the windows to exit.

Honors the split_info recorded during training so the val set here matches
what the model was actually scored against. In particular:
  - filter_training_mode_only  → same motor-mode filter as training
  - single_session_split       → same sample_fraction_range for the val slice

Usage:
  python evaluate.py --model assist
  python evaluate.py --model turn
  python evaluate.py --model assist --log-dirs /path/to/other/logs
        (override the val sessions from metadata — useful for testing on
         genuinely fresh sessions the model has never seen)
"""

from __future__ import annotations

import argparse
import dataclasses
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import torch

from config import (
    BATCH_SIZE,
    CHECKPOINT_DIR,
    DIRECTION_ACCURACY_THRESHOLD,
    MODEL_TYPES,
)
from dataset import SessionDataset, SessionMeta, discover_sessions
from model import build_model
from plotters.labels import LabelConfig
from utils import get_device, load_norm_stats, seed_everything


# Defaults used when the metadata sidecar predates split_info being recorded.
# Match what train.py defaults were at the time this evaluate.py was written.
_DEFAULT_SPLIT_INFO = {
    "filter_training_mode_only": True,
    "single_session_split": False,
    "val_split": 0.2,
}


# ── Small helpers ─────────────────────────────────────────────────────────

def _checkpoint_paths(model_type: str) -> dict[str, Path]:
    return {
        "checkpoint": CHECKPOINT_DIR / f"{model_type}_best.pt",
        "norm_stats": CHECKPOINT_DIR / f"{model_type}_norm_stats.json",
        "metadata":   CHECKPOINT_DIR / f"{model_type}_meta.json",
    }


def _load_meta(meta_path: Path) -> dict:
    with open(meta_path) as f:
        return json.load(f)


def _load_label_config(meta: dict) -> LabelConfig:
    """Reconstruct the LabelConfig that was used during training so we
    generate identical labels at eval time. Anything else would be
    comparing predictions against labels the model never saw."""
    return LabelConfig(**meta["label_config"])


def _load_val_sessions(meta: dict) -> list[SessionMeta]:
    return [SessionMeta(log_dir=Path(m["log_dir"]), name=m["name"])
            for m in meta["val_sessions"]]


def _load_split_info(meta: dict) -> dict:
    """Read the split_info block, falling back to defaults on older
    checkpoints that don't have it recorded."""
    info = dict(_DEFAULT_SPLIT_INFO)
    info.update(meta.get("split_info", {}))
    return info


def _format_metric(value: float, width: int = 8, precision: int = 4) -> str:
    if np.isnan(value):
        return f"{'N/A':>{width}}"
    return f"{value:>{width}.{precision}f}"


# ── Prediction ────────────────────────────────────────────────────────────

def predict_session(model: torch.nn.Module,
                    dataset: SessionDataset,
                    device: torch.device,
                    batch_size: int = BATCH_SIZE
                    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Run the model over every valid sample in a SessionDataset.

    Returns three arrays of equal length (one entry per valid sample):
      timestamps  — pandas datetime for each prediction
      predictions — model output at that sample
      targets     — the label the model was trying to predict
    """
    if len(dataset) == 0:
        return np.array([]), np.array([]), np.array([])

    model.eval()
    preds_out = []
    targets_out = []
    timestamps_out = []

    with torch.no_grad():
        for start in range(0, len(dataset), batch_size):
            end = min(start + batch_size, len(dataset))
            windows, targets = [], []
            for i in range(start, end):
                w, t = dataset[i]
                windows.append(w)
                targets.append(t.item())
            batch = torch.stack(windows).to(device)
            predictions = model(batch).cpu().numpy()
            preds_out.append(predictions)
            targets_out.extend(targets)

            # Sample timestamps: the timestamp of the sample at the "end"
            # index of each window (which is what the label is aligned to).
            for i in range(start, end):
                sample_idx = int(dataset.valid_indices[i])
                timestamps_out.append(dataset.timestamps[sample_idx])

    return (np.array(timestamps_out),
            np.concatenate(preds_out),
            np.array(targets_out, dtype=np.float32))


# ── Metrics ───────────────────────────────────────────────────────────────

def compute_metrics(predictions: np.ndarray, targets: np.ndarray) -> dict[str, float]:
    """Same metric set as train.py's validate(): MAE, MAE%, R², bias, direction acc."""
    if predictions.size == 0:
        return {"mae": float("nan"), "mae_percent": float("nan"),
                "r2": float("nan"), "bias": float("nan"),
                "direction_accuracy": float("nan"), "num_samples": 0}

    mae = float(np.mean(np.abs(predictions - targets)))
    bias = float(np.mean(predictions - targets))

    mean_abs_target = float(np.mean(np.abs(targets)))
    mae_percent = 100.0 * mae / mean_abs_target if mean_abs_target > 1e-6 else float("nan")

    ss_res = float(np.sum((targets - predictions) ** 2))
    ss_tot = float(np.sum((targets - np.mean(targets)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    non_neutral = np.abs(targets) > DIRECTION_ACCURACY_THRESHOLD
    if non_neutral.sum() > 0:
        direction_accuracy = float(
            np.mean(np.sign(predictions[non_neutral]) == np.sign(targets[non_neutral])))
    else:
        direction_accuracy = float("nan")

    return {"mae": mae, "mae_percent": mae_percent, "r2": r2, "bias": bias,
            "direction_accuracy": direction_accuracy,
            "num_samples": int(predictions.size)}


# ── Plot: three-panel diagnostic figure ───────────────────────────────────

def build_session_figure(session_name: str,
                         timestamps: np.ndarray,
                         predictions: np.ndarray,
                         targets: np.ndarray,
                         model_type: str) -> plt.Figure:
    """Three-panel diagnostic figure for one session's predictions.

    Not saved. Caller keeps a reference; plt.show() at end of main pops
    all figures at once.

    Panel layout:
        top row (both columns):     time-series overlay + error trace
        bottom-left:                 scatter of predicted vs. true with y=x
        bottom-right:                residual histogram
    """
    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1], hspace=0.35, wspace=0.25)
    ax_time = fig.add_subplot(gs[0, :])
    ax_scatter = fig.add_subplot(gs[1, 0])
    ax_hist = fig.add_subplot(gs[1, 1])

    # ── Time series overlay ──
    # Reveals *when* in a session the model tracks well vs. fails.
    ax_time.plot(timestamps, targets, color="tab:blue", linewidth=1.0,
                 label=f"{model_type} label")
    ax_time.plot(timestamps, predictions, color="tab:orange", linewidth=1.0,
                 label=f"{model_type} prediction")
    ax_time.plot(timestamps, predictions - targets, color="tab:red", linewidth=0.7,
                 alpha=0.6, label="error (pred - label)")
    ax_time.axhline(0, color="gray", linewidth=0.5)
    ax_time.set_title(f"{model_type} prediction vs. label over time")
    ax_time.set_xlabel("time")
    ax_time.grid(True, alpha=0.3)
    ax_time.legend(loc="upper right", fontsize=8)

    # ── Predicted vs. true scatter ──
    # Points on the y=x diagonal are perfect predictions. Common failures:
    #   horizontal band around 0     → regression to the mean
    #   flat top / bottom of range   → prediction saturation
    #   whole cloud shifted off y=x  → systematic bias
    ax_scatter.scatter(targets, predictions, s=3, alpha=0.35, color="tab:orange")
    lo = float(min(targets.min(), predictions.min()))
    hi = float(max(targets.max(), predictions.max()))
    ax_scatter.plot([lo, hi], [lo, hi], color="gray", linewidth=0.8, linestyle="--",
                    label="y = x (perfect)")
    ax_scatter.set_xlabel(f"{model_type} label (target)")
    ax_scatter.set_ylabel(f"{model_type} prediction")
    ax_scatter.set_title("Predicted vs. true — points on y=x are perfect")
    ax_scatter.grid(True, alpha=0.3)
    ax_scatter.legend(loc="upper left", fontsize=8)
    ax_scatter.set_aspect("equal", adjustable="datalim")

    # ── Residual histogram ──
    # Ideal: symmetric bell centered on 0. Off-center peak = the bias
    # metric visualized. Skewed or bimodal shape = asymmetric error modes.
    residuals = predictions - targets
    ax_hist.hist(residuals, bins=60, color="tab:red", alpha=0.7)
    ax_hist.axvline(0, color="gray", linewidth=0.6)
    ax_hist.axvline(float(residuals.mean()), color="tab:blue", linewidth=1.0,
                    linestyle="--", label=f"mean = {residuals.mean():+.4f}")
    ax_hist.set_xlabel("residual (prediction - target)")
    ax_hist.set_ylabel("count")
    ax_hist.set_title("Residual distribution — ideally centered on 0")
    ax_hist.grid(True, alpha=0.3)
    ax_hist.legend(loc="upper right", fontsize=8)

    fig.suptitle(f"Session: {session_name}  ({len(predictions)} samples, {model_type} model)",
                 fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    return fig


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Evaluate a trained regression model on validation sessions.")
    parser.add_argument("--model", choices=MODEL_TYPES, required=True,
                        help="Which trained model to evaluate.")
    parser.add_argument("--log-dirs", nargs="+", type=Path, default=None,
                        help="Override the val sessions from metadata by "
                             "discovering sessions in these directories.")
    args = parser.parse_args()

    seed_everything()
    device = get_device()
    paths = _checkpoint_paths(args.model)

    # Sanity check: the three sidecar files must exist.
    for name, path in paths.items():
        if not path.exists():
            raise SystemExit(
                f"{args.model} {name} not found: {path}\n"
                f"Have you trained this model yet? Run:\n"
                f"  python train.py --model {args.model} --log-dirs <log_dir>")

    # ── Load training-time context ─────────────────────────────────────────
    meta = _load_meta(paths["metadata"])
    label_config = _load_label_config(meta)
    split_info = _load_split_info(meta)
    means, stds = load_norm_stats(paths["norm_stats"])

    # Session list + sample-range slice must match what training used
    # for the val set, otherwise we'd be evaluating on samples the model
    # actually saw during training.
    if args.log_dirs:
        sessions = discover_sessions(args.log_dirs)
        # Fresh sessions from --log-dirs are truly held-out — use everything.
        eval_fraction_range = (0.0, 1.0)
        print(f"Overriding val sessions with {len(sessions)} session(s) from "
              f"{[str(d) for d in args.log_dirs]}")
    else:
        sessions = _load_val_sessions(meta)
        if split_info["single_session_split"]:
            # Training kept the last val_split fraction of samples as val.
            eval_fraction_range = (1.0 - split_info["val_split"], 1.0)
        else:
            eval_fraction_range = (0.0, 1.0)
        print(f"Using {len(sessions)} val session(s) from {paths['metadata'].name}")

    print(f"Split info: {split_info}")
    if eval_fraction_range != (0.0, 1.0):
        print(f"Evaluating on sample fraction range {eval_fraction_range} of each session "
              f"(single-session-split slice).")

    if not sessions:
        raise SystemExit("No sessions to evaluate.")

    # ── Load checkpoint and reconstruct model ──────────────────────────────
    checkpoint = torch.load(paths["checkpoint"], map_location=device, weights_only=True)
    model = build_model(args.model).to(device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    print(f"Loaded checkpoint from epoch {checkpoint['epoch']} "
          f"(validation_loss={checkpoint['val_loss']:.4f})")
    print(f"Label config: {dataclasses.asdict(label_config)}")

    # ── Predict per session, aggregate ─────────────────────────────────────
    print(f"\n{'Session':<50} | {'Samples':>7} {'MAE':>8} {'MAE%':>7} {'R²':>8} {'Bias':>8} {'DirAcc':>8}")
    print("-" * 108)

    all_predictions: list[np.ndarray] = []
    all_targets: list[np.ndarray] = []
    figures: list[plt.Figure] = []

    for meta_session in sessions:
        try:
            dataset = SessionDataset(
                meta_session, args.model, label_config,
                norm_stats=(means, stds),
                filter_training_mode_only=split_info["filter_training_mode_only"],
                sample_fraction_range=eval_fraction_range,
            )
        except Exception as e:
            print(f"[skip] {meta_session.log_dir.name}/{meta_session.name}: {e}")
            continue

        timestamps, predictions, targets = predict_session(model, dataset, device)
        session_metrics = compute_metrics(predictions, targets)
        label = f"{meta_session.log_dir.name}/{meta_session.name}"
        print(f"{label:<50} | "
              f"{session_metrics['num_samples']:>7d} "
              f"{_format_metric(session_metrics['mae'])} "
              f"{_format_metric(session_metrics['mae_percent'], width=7, precision=1)} "
              f"{_format_metric(session_metrics['r2'])} "
              f"{_format_metric(session_metrics['bias'])} "
              f"{_format_metric(session_metrics['direction_accuracy'])}")

        if predictions.size > 0:
            fig = build_session_figure(
                meta_session.name, timestamps, predictions, targets, args.model)
            figures.append(fig)
            all_predictions.append(predictions)
            all_targets.append(targets)

    if not all_predictions:
        raise SystemExit("No usable predictions across evaluated sessions.")

    all_predictions_arr = np.concatenate(all_predictions)
    all_targets_arr = np.concatenate(all_targets)
    aggregate = compute_metrics(all_predictions_arr, all_targets_arr)

    print("-" * 108)
    print(f"{'AGGREGATE (all evaluated sessions)':<50} | "
          f"{aggregate['num_samples']:>7d} "
          f"{_format_metric(aggregate['mae'])} "
          f"{_format_metric(aggregate['mae_percent'], width=7, precision=1)} "
          f"{_format_metric(aggregate['r2'])} "
          f"{_format_metric(aggregate['bias'])} "
          f"{_format_metric(aggregate['direction_accuracy'])}")

    if figures:
        print(f"\nOpening {len(figures)} diagnostic figure(s). Close all windows to exit.")
        plt.show()


if __name__ == "__main__":
    main()
