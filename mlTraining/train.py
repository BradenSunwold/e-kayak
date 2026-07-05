"""
Training loop for the paddle regression models.

Trains one model (assist or turn) end-to-end from a corpus of session logs.

Usage:
  python train.py --model assist --log-dirs /path/to/logs1 [more dirs...]
  python train.py --model turn   --log-dirs /path/to/logs1

Per-run artifacts (one set per model type):
  checkpoints/{model}_best.pt          — model weights + training metadata
  checkpoints/{model}_norm_stats.json  — paddle z-score means/stds
  checkpoints/{model}_meta.json        — LabelConfig snapshot + session list

The .pt file is what evaluate.py / export_onnx.py load. The two JSON files
travel with it so training-time context (which sessions were used, what
label config was in effect, what normalization scale to apply) never has
to be reconstructed by hand later.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from config import (
    BATCH_SIZE,
    CHECKPOINT_DIR,
    DIRECTION_ACCURACY_THRESHOLD,
    EPOCHS,
    LEARNING_RATE,
    MODEL_TYPES,
    RANDOM_SEED,
    VAL_SPLIT,
    WEIGHT_DECAY,
)
from dataset import SessionMeta, build_train_val_datasets
from model import build_model, count_parameters, input_window_size
from plotters.labels import LabelConfig
from utils import get_device, save_norm_stats, seed_everything


def _checkpoint_paths(model_type: str) -> dict[str, Path]:
    """Where the three artifacts for a given model type live."""
    return {
        "checkpoint": CHECKPOINT_DIR / f"{model_type}_best.pt",
        "norm_stats": CHECKPOINT_DIR / f"{model_type}_norm_stats.json",
        "metadata":   CHECKPOINT_DIR / f"{model_type}_meta.json",
    }


def train_one_epoch(model, loader, criterion, optimizer, device) -> float:
    """Run one training pass over the data. Returns average per-sample loss."""
    model.train()  # enable dropout + batchnorm training-mode
    total_loss = 0.0
    total_samples = 0

    for windows, targets in loader:
        windows = windows.to(device)
        targets = targets.to(device)

        # Forward — model predicts, loss measures how wrong.
        predictions = model(windows)
        loss = criterion(predictions, targets)

        # Backward — compute gradients, update weights.
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        n = targets.size(0)
        total_loss += loss.item() * n
        total_samples += n

    return total_loss / max(total_samples, 1)


def validate(model, loader, criterion, device) -> dict[str, float]:
    """Run one validation pass. Returns average loss plus regression metrics.

    Metrics returned:
      loss                 — average per-sample loss under `criterion`
      mae                  — mean absolute error, same units as the label
      mae_percent          — MAE as a percentage of mean(|target|). Gives a
                             scale-independent read on error magnitude: 12
                             means "typical error is 12% of a typical label."
                             NaN when mean(|target|) is essentially zero.
      r2                   — 1 - (SS_residual / SS_total). 1 is perfect,
                             0 is "as good as always predicting the mean,"
                             negative is worse than predicting the mean.
      bias                 — mean(prediction - target). Should be ~0. A
                             consistent nonzero value means systematic
                             over/under-prediction.
      direction_accuracy   — fraction of non-neutral samples where
                             sign(prediction) == sign(target). NaN if no
                             samples pass the neutrality threshold.
                             Only meaningful for the turn model — assist is
                             one-sided so its sign is nearly constant.
    """
    model.eval()
    total_loss = 0.0
    total_samples = 0
    all_preds: list[torch.Tensor] = []
    all_targets: list[torch.Tensor] = []

    with torch.no_grad():
        for windows, targets in loader:
            windows = windows.to(device)
            targets = targets.to(device)
            predictions = model(windows)
            loss = criterion(predictions, targets)

            n = targets.size(0)
            total_loss += loss.item() * n
            total_samples += n
            all_preds.append(predictions.cpu())
            all_targets.append(targets.cpu())

    preds = torch.cat(all_preds).numpy()
    targets = torch.cat(all_targets).numpy()

    avg_loss = total_loss / max(total_samples, 1)
    mae = float(np.mean(np.abs(preds - targets)))
    bias = float(np.mean(preds - targets))

    # Scale-independent MAE. Divide by mean absolute target so tiny labels
    # near 0 don't dominate. NaN if the target set is essentially constant
    # at zero — dividing would be meaningless.
    mean_abs_target = float(np.mean(np.abs(targets)))
    mae_percent = 100.0 * mae / mean_abs_target if mean_abs_target > 1e-6 else float("nan")

    # R² needs total variance of the targets; guard against a constant
    # val set where the denominator would be 0.
    ss_res = float(np.sum((targets - preds) ** 2))
    ss_tot = float(np.sum((targets - np.mean(targets)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    non_neutral = np.abs(targets) > DIRECTION_ACCURACY_THRESHOLD
    if non_neutral.sum() > 0:
        direction_accuracy = float(
            np.mean(np.sign(preds[non_neutral]) == np.sign(targets[non_neutral])))
    else:
        direction_accuracy = float("nan")

    return {
        "loss": avg_loss,
        "mae": mae,
        "mae_percent": mae_percent,
        "r2": r2,
        "bias": bias,
        "direction_accuracy": direction_accuracy,
    }


def save_metadata(path: Path,
                  model_type: str,
                  label_config: LabelConfig,
                  train_sessions: list[SessionMeta],
                  val_sessions: list[SessionMeta],
                  split_info: dict) -> None:
    """Snapshot everything needed to make sense of this checkpoint later:
    which sessions it trained on, what label definition it was trained
    against, and how train/val was split. The .pt file has the weights;
    this JSON has the human (and evaluate.py) context.

    split_info fields recorded:
      filter_training_mode_only — whether MANUAL/AUTO samples were filtered out
      single_session_split      — whether the "one session, split temporally" hack was used
      val_split                 — the val fraction used (of sessions or of samples)
    """
    metadata = {
        "model_type": model_type,
        "input_window_size": input_window_size(model_type),
        "label_config": dataclasses.asdict(label_config),
        "split_info": split_info,
        "train_sessions": [
            {"log_dir": str(m.log_dir), "name": m.name} for m in train_sessions
        ],
        "val_sessions": [
            {"log_dir": str(m.log_dir), "name": m.name} for m in val_sessions
        ],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(metadata, f, indent=2)


def _format_metric(value: float, width: int = 8, precision: int = 4) -> str:
    """Consistent per-metric formatting. NaN prints as 'N/A' right-aligned."""
    if np.isnan(value):
        return f"{'N/A':>{width}}"
    return f"{value:>{width}.{precision}f}"


def main():
    parser = argparse.ArgumentParser(description="Train paddle regression model.")
    parser.add_argument("--model", choices=MODEL_TYPES, required=True,
                        help="Which model to train ('assist' or 'turn').")
    parser.add_argument("--log-dirs", nargs="+", type=Path, required=True,
                        help="Directories containing paired imuLog_*.log and rfLog_*.log files.")
    parser.add_argument("--epochs", type=int, default=EPOCHS,
                        help=f"Training epochs (default {EPOCHS}).")
    parser.add_argument("--val-split", type=float, default=VAL_SPLIT,
                        help=f"Fraction of sessions held out for validation (default {VAL_SPLIT}).")
    parser.add_argument("--random-seed", type=int, default=RANDOM_SEED,
                        help="Random seed for reproducible splits and weight init.")
    parser.add_argument("--lr", type=float, default=LEARNING_RATE,
                        help=f"Learning rate for Adam (default {LEARNING_RATE}). "
                             "Try 3e-4 if val loss bounces; 3e-3 if training crawls.")
    parser.add_argument("--weight-decay", type=float, default=WEIGHT_DECAY,
                        help=f"L2 regularization strength (default {WEIGHT_DECAY}). "
                             "Bump to 1e-3 if overfitting; drop to 0 if underfitting.")
    parser.add_argument("--all-modes", action="store_true",
                        help="Include paddle samples from all motor modes. "
                             "Default is TRAINING-mode-only, since motor thrust in "
                             "MANUAL/AUTO contaminates the kayak IMU labels.")
    parser.add_argument("--single-session-split", action="store_true",
                        help="Split one session temporally into train/val chunks "
                             "instead of splitting across sessions. Smoke-test only — "
                             "val metrics are not honest because samples are correlated.")
    args = parser.parse_args()

    seed_everything(args.random_seed)
    device = get_device()
    print(f"Using device: {device}")

    # ── Discover, split, and build datasets ────────────────────────────────
    label_config = LabelConfig()
    print(f"Discovering sessions across {len(args.log_dirs)} log directory(ies)...")

    train_dataset, val_dataset, means, stds, train_sessions, val_sessions = \
        build_train_val_datasets(
            args.log_dirs, args.model, label_config,
            args.val_split, args.random_seed,
            filter_training_mode_only=not args.all_modes,
            single_session_split=args.single_session_split,
        )

    print(f"  Train sessions: {len(train_sessions)} "
          f"({sum(len(ds) for ds in train_dataset.datasets)} samples)")
    for m in train_sessions:
        print(f"    TRAIN:      {m.log_dir.name}/{m.name}")
    print(f"  Validation sessions: {len(val_sessions)} "
          f"({sum(len(ds) for ds in val_dataset.datasets)} samples)")
    for m in val_sessions:
        print(f"    VALIDATION: {m.log_dir.name}/{m.name}")
    print(f"  Paddle normalization: means={means.round(3)}, stds={stds.round(3)}")

    # Label distribution stats — helps interpret raw MAE below.
    val_targets = np.concatenate([ds.targets[ds.valid_indices]
                                  for ds in val_dataset.datasets])
    print(f"  Val label stats: "
          f"mean={val_targets.mean():+.4f}  "
          f"std={val_targets.std():.4f}  "
          f"mean|·|={np.mean(np.abs(val_targets)):.4f}  "
          f"p95|·|={np.percentile(np.abs(val_targets), 95):.4f}  "
          f"(N={val_targets.size})")

    train_loader = DataLoader(
        train_dataset, batch_size=BATCH_SIZE, shuffle=True,
        num_workers=0, pin_memory=False,
    )
    val_loader = DataLoader(
        val_dataset, batch_size=BATCH_SIZE, shuffle=False,
        num_workers=0, pin_memory=False,
    )

    # ── Build model, loss, optimizer ───────────────────────────────────────
    model = build_model(args.model).to(device)
    print(f"Model: {args.model} ({count_parameters(model):,} trainable parameters)")

    # SmoothL1 is Huber loss — behaves like MSE for small errors, like MAE
    # for large ones. More robust to outlier labels than pure MSE.
    criterion = nn.SmoothL1Loss()
    optimizer = torch.optim.Adam(
        model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    print(f"  Optimizer: Adam(lr={args.lr}, weight_decay={args.weight_decay})")

    # ── Write per-model sidecar artifacts up front ────────────────────────
    paths = _checkpoint_paths(args.model)
    CHECKPOINT_DIR.mkdir(parents=True, exist_ok=True)
    save_norm_stats(paths["norm_stats"], means, stds)
    save_metadata(paths["metadata"], args.model, label_config,
                  train_sessions, val_sessions,
                  split_info={
                      "filter_training_mode_only": not args.all_modes,
                      "single_session_split": args.single_session_split,
                      "val_split": args.val_split,
                  })

    # ── Training loop ──────────────────────────────────────────────────────
    best_val_loss = float("inf")
    best_epoch = 0

    print(f"\nTraining {args.model} for {args.epochs} epochs...")
    header = (f"{'Epoch':>5} | {'Train Loss':>10} | "
              f"{'Val Loss':>9} {'MAE':>8} {'MAE%':>7} {'R²':>8} {'Bias':>8} {'Dir Acc':>8} | "
              f"{'Best':>4}")
    print(header)

    for epoch in range(1, args.epochs + 1):
        train_loss = train_one_epoch(model, train_loader, criterion, optimizer, device)
        val = validate(model, val_loader, criterion, device)

        is_best = val["loss"] < best_val_loss
        if is_best:
            best_val_loss = val["loss"]
            best_epoch = epoch
            torch.save({
                "epoch": epoch,
                "model_type": args.model,
                "model_state_dict": model.state_dict(),
                "optimizer_state_dict": optimizer.state_dict(),
                "val_loss": val["loss"],
                "val_mae": val["mae"],
                "val_mae_percent": val["mae_percent"],
                "val_r2": val["r2"],
                "val_bias": val["bias"],
                "val_direction_accuracy": val["direction_accuracy"],
            }, paths["checkpoint"])

        print(f"{epoch:>5} | {train_loss:>10.4f} | "
              f"{val['loss']:>9.4f} "
              f"{_format_metric(val['mae'])} "
              f"{_format_metric(val['mae_percent'], width=7, precision=1)} "
              f"{_format_metric(val['r2'])} "
              f"{_format_metric(val['bias'])} "
              f"{_format_metric(val['direction_accuracy'])} | "
              f"{'*' if is_best else ''}")

    print(f"\nDone! Best validation loss: {best_val_loss:.4f} at epoch {best_epoch}")
    print(f"  Checkpoint:  {paths['checkpoint']}")
    print(f"  Norm stats:  {paths['norm_stats']}")
    print(f"  Metadata:    {paths['metadata']}")


if __name__ == "__main__":
    main()
