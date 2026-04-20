"""
Training loop for the paddle stroke classifier.

What this script does:
1. Discovers all labeled CSVs in data/
2. Splits them into train/val sets BY FILE (not by window -- prevents leakage)
3. Computes normalization stats from training files only, saves to checkpoints/
4. Creates DataLoaders for train and val sets
5. Trains the CNN, saving the best model (lowest val loss) to checkpoints/

Usage:
    python train.py

MPS notes (Apple Silicon):
    - num_workers=0: MPS doesn't support multiprocess data loading
    - pin_memory=False: only useful for CUDA, not MPS
"""

import random

import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from config import (
    BATCH_SIZE,
    CHECKPOINT_DIR,
    EPOCHS,
    LABEL_MAP,
    LEARNING_RATE,
    RANDOM_SEED,
    VAL_SPLIT,
    WEIGHT_DECAY,
)
from dataset import StrokeDataset, compute_norm_stats, discover_csv_files
from model import StrokeCNN
from utils import get_device, save_norm_stats, seed_everything


def split_files(file_list, val_split=VAL_SPLIT):
    """
    Split file list into train and val sets, stratified by class.

    Stratification = each class is represented in both splits proportionally.
    With only a handful of files per class, an unstratified random split can
    silently put every file of a class on one side -- which breaks validation
    (you can't measure accuracy on a class with zero samples in val).

    We also split by FILE, not by window. If we split by window, adjacent
    windows from the same file would end up in both train and val (because
    of the 75% overlap), and the model would essentially memorize the
    validation set.
    """
    random.seed(RANDOM_SEED)

    by_label = {}
    for path, label in file_list:
        by_label.setdefault(label, []).append((path, label))

    train_files, val_files = [], []
    for label, files in by_label.items():
        shuffled = files.copy()
        random.shuffle(shuffled)
        n_val = max(1, int(len(shuffled) * val_split))
        val_files.extend(shuffled[:n_val])
        train_files.extend(shuffled[n_val:])

    return train_files, val_files


def train_one_epoch(model, loader, criterion, optimizer, device):
    """Run one training epoch. Returns average loss."""
    model.train()  # enable dropout and batchnorm training behavior
    total_loss = 0.0
    total_correct = 0
    total_samples = 0

    for windows, labels in loader:
        windows = windows.to(device)  # (batch, 6, 40)
        labels = labels.to(device)    # (batch,)

        # Forward pass: model predicts, loss measures how wrong it is
        predictions = model(windows)          # (batch, 2)
        loss = criterion(predictions, labels)

        # Backward pass: compute gradients, update weights
        optimizer.zero_grad()  # clear gradients from previous batch
        loss.backward()        # compute new gradients
        optimizer.step()       # adjust weights using gradients

        # Track metrics
        total_loss += loss.item() * labels.size(0)
        total_correct += (predictions.argmax(dim=1) == labels).sum().item()
        total_samples += labels.size(0)

    avg_loss = total_loss / total_samples
    accuracy = total_correct / total_samples
    return avg_loss, accuracy


def validate(model, loader, criterion, device):
    """
    Run validation. Returns (avg_loss, overall_accuracy, per_class_accuracy).

    per_class_accuracy is a dict mapping class_int -> accuracy for that class.
    A class missing from the dict means it had zero samples in the val set.
    """
    model.eval()  # disable dropout, batchnorm uses running stats
    total_loss = 0.0
    total_correct = 0
    total_samples = 0
    class_correct = {}  # class_int -> count of correct predictions
    class_total = {}    # class_int -> total count of samples

    with torch.no_grad():  # no gradient computation needed for validation
        for windows, labels in loader:
            windows = windows.to(device)
            labels = labels.to(device)

            predictions = model(windows)
            loss = criterion(predictions, labels)

            predicted_labels = predictions.argmax(dim=1)
            total_loss += loss.item() * labels.size(0)
            total_correct += (predicted_labels == labels).sum().item()
            total_samples += labels.size(0)

            # Tally correct/total per class for this batch
            for label_tensor in torch.unique(labels):
                label = label_tensor.item()
                mask = labels == label_tensor
                class_correct[label] = class_correct.get(label, 0) + \
                    (predicted_labels[mask] == label_tensor).sum().item()
                class_total[label] = class_total.get(label, 0) + mask.sum().item()

    avg_loss = total_loss / total_samples
    accuracy = total_correct / total_samples
    per_class_acc = {
        label: class_correct[label] / class_total[label]
        for label in class_total
    }
    return avg_loss, accuracy, per_class_acc


def main():
    seed_everything()
    device = get_device()
    print(f"Using device: {device}")

    # Discover and split data files
    all_files = discover_csv_files()
    if len(all_files) < 2:
        print(f"Found {len(all_files)} CSV file(s) in data/. Need at least 2 "
              "(one for train, one for validation). Collect more data!")
        return

    train_files, val_files = split_files(all_files)
    print(f"Train files: {len(train_files)}, Validation files: {len(val_files)}")
    for path, label in train_files:
        print(f"  TRAIN:      {path.name} (label={label})")
    for path, label in val_files:
        print(f"  VALIDATION: {path.name} (label={label})")

    # Compute normalization stats from training data only
    means, stds = compute_norm_stats(train_files)
    save_norm_stats(means, stds)
    print(f"Norm stats saved (means={means.round(3)}, stds={stds.round(3)})")

    # Create datasets and loaders
    train_dataset = StrokeDataset(train_files, means, stds)
    val_dataset = StrokeDataset(val_files, means, stds)
    print(f"Train windows: {len(train_dataset)}, Validation windows: {len(val_dataset)}")

    train_loader = DataLoader(
        train_dataset,
        batch_size=BATCH_SIZE,
        shuffle=True,       # randomize window order each epoch
        num_workers=0,       # MPS doesn't support multiprocess loading
        pin_memory=False,    # only useful for CUDA
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=BATCH_SIZE,
        shuffle=False,
        num_workers=0,
        pin_memory=False,
    )

    # Create model, loss function, optimizer
    model = StrokeCNN().to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam(
        model.parameters(),
        lr=LEARNING_RATE,
        weight_decay=WEIGHT_DECAY,
    )

    # Count parameters so you know model size
    num_params = sum(p.numel() for p in model.parameters())
    print(f"Model parameters: {num_params:,}")

    # Training loop
    best_val_loss = float("inf")
    best_epoch = 0
    CHECKPOINT_DIR.mkdir(parents=True, exist_ok=True)
    checkpoint_path = CHECKPOINT_DIR / "best_model.pt"

    # Class labels sorted by int, e.g. [(0, "no_stroke"), (1, "stroke")].
    # Used to build per-class accuracy columns in a stable order.
    class_labels = sorted(
        ((lbl_int, lbl_name) for lbl_name, lbl_int in LABEL_MAP.items())
    )
    per_class_header = " ".join(
        f"{name + ' Accuracy':>18}" for _, name in class_labels
    )

    print(f"\nTraining for {EPOCHS} epochs...")
    print(f"{'Epoch':>5} | {'Train Loss':>10} {'Train Accuracy':>15} | "
          f"{'Validation Loss':>15} {'Validation Accuracy':>19} | "
          f"{per_class_header} | {'Best':>4}")

    for epoch in range(1, EPOCHS + 1):
        train_loss, train_acc = train_one_epoch(
            model, train_loader, criterion, optimizer, device
        )
        val_loss, val_acc, per_class_acc = validate(
            model, val_loader, criterion, device
        )

        # Save checkpoint if this is the best validation loss so far
        is_best = val_loss < best_val_loss
        if is_best:
            best_val_loss = val_loss
            best_epoch = epoch
            torch.save({
                "epoch": epoch,
                "model_state_dict": model.state_dict(),
                "optimizer_state_dict": optimizer.state_dict(),
                "val_loss": val_loss,
                "val_acc": val_acc,
                "per_class_acc": per_class_acc,
            }, checkpoint_path)

        per_class_values = " ".join(
            f"{per_class_acc[lbl_int]:18.4f}"
            if lbl_int in per_class_acc else f"{'N/A':>18}"
            for lbl_int, _ in class_labels
        )
        print(f"{epoch:5d} | {train_loss:10.4f} {train_acc:15.4f} | "
              f"{val_loss:15.4f} {val_acc:19.4f} | "
              f"{per_class_values} | {'*' if is_best else ''}")

    print(f"\nDone! Best validation loss: {best_val_loss:.4f} at epoch {best_epoch}")
    print(f"Checkpoint saved to: {checkpoint_path}")


if __name__ == "__main__":
    main()
