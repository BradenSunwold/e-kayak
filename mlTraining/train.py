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
    Split file list into train and val sets.

    We split by FILE, not by window. If we split by window, adjacent windows
    from the same file would end up in both train and val (because of the 75%
    overlap), and the model would essentially memorize the validation set.
    """
    random.seed(RANDOM_SEED)
    shuffled = file_list.copy()
    random.shuffle(shuffled)
    n_val = max(1, int(len(shuffled) * val_split))
    return shuffled[n_val:], shuffled[:n_val]


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
    """Run validation. Returns average loss and accuracy."""
    model.eval()  # disable dropout, batchnorm uses running stats
    total_loss = 0.0
    total_correct = 0
    total_samples = 0

    with torch.no_grad():  # no gradient computation needed for validation
        for windows, labels in loader:
            windows = windows.to(device)
            labels = labels.to(device)

            predictions = model(windows)
            loss = criterion(predictions, labels)

            total_loss += loss.item() * labels.size(0)
            total_correct += (predictions.argmax(dim=1) == labels).sum().item()
            total_samples += labels.size(0)

    avg_loss = total_loss / total_samples
    accuracy = total_correct / total_samples
    return avg_loss, accuracy


def main():
    seed_everything()
    device = get_device()
    print(f"Using device: {device}")

    # Discover and split data files
    all_files = discover_csv_files()
    if len(all_files) < 2:
        print(f"Found {len(all_files)} CSV file(s) in data/. Need at least 2 "
              "(one for train, one for val). Collect more data!")
        return

    train_files, val_files = split_files(all_files)
    print(f"Train files: {len(train_files)}, Val files: {len(val_files)}")
    for path, label in train_files:
        print(f"  TRAIN: {path.name} (label={label})")
    for path, label in val_files:
        print(f"  VAL:   {path.name} (label={label})")

    # Compute normalization stats from training data only
    means, stds = compute_norm_stats(train_files)
    save_norm_stats(means, stds)
    print(f"Norm stats saved (means={means.round(3)}, stds={stds.round(3)})")

    # Create datasets and loaders
    train_dataset = StrokeDataset(train_files, means, stds)
    val_dataset = StrokeDataset(val_files, means, stds)
    print(f"Train windows: {len(train_dataset)}, Val windows: {len(val_dataset)}")

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

    print(f"\nTraining for {EPOCHS} epochs...")
    print(f"{'Epoch':>5} | {'Train Loss':>10} {'Train Acc':>10} | "
          f"{'Val Loss':>10} {'Val Acc':>10} | {'Best':>4}")

    for epoch in range(1, EPOCHS + 1):
        train_loss, train_acc = train_one_epoch(
            model, train_loader, criterion, optimizer, device
        )
        val_loss, val_acc = validate(model, val_loader, criterion, device)

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
            }, checkpoint_path)

        print(f"{epoch:5d} | {train_loss:10.4f} {train_acc:10.4f} | "
              f"{val_loss:10.4f} {val_acc:10.4f} | {'*' if is_best else ''}")

    print(f"\nDone! Best val loss: {best_val_loss:.4f} at epoch {best_epoch}")
    print(f"Checkpoint saved to: {checkpoint_path}")


if __name__ == "__main__":
    main()
