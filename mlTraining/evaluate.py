"""
Evaluate the best trained model on the validation set.

Loads the best checkpoint and prints:
- Overall accuracy
- Per-class precision, recall, F1
- Confusion matrix

Usage:
    python evaluate.py

What the metrics mean:
    Accuracy:  % of all windows classified correctly
    Precision: when the model says "stroke", how often is it right?
    Recall:    of all actual strokes, how many did the model catch?
    F1:        harmonic mean of precision and recall (balances both)

    Confusion matrix:
        Rows = actual label, Columns = predicted label
        Diagonal = correct predictions, off-diagonal = errors
        e.g. row "stroke", column "no_stroke" = strokes the model missed
"""

import numpy as np
import torch

from config import CHECKPOINT_DIR, LABEL_MAP
from dataset import StrokeDataset, discover_csv_files
from model import StrokeCNN
from train import split_files  # reuse the exact split used during training
from utils import get_device, load_norm_stats, seed_everything


def main():
    seed_everything()
    device = get_device()

    # Reconstruct the same val split used during training
    all_files = discover_csv_files()
    train_files, val_files = split_files(all_files)

    # Load norm stats that were saved during training
    means, stds = load_norm_stats()
    val_dataset = StrokeDataset(val_files, means, stds)
    print(f"Evaluating on {len(val_dataset)} windows from {len(val_files)} file(s)")

    # Load best model checkpoint
    checkpoint_path = CHECKPOINT_DIR / "best_model.pt"
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=True)

    model = StrokeCNN().to(device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()

    print(f"Loaded checkpoint from epoch {checkpoint['epoch']} "
          f"(validation_loss={checkpoint['val_loss']:.4f})")

    # Run predictions on all val windows
    all_preds = []
    all_labels = []

    with torch.no_grad():
        for i in range(len(val_dataset)):
            window, label = val_dataset[i]
            window = window.unsqueeze(0).to(device)  # add batch dim: (1, 6, 40)
            output = model(window)                    # (1, 2)
            pred = output.argmax(dim=1).item()
            all_preds.append(pred)
            all_labels.append(label)

    all_preds = np.array(all_preds)
    all_labels = np.array(all_labels)

    # Reverse LABEL_MAP for readable output: {0: "no_stroke", 1: "stroke"}
    label_names = {v: k for k, v in LABEL_MAP.items()}
    num_classes = len(LABEL_MAP)

    # Overall accuracy
    accuracy = (all_preds == all_labels).mean()
    print(f"\nOverall accuracy: {accuracy:.1%}")

    # Confusion matrix
    confusion = np.zeros((num_classes, num_classes), dtype=int)
    for true, pred in zip(all_labels, all_preds):
        confusion[true][pred] += 1

    # Print confusion matrix
    col_width = max(len(n) for n in label_names.values()) + 2
    header = "Actual \\ Predicted".ljust(col_width)
    for i in range(num_classes):
        header += label_names[i].rjust(col_width)
    print(f"\n{header}")
    print("-" * len(header))
    for i in range(num_classes):
        row = label_names[i].ljust(col_width)
        for j in range(num_classes):
            row += str(confusion[i][j]).rjust(col_width)
        print(row)

    # Per-class precision, recall, F1
    print(f"\n{'Class':<12} {'Precision':>10} {'Recall':>10} {'F1':>10}")
    print("-" * 44)
    for i in range(num_classes):
        tp = confusion[i][i]
        fp = confusion[:, i].sum() - tp  # other classes predicted as this
        fn = confusion[i, :].sum() - tp  # this class predicted as other

        precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
        f1 = (2 * precision * recall / (precision + recall)
              if (precision + recall) > 0 else 0.0)

        print(f"{label_names[i]:<12} {precision:>10.1%} {recall:>10.1%} {f1:>10.1%}")


if __name__ == "__main__":
    main()
