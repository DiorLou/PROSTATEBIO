from __future__ import annotations

import argparse
import json
import random
from collections import Counter, defaultdict
from pathlib import Path

from model1_center_selector.label_rules import CLASS_NAMES


def seed_everything(seed: int) -> None:
    random.seed(seed)
    try:
        import numpy as np
        import torch

        np.random.seed(seed)
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(seed)
    except ModuleNotFoundError:
        pass


def stratified_split_indices(rows, val_fraction: float, seed: int) -> tuple[list[int], list[int]]:
    """Split complete seven-frame sequences while approximately preserving classes."""
    if not 0.0 < val_fraction < 1.0:
        raise ValueError("--val-fraction must be between 0 and 1")
    if len(rows) < 2:
        raise ValueError("At least two sequences are required for an automatic train/validation split")

    by_class: dict[int, list[int]] = defaultdict(list)
    for index, row in enumerate(rows):
        by_class[int(row["target_class"])].append(index)

    rng = random.Random(seed)
    train_indices: list[int] = []
    val_indices: list[int] = []
    for indices in by_class.values():
        rng.shuffle(indices)
        if len(indices) == 1:
            train_indices.extend(indices)
            continue
        val_count = min(len(indices) - 1, max(1, int(round(len(indices) * val_fraction))))
        val_indices.extend(indices[:val_count])
        train_indices.extend(indices[val_count:])

    if not val_indices:
        rng.shuffle(train_indices)
        val_indices.append(train_indices.pop())
    rng.shuffle(train_indices)
    rng.shuffle(val_indices)
    return train_indices, val_indices


def confusion_metrics(confusion) -> tuple[float, float, list[float]]:
    total = int(confusion.sum().item())
    accuracy = float(confusion.diag().sum().item() / total) if total else 0.0
    f1_values: list[float] = []
    recalls: list[float] = []
    for class_index in range(confusion.shape[0]):
        tp = float(confusion[class_index, class_index].item())
        actual = float(confusion[class_index, :].sum().item())
        predicted = float(confusion[:, class_index].sum().item())
        recall = tp / actual if actual else 0.0
        precision = tp / predicted if predicted else 0.0
        f1 = 2.0 * precision * recall / (precision + recall) if precision + recall else 0.0
        recalls.append(recall)
        if actual:
            f1_values.append(f1)
    macro_f1 = sum(f1_values) / len(f1_values) if f1_values else 0.0
    return accuracy, macro_f1, recalls


def evaluate(model, loader, device, criterion):
    import torch

    model.eval()
    total_loss = 0.0
    total = 0
    confusion = torch.zeros((len(CLASS_NAMES), len(CLASS_NAMES)), dtype=torch.long)
    with torch.no_grad():
        for x, y in loader:
            x = x.to(device)
            y = y.to(device)
            logits = model(x)
            loss = criterion(logits, y)
            total_loss += loss.item() * y.size(0)
            total += y.size(0)
            for target, prediction in zip(y.cpu().tolist(), logits.argmax(dim=1).cpu().tolist()):
                confusion[target, prediction] += 1
    accuracy, macro_f1, recalls = confusion_metrics(confusion)
    return total_loss / max(total, 1), accuracy, macro_f1, recalls, confusion


def class_weights_from_rows(rows, indices):
    import torch

    counts = Counter(int(rows[index]["target_class"]) for index in indices)
    total = sum(counts.values())
    weights = [total / (len(CLASS_NAMES) * counts[i]) if counts[i] else 0.0 for i in range(len(CLASS_NAMES))]
    return torch.tensor(weights, dtype=torch.float32), counts


def main() -> None:
    parser = argparse.ArgumentParser(description="Train Model 1: seven-frame ultrasound center selector")
    parser.add_argument("--train-csv", required=True, type=Path)
    parser.add_argument("--val-csv", type=Path, help="Recommended: validation CSV from separate capture sessions")
    parser.add_argument("--root-dir", default=".", type=Path)
    parser.add_argument("--output", default="runs/model1_center_selector/best.pt", type=Path)
    parser.add_argument("--epochs", default=30, type=int)
    parser.add_argument("--batch-size", default=8, type=int)
    parser.add_argument("--lr", default=1e-4, type=float)
    parser.add_argument("--image-size", default=224, type=int)
    parser.add_argument("--use-masks", action="store_true")
    parser.add_argument("--pretrained", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--class-balanced-loss", action="store_true")
    parser.add_argument("--val-fraction", default=0.2, type=float)
    parser.add_argument("--seed", default=42, type=int)
    args = parser.parse_args()

    if args.epochs <= 0 or args.batch_size <= 0 or args.lr <= 0 or args.image_size <= 0:
        parser.error("epochs, batch-size, lr and image-size must all be positive")

    try:
        import torch
        from torch import nn
        from torch.utils.data import DataLoader, Subset
        from torchvision import transforms
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Missing training dependency. Activate .venv311 (or another environment with torch and torchvision)."
        ) from exc

    from model1_center_selector.dataset import CenterSequenceDataset
    from model1_center_selector.model import SevenFrameCenterSelector

    seed_everything(args.seed)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    transform = transforms.Compose([
        transforms.Resize((args.image_size, args.image_size)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.5], std=[0.5]),
    ])

    train_dataset_full = CenterSequenceDataset(
        args.train_csv, root_dir=args.root_dir, image_transform=transform,
        mask_transform=transform, use_masks=args.use_masks,
    )
    if args.val_csv:
        train_dataset = train_dataset_full
        train_indices = list(range(len(train_dataset_full)))
        val_dataset = CenterSequenceDataset(
            args.val_csv, root_dir=args.root_dir, image_transform=transform,
            mask_transform=transform, use_masks=args.use_masks,
        )
    else:
        train_indices, val_indices = stratified_split_indices(
            train_dataset_full.rows, args.val_fraction, args.seed
        )
        train_dataset = Subset(train_dataset_full, train_indices)
        val_dataset = Subset(train_dataset_full, val_indices)

    generator = torch.Generator().manual_seed(args.seed)
    train_loader = DataLoader(
        train_dataset, batch_size=args.batch_size, shuffle=True, num_workers=0, generator=generator
    )
    val_loader = DataLoader(val_dataset, batch_size=args.batch_size, shuffle=False, num_workers=0)

    input_channels = 2 if args.use_masks else 1
    model = SevenFrameCenterSelector(input_channels=input_channels, pretrained=args.pretrained).to(device)
    class_weights, train_counts = class_weights_from_rows(train_dataset_full.rows, train_indices)
    criterion = nn.CrossEntropyLoss(weight=class_weights.to(device) if args.class_balanced_loss else None)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode="max", factor=0.5, patience=3)

    best_val_macro_f1 = -1.0
    args.output.parent.mkdir(parents=True, exist_ok=True)
    print(f"Classes: {', '.join(CLASS_NAMES)}")
    print(f"Device: {device}; train={len(train_dataset)}; val={len(val_dataset)}")
    print("Training class counts: " + ", ".join(f"{CLASS_NAMES[i]}={train_counts[i]}" for i in range(8)))

    for epoch in range(1, args.epochs + 1):
        model.train()
        train_loss = 0.0
        train_correct = 0
        train_total = 0
        for x, y in train_loader:
            x = x.to(device)
            y = y.to(device)
            optimizer.zero_grad(set_to_none=True)
            logits = model(x)
            loss = criterion(logits, y)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * y.size(0)
            train_correct += (logits.argmax(dim=1) == y).sum().item()
            train_total += y.size(0)

        val_loss, val_acc, val_macro_f1, recalls, confusion = evaluate(
            model, val_loader, device, criterion
        )
        scheduler.step(val_macro_f1)
        train_acc = train_correct / max(train_total, 1)
        print(
            f"epoch={epoch:03d} train_loss={train_loss / max(train_total, 1):.4f} "
            f"train_acc={train_acc:.4f} val_loss={val_loss:.4f} "
            f"val_acc={val_acc:.4f} val_macro_f1={val_macro_f1:.4f}"
        )

        if val_macro_f1 > best_val_macro_f1:
            best_val_macro_f1 = val_macro_f1
            torch.save({
                "model_state": model.state_dict(),
                "class_names": CLASS_NAMES,
                "use_masks": args.use_masks,
                "image_size": args.image_size,
                "input_channels": input_channels,
                "pretrained_backbone": args.pretrained,
                "best_val_accuracy": val_acc,
                "best_val_macro_f1": val_macro_f1,
                "val_class_recalls": recalls,
                "val_confusion_matrix": confusion.tolist(),
            }, args.output)
            print(f"saved best checkpoint to {args.output}")

    metrics_path = args.output.with_suffix(".metrics.json")
    metrics_path.write_text(json.dumps({
        "best_val_macro_f1": best_val_macro_f1,
        "class_names": CLASS_NAMES,
        "training_class_counts": {CLASS_NAMES[i]: train_counts[i] for i in range(8)},
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"saved metrics summary to {metrics_path}")


if __name__ == "__main__":
    main()
