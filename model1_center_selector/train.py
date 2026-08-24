from __future__ import annotations

import argparse
import random
from pathlib import Path

from model1_center_selector.label_rules import CLASS_NAMES


def split_dataset(dataset, val_fraction: float, seed: int):
    indices = list(range(len(dataset)))
    random.Random(seed).shuffle(indices)
    val_count = max(1, int(round(len(indices) * val_fraction)))
    val_indices = indices[:val_count]
    train_indices = indices[val_count:]
    if not train_indices:
        raise ValueError("Training set is empty after split")
    return Subset(dataset, train_indices), Subset(dataset, val_indices)


def evaluate(model, loader, device) -> tuple[float, float]:
    import torch
    from torch import nn

    model.eval()
    criterion = nn.CrossEntropyLoss()
    total_loss = 0.0
    correct = 0
    total = 0
    with torch.no_grad():
        for x, y in loader:
            x = x.to(device)
            y = y.to(device)
            logits = model(x)
            loss = criterion(logits, y)
            total_loss += loss.item() * y.size(0)
            correct += (logits.argmax(dim=1) == y).sum().item()
            total += y.size(0)
    return total_loss / max(total, 1), correct / max(total, 1)


def main() -> None:
    parser = argparse.ArgumentParser(description="Train Model 1: seven-frame ultrasound center selector")
    parser.add_argument("--train-csv", required=True, type=Path)
    parser.add_argument("--val-csv", type=Path)
    parser.add_argument("--root-dir", default=".", type=Path)
    parser.add_argument("--output", default="runs/model1_center_selector/best.pt", type=Path)
    parser.add_argument("--epochs", default=30, type=int)
    parser.add_argument("--batch-size", default=8, type=int)
    parser.add_argument("--lr", default=1e-4, type=float)
    parser.add_argument("--image-size", default=224, type=int)
    parser.add_argument("--use-masks", action="store_true")
    parser.add_argument("--pretrained", action="store_true")
    parser.add_argument("--val-fraction", default=0.2, type=float)
    parser.add_argument("--seed", default=42, type=int)
    args = parser.parse_args()

    try:
        import torch
        from torch import nn
        from torch.utils.data import DataLoader
        from torchvision import transforms
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Missing training dependency. Activate an environment with torch and torchvision installed, "
            "then run this command again."
        ) from exc

    from model1_center_selector.dataset import CenterSequenceDataset
    from model1_center_selector.model import SevenFrameCenterSelector

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    transform = transforms.Compose(
        [
            transforms.Resize((args.image_size, args.image_size)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.5], std=[0.5]),
        ]
    )

    train_dataset_full = CenterSequenceDataset(
        args.train_csv,
        root_dir=args.root_dir,
        image_transform=transform,
        mask_transform=transform,
        use_masks=args.use_masks,
    )
    if args.val_csv:
        train_dataset = train_dataset_full
        val_dataset = CenterSequenceDataset(
            args.val_csv,
            root_dir=args.root_dir,
            image_transform=transform,
            mask_transform=transform,
            use_masks=args.use_masks,
        )
    else:
        train_dataset, val_dataset = split_dataset(train_dataset_full, args.val_fraction, args.seed)

    train_loader = DataLoader(train_dataset, batch_size=args.batch_size, shuffle=True, num_workers=0)
    val_loader = DataLoader(val_dataset, batch_size=args.batch_size, shuffle=False, num_workers=0)

    input_channels = 2 if args.use_masks else 1
    model = SevenFrameCenterSelector(input_channels=input_channels, pretrained=args.pretrained).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    criterion = nn.CrossEntropyLoss()

    best_val_acc = -1.0
    args.output.parent.mkdir(parents=True, exist_ok=True)
    print(f"Classes: {', '.join(CLASS_NAMES)}")
    print(f"Device: {device}")

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

        val_loss, val_acc = evaluate(model, val_loader, device)
        train_acc = train_correct / max(train_total, 1)
        print(
            f"epoch={epoch:03d} "
            f"train_loss={train_loss / max(train_total, 1):.4f} "
            f"train_acc={train_acc:.4f} "
            f"val_loss={val_loss:.4f} "
            f"val_acc={val_acc:.4f}"
        )

        if val_acc > best_val_acc:
            best_val_acc = val_acc
            torch.save(
                {
                    "model_state": model.state_dict(),
                    "class_names": CLASS_NAMES,
                    "use_masks": args.use_masks,
                    "image_size": args.image_size,
                },
                args.output,
            )
            print(f"saved best checkpoint to {args.output}")


if __name__ == "__main__":
    main()
