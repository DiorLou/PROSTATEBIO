from __future__ import annotations

import argparse
import json
from pathlib import Path

from model1_center_selector.label_rules import CLASS_NAMES, class_index_to_yaw_offset


def main() -> None:
    parser = argparse.ArgumentParser(description="Predict Model 1 target yaw from seven TRUS frames")
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--images", required=True, nargs=7, type=Path, help="Images in order: -3 -2 -1 0 +1 +2 +3")
    parser.add_argument("--masks", nargs=7, type=Path, help="Optional masks in the same yaw order")
    parser.add_argument("--image-size", default=None, type=int)
    parser.add_argument("--output-json", type=Path)
    args = parser.parse_args()

    try:
        import torch
        from PIL import Image
        from torchvision import transforms
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Missing prediction dependency. Activate an environment with torch, torchvision, and pillow installed, "
            "then run this command again."
        ) from exc

    from model1_center_selector.model import SevenFrameCenterSelector

    def load_frame(path: Path, transform: transforms.Compose) -> torch.Tensor:
        image = Image.open(path).convert("L")
        return transform(image)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if not args.checkpoint.is_file():
        raise FileNotFoundError(f"Checkpoint not found: {args.checkpoint}")
    for path in args.images:
        if not path.is_file():
            raise FileNotFoundError(f"TRUS image not found: {path}")

    checkpoint = torch.load(args.checkpoint, map_location=device, weights_only=True)
    use_masks = bool(checkpoint.get("use_masks", False))
    image_size = args.image_size or int(checkpoint.get("image_size", 224))
    if use_masks and not args.masks:
        raise ValueError("This checkpoint was trained with masks; provide --masks with 7 paths")
    if not use_masks and args.masks:
        raise ValueError("This checkpoint was trained without masks; remove --masks")
    if args.masks:
        for path in args.masks:
            if not path.is_file():
                raise FileNotFoundError(f"Prior mask not found: {path}")

    transform = transforms.Compose(
        [
            transforms.Resize((image_size, image_size)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.5], std=[0.5]),
        ]
    )

    frames = []
    for index, image_path in enumerate(args.images):
        frame = load_frame(image_path, transform)
        if use_masks:
            assert args.masks is not None
            mask = load_frame(args.masks[index], transform)
            frame = torch.cat([frame, mask], dim=0)
        frames.append(frame)

    x = torch.stack(frames, dim=0).unsqueeze(0).to(device)
    input_channels = 2 if use_masks else 1
    model = SevenFrameCenterSelector(input_channels=input_channels, pretrained=False).to(device)
    model.load_state_dict(checkpoint["model_state"])
    model.eval()

    with torch.no_grad():
        logits = model(x)
        probs = torch.softmax(logits, dim=1).squeeze(0)
        pred_class = int(probs.argmax().item())

    yaw_offset = class_index_to_yaw_offset(pred_class)
    class_names = checkpoint.get("class_names", CLASS_NAMES)
    if len(class_names) != len(CLASS_NAMES):
        raise ValueError(f"Checkpoint has {len(class_names)} classes; expected {len(CLASS_NAMES)}")
    print(f"pred_class={pred_class}")
    print(f"pred_name={class_names[pred_class]}")
    print(f"pred_yaw_offset_deg={yaw_offset if yaw_offset is not None else 'NO_VALID'}")
    print("probabilities:")
    for index, prob in enumerate(probs.tolist()):
        yaw = class_index_to_yaw_offset(index)
        yaw_text = str(yaw) if yaw is not None else "NO_VALID"
        print(f"  {index}: {class_names[index]} yaw={yaw_text} prob={prob:.4f}")

    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps({
            "pred_class": pred_class,
            "pred_name": class_names[pred_class],
            "pred_yaw_offset_deg": yaw_offset,
            "probabilities": {
                class_names[index]: probability
                for index, probability in enumerate(probs.tolist())
            },
        }, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"saved prediction JSON to {args.output_json}")


if __name__ == "__main__":
    main()
