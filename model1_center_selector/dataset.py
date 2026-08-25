from __future__ import annotations

import csv
from pathlib import Path
from typing import Callable

import torch
from PIL import Image
from torch.utils.data import Dataset

from model1_center_selector.build_sequences import YAW_COLUMN_NAMES


ImageTransform = Callable[[Image.Image], torch.Tensor]


class CenterSequenceDataset(Dataset):
    """Loads one training sample as seven yaw-sweep TRUS frames."""

    def __init__(
        self,
        csv_path: str | Path,
        root_dir: str | Path = ".",
        image_transform: ImageTransform | None = None,
        mask_transform: ImageTransform | None = None,
        use_masks: bool = False,
    ) -> None:
        self.csv_path = Path(csv_path)
        self.root_dir = Path(root_dir)
        self.image_transform = image_transform
        self.mask_transform = mask_transform or image_transform
        self.use_masks = use_masks

        with self.csv_path.open("r", encoding="utf-8-sig", newline="") as f:
            reader = csv.DictReader(f)
            required = {"sequence_id", "target_class"}
            required.update(f"image_{name}" for name in YAW_COLUMN_NAMES)
            if self.use_masks:
                required.update(f"mask_{name}" for name in YAW_COLUMN_NAMES)
            missing = required.difference(reader.fieldnames or [])
            if missing:
                raise ValueError(f"{self.csv_path} is missing columns: {sorted(missing)}")
            self.rows = list(reader)

        if not self.rows:
            raise ValueError(f"No rows found in {self.csv_path}")
        for row_number, row in enumerate(self.rows, start=2):
            try:
                target_class = int(row["target_class"])
            except (TypeError, ValueError) as exc:
                raise ValueError(f"Invalid target_class at {self.csv_path}:{row_number}") from exc
            if not 0 <= target_class < 8:
                raise ValueError(f"target_class must be 0..7 at {self.csv_path}:{row_number}")

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int) -> tuple[torch.Tensor, torch.Tensor]:
        row = self.rows[index]
        frames: list[torch.Tensor] = []
        for yaw_name in YAW_COLUMN_NAMES:
            image = self._load_grayscale(row[f"image_{yaw_name}"])
            image_tensor = self._to_tensor(image, self.image_transform)

            if self.use_masks:
                mask_path = row.get(f"mask_{yaw_name}", "")
                if not mask_path:
                    raise ValueError(f"Missing mask path for {row['sequence_id']} yaw {yaw_name}")
                mask = self._load_grayscale(mask_path)
                mask_tensor = self._to_tensor(mask, self.mask_transform)
                image_tensor = torch.cat([image_tensor, mask_tensor], dim=0)

            frames.append(image_tensor)

        x = torch.stack(frames, dim=0)
        y = torch.tensor(int(row["target_class"]), dtype=torch.long)
        return x, y

    def _load_grayscale(self, path_value: str) -> Image.Image:
        if not path_value:
            raise ValueError("Empty image path in sequence CSV")
        path = Path(path_value)
        if not path.is_absolute():
            path = self.root_dir / path
        if not path.is_file():
            raise FileNotFoundError(f"Model 1 input file not found: {path}")
        with Image.open(path) as image:
            return image.convert("L")

    @staticmethod
    def _to_tensor(image: Image.Image, transform: ImageTransform | None) -> torch.Tensor:
        if transform is not None:
            return transform(image)
        data = torch.ByteTensor(torch.ByteStorage.from_buffer(image.tobytes()))
        data = data.view(image.size[1], image.size[0], 1).permute(2, 0, 1).float()
        return data.div(255.0)

