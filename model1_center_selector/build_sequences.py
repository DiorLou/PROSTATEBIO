from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path

from model1_center_selector.label_rules import (
    CLASS_NAMES,
    YAW_OFFSETS_DEG,
    normalize_center_label,
    select_center_target,
)


YAW_COLUMN_NAMES = ("m3", "m2", "m1", "0", "p1", "p2", "p3")


def _yaw_to_index(yaw_offset_deg: float) -> int:
    rounded = int(round(yaw_offset_deg))
    if rounded not in YAW_OFFSETS_DEG:
        raise ValueError(f"Unsupported yaw offset {yaw_offset_deg}; expected one of {YAW_OFFSETS_DEG}")
    return YAW_OFFSETS_DEG.index(rounded)


def build_sequence_csv(frame_csv: Path, output_csv: Path) -> None:
    groups: dict[str, list[dict[str, str]]] = defaultdict(list)
    with frame_csv.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        required = {"sequence_id", "yaw_offset_deg", "image_path", "center_label"}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{frame_csv} is missing columns: {sorted(missing)}")

        for row in reader:
            row["center_label"] = normalize_center_label(row["center_label"])
            groups[row["sequence_id"]].append(row)

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", encoding="utf-8", newline="") as f:
        fieldnames = ["sequence_id"]
        fieldnames += [f"image_{name}" for name in YAW_COLUMN_NAMES]
        fieldnames += [f"mask_{name}" for name in YAW_COLUMN_NAMES]
        fieldnames += [f"label_{name}" for name in YAW_COLUMN_NAMES]
        fieldnames += ["target_class", "target_name", "target_yaw_offset_deg"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for sequence_id in sorted(groups):
            slots: list[dict[str, str] | None] = [None] * len(YAW_OFFSETS_DEG)
            for row in groups[sequence_id]:
                index = _yaw_to_index(float(row["yaw_offset_deg"]))
                if slots[index] is not None:
                    raise ValueError(f"Duplicate yaw offset {row['yaw_offset_deg']} in sequence {sequence_id}")
                slots[index] = row

            if any(slot is None for slot in slots):
                missing_offsets = [
                    str(YAW_OFFSETS_DEG[index])
                    for index, slot in enumerate(slots)
                    if slot is None
                ]
                raise ValueError(f"Sequence {sequence_id} is missing yaw offsets: {', '.join(missing_offsets)}")

            labels = [slot["center_label"] for slot in slots if slot is not None]
            decision = select_center_target(labels)
            out = {
                "sequence_id": sequence_id,
                "target_class": str(decision.target_class),
                "target_name": decision.target_name,
                "target_yaw_offset_deg": "" if decision.target_yaw_offset_deg is None else str(decision.target_yaw_offset_deg),
            }

            for name, slot in zip(YAW_COLUMN_NAMES, slots):
                assert slot is not None
                out[f"image_{name}"] = slot["image_path"]
                out[f"mask_{name}"] = slot.get("mask_path", "")
                out[f"label_{name}"] = slot["center_label"]

            writer.writerow(out)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Build sequence-level labels for Model 1 from per-frame IN_CENTER/OFF_CENTER labels. "
            "Target priority: 0, -1, +1, -2, +2, -3, +3, then NO_VALID."
        )
    )
    parser.add_argument("--frame-csv", required=True, type=Path)
    parser.add_argument("--output-csv", required=True, type=Path)
    args = parser.parse_args()
    build_sequence_csv(args.frame_csv, args.output_csv)
    print(f"Wrote {args.output_csv}")
    print(f"Classes: {', '.join(CLASS_NAMES)}")


if __name__ == "__main__":
    main()

