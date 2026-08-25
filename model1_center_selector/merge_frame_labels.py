from __future__ import annotations

import argparse
import csv
from pathlib import Path


FIELDNAMES = ("sequence_id", "yaw_offset_deg", "image_path", "mask_path", "center_label")


def merge_frame_csvs(capture_root: Path, output_csv: Path) -> int:
    csv_paths = sorted(capture_root.glob("seq_*/frame_labels.csv"))
    if not csv_paths:
        raise ValueError(f"No seq_*/frame_labels.csv files found under {capture_root}")

    rows: list[dict[str, str]] = []
    seen: set[tuple[str, str]] = set()
    for csv_path in csv_paths:
        with csv_path.open("r", encoding="utf-8-sig", newline="") as csv_file:
            reader = csv.DictReader(csv_file)
            missing = set(FIELDNAMES).difference(reader.fieldnames or [])
            if missing:
                raise ValueError(f"{csv_path} is missing columns: {sorted(missing)}")
            for row in reader:
                key = (row["sequence_id"], row["yaw_offset_deg"])
                if key in seen:
                    raise ValueError(f"Duplicate sequence/yaw pair: {key}")
                seen.add(key)
                rows.append({name: row.get(name, "") for name in FIELDNAMES})

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", encoding="utf-8-sig", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description="Merge Model 1 per-sequence frame label CSV files")
    parser.add_argument("--capture-root", default="image/model1_center_capture", type=Path)
    parser.add_argument("--output-csv", default="model1_center_frame_labels.csv", type=Path)
    args = parser.parse_args()
    row_count = merge_frame_csvs(args.capture_root, args.output_csv)
    print(f"Wrote {row_count} frame rows to {args.output_csv}")


if __name__ == "__main__":
    main()
