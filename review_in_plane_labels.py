import argparse
import csv
from datetime import datetime
from pathlib import Path
import tkinter as tk

from PIL import Image, ImageDraw, ImageTk


LABEL_IN_PLANE = "IN_PLANE"
LABEL_OUT_OF_PLANE = "OUT_OF_PLANE"


def load_manifest(path, root, only_inside=True):
    with Path(path).open("r", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if only_inside:
        rows = [r for r in rows if r.get("projected_tip_inside_image") == "1"]
    for row in rows:
        row["_image_abs"] = root / row["image_path"]
    return rows


def load_labels(path):
    labels = {}
    label_path = Path(path)
    if not label_path.exists():
        return labels
    with label_path.open("r", newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row.get("sample_id"):
                labels[row["sample_id"]] = row
    return labels


def save_labels(path, labels):
    fieldnames = [
        "sample_id",
        "image_path",
        "sequence_id",
        "frame_index",
        "in_plane_label",
        "labeled_at",
    ]
    with Path(path).open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for sample_id in sorted(labels):
            writer.writerow(labels[sample_id])


def row_float(row, key):
    value = row.get(key, "")
    if value == "":
        return None
    return float(value)


def draw_overlay(row):
    image = Image.open(row["_image_abs"]).convert("RGB")
    draw = ImageDraw.Draw(image)

    box = []
    for idx in range(1, 5):
        x = row_float(row, f"box_x{idx}")
        y = row_float(row, f"box_y{idx}")
        if x is None or y is None:
            box = []
            break
        box.append((x, y))
    if len(box) == 4:
        draw.line(box + [box[0]], fill=(255, 255, 0), width=2)

    tip = (row_float(row, "tip_px_u"), row_float(row, "tip_px_v"))
    center = (row_float(row, "biopsy_center_px_u"), row_float(row, "biopsy_center_px_v"))
    if tip[0] is not None and tip[1] is not None:
        draw.ellipse((tip[0] - 5, tip[1] - 5, tip[0] + 5, tip[1] + 5), outline=(255, 0, 0), width=3)
    if center[0] is not None and center[1] is not None:
        draw.ellipse((center[0] - 5, center[1] - 5, center[0] + 5, center[1] + 5), outline=(0, 128, 255), width=3)

    if len(box) == 4:
        mid_start = ((box[0][0] + box[3][0]) / 2.0, (box[0][1] + box[3][1]) / 2.0)
        mid_end = ((box[1][0] + box[2][0]) / 2.0, (box[1][1] + box[2][1]) / 2.0)
        draw.line([mid_start, mid_end], fill=(0, 255, 0), width=2)

    return image


class Reviewer:
    def __init__(self, root, rows, label_path):
        self.root_dir = root
        self.rows = rows
        self.label_path = Path(label_path)
        self.labels = load_labels(self.label_path)
        self.index = 0

        self.window = tk.Tk()
        self.window.title("IN_PLANE / OUT_OF_PLANE reviewer")
        self.image_label = tk.Label(self.window)
        self.image_label.pack()
        self.status = tk.Label(self.window, anchor="w", justify="left")
        self.status.pack(fill="x")
        self.help = tk.Label(
            self.window,
            text="Left/Right: previous/next    i: IN_PLANE    o: OUT_OF_PLANE    u: clear label    q: quit",
            anchor="w",
        )
        self.help.pack(fill="x")

        for key in ("<Right>", "<space>"):
            self.window.bind(key, self.next_image)
        self.window.bind("<Left>", self.prev_image)
        self.window.bind("i", lambda event: self.set_label(LABEL_IN_PLANE))
        self.window.bind("o", lambda event: self.set_label(LABEL_OUT_OF_PLANE))
        self.window.bind("u", self.clear_label)
        self.window.bind("q", lambda event: self.window.destroy())
        self.render()

    def current_row(self):
        return self.rows[self.index]

    def current_label(self):
        row = self.current_row()
        return self.labels.get(row["sample_id"], {}).get("in_plane_label", "")

    def render(self):
        if not self.rows:
            self.status.config(text="No rows to review.")
            return
        row = self.current_row()
        image = draw_overlay(row)
        self.photo = ImageTk.PhotoImage(image)
        self.image_label.config(image=self.photo)
        label = self.current_label() or "UNLABELED"
        text = (
            f"{self.index + 1}/{len(self.rows)}    {label}\n"
            f"{row['sample_id']}\n"
            "red: unfired tip    blue: biopsy center    green/yellow: prior region"
        )
        self.status.config(text=text)

    def save_current_label(self, label):
        row = self.current_row()
        self.labels[row["sample_id"]] = {
            "sample_id": row["sample_id"],
            "image_path": row["image_path"],
            "sequence_id": row["sequence_id"],
            "frame_index": row["frame_index"],
            "in_plane_label": label,
            "labeled_at": datetime.now().isoformat(timespec="seconds"),
        }
        save_labels(self.label_path, self.labels)

    def set_label(self, label):
        self.save_current_label(label)
        self.next_image()

    def clear_label(self, event=None):
        row = self.current_row()
        self.labels.pop(row["sample_id"], None)
        save_labels(self.label_path, self.labels)
        self.render()

    def next_image(self, event=None):
        if self.rows:
            self.index = min(self.index + 1, len(self.rows) - 1)
        self.render()

    def prev_image(self, event=None):
        if self.rows:
            self.index = max(self.index - 1, 0)
        self.render()

    def run(self):
        self.window.mainloop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=".")
    parser.add_argument("--manifest", default="phantom_manifest.csv")
    parser.add_argument("--labels", default="phantom_in_plane_labels.csv")
    parser.add_argument("--all", action="store_true", help="Review all rows, not only projected-tip-inside rows.")
    args = parser.parse_args()

    root = Path(args.root).resolve()
    rows = load_manifest(root / args.manifest, root, only_inside=not args.all)
    Reviewer(root, rows, root / args.labels).run()


if __name__ == "__main__":
    main()
