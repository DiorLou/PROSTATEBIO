import argparse
import csv
import math
import os
import re
from datetime import datetime
from pathlib import Path

from PIL import Image, ImageDraw


MM_PER_PIXEL = 66.0 / 420.0
TCP_U_ORIGIN_U_PX = 406.0
TCP_U_ORIGIN_V_PX = 420.0 + 10.0 / MM_PER_PIXEL
UNFIRED_NEEDLE_RETRACTION_MM = 26.0
BIOPSY_CENTER_OFFSET_MM = 10.0
KINEMATIC_BOX_HALF_WIDTH_PX = 25.0
KINEMATIC_BOX_TIP_EXTENSION_MM = 6.0

FILENAME_RE = re.compile(
    r"^(?P<frame>\d+)_E\((?P<pose_e>[^)]*)\)"
    r"_TipU\((?P<tip>[-0-9.,]+)\)"
    r"_VecU\((?P<vec>[-0-9.,]+)\)_Box"
)


def parse_floats(text, expected):
    values = [float(v) for v in text.split(",")]
    if len(values) != expected:
        raise ValueError(f"Expected {expected} values, got {len(values)}.")
    return values


def project_point_u(point_u):
    x, y, z = point_u
    return (
        TCP_U_ORIGIN_U_PX - z / MM_PER_PIXEL,
        TCP_U_ORIGIN_V_PX + y / MM_PER_PIXEL,
    )


def vector_to_image_direction(vector_u):
    vx, vy, vz = vector_u
    du = -vz / MM_PER_PIXEL
    dv = vy / MM_PER_PIXEL
    norm = math.hypot(du, dv)
    if norm < 1e-9:
        return None
    return (du / norm, dv / norm)


def clip_line_to_rect(px, py, dx, dy, width, height):
    x1, y1 = px - dx * 2000.0, py - dy * 2000.0
    x2, y2 = px + dx * 2000.0, py + dy * 2000.0
    p_values = [-(x2 - x1), x2 - x1, -(y2 - y1), y2 - y1]
    q_values = [x1, width - 1 - x1, y1, height - 1 - y1]
    u1, u2 = 0.0, 1.0
    for p, q in zip(p_values, q_values):
        if abs(p) < 1e-12:
            if q < 0:
                return None
            continue
        r = q / p
        if p < 0:
            u1 = max(u1, r)
        else:
            u2 = min(u2, r)
    if u1 > u2:
        return None
    return (
        (x1 + u1 * (x2 - x1), y1 + u1 * (y2 - y1)),
        (x1 + u2 * (x2 - x1), y1 + u2 * (y2 - y1)),
    )


def make_box(tip_px, direction_px, width, height, half_width_px, tip_extension_px):
    u, v = tip_px
    if not (0.0 <= u <= width - 1 and 0.0 <= v <= height - 1):
        return None

    du, dv = direction_px
    shaft_px = (u - du * 2000.0, v - dv * 2000.0)
    clipped = clip_line_to_rect(
        (shaft_px[0] + u) / 2.0,
        (shaft_px[1] + v) / 2.0,
        du,
        dv,
        width,
        height,
    )
    if clipped is None:
        return None

    start, end = clipped
    # Extend the tip side forward so calibration error does not exclude the
    # actual visual tip from the kinematic search region.
    end = (
        tip_px[0] + direction_px[0] * tip_extension_px,
        tip_px[1] + direction_px[1] * tip_extension_px,
    )
    segment = (end[0] - start[0], end[1] - start[1])
    segment_norm = math.hypot(segment[0], segment[1])
    if segment_norm < 1e-9:
        return None
    sx, sy = segment[0] / segment_norm, segment[1] / segment_norm
    nx, ny = -sy * half_width_px, sx * half_width_px
    corners = [
        (start[0] + nx, start[1] + ny),
        (end[0] + nx, end[1] + ny),
        (end[0] - nx, end[1] - ny),
        (start[0] - nx, start[1] - ny),
    ]
    return [
        (
            min(max(x, 0.0), width - 1.0),
            min(max(y, 0.0), height - 1.0),
        )
        for x, y in corners
    ]


def write_mask(mask_path, size, box):
    mask = Image.new("L", size, 0)
    if box is not None:
        draw = ImageDraw.Draw(mask)
        draw.polygon(box, fill=255)
    mask_path.parent.mkdir(parents=True, exist_ok=True)
    mask.save(mask_path)


def relpath(path, root):
    return Path(path).resolve().relative_to(root.resolve()).as_posix()


def build_manifest(args):
    root = Path(args.root).resolve()
    image_dir = (root / args.image_dir).resolve()
    mask_dir = (root / args.mask_dir).resolve()
    output_csv = (root / args.output).resolve()

    rows = []
    png_paths = sorted(image_dir.rglob("*.png"))
    for image_path in png_paths:
        match = FILENAME_RE.search(image_path.name)
        if match is None:
            continue

        frame_index = int(match.group("frame"))
        pose_e = parse_floats(match.group("pose_e"), 6)
        filename_tip_u = parse_floats(match.group("tip"), 3)
        vec_u = parse_floats(match.group("vec"), 3)
        vec_norm = math.sqrt(sum(v * v for v in vec_u))
        if vec_norm < 1e-9:
            continue
        vec_u = [v / vec_norm for v in vec_u]

        if args.filename_tip_state == "fired":
            tip_unfired_u = [
                filename_tip_u[i] - args.unfired_retraction_mm * vec_u[i]
                for i in range(3)
            ]
        else:
            tip_unfired_u = filename_tip_u[:]

        biopsy_center_u = [
            tip_unfired_u[i] + args.biopsy_center_offset_mm * vec_u[i]
            for i in range(3)
        ]

        with Image.open(image_path) as image:
            width, height = image.size

        tip_px = project_point_u(tip_unfired_u)
        biopsy_center_px = project_point_u(biopsy_center_u)
        direction_px = vector_to_image_direction(vec_u)
        projected_tip_inside = (
            0.0 <= tip_px[0] <= width - 1 and 0.0 <= tip_px[1] <= height - 1
        )
        box = None
        if direction_px is not None:
            box = make_box(
                tip_px,
                direction_px,
                width,
                height,
                args.box_half_width_px,
                args.box_tip_extension_mm / MM_PER_PIXEL,
            )

        sequence_id = image_path.parent.name
        sample_id = f"{sequence_id}_{frame_index:04d}"
        mask_path = mask_dir / sequence_id / f"{frame_index:04d}_prior_mask.png"
        write_mask(mask_path, (width, height), box)

        row = {
            "sample_id": sample_id,
            "sequence_id": sequence_id,
            "frame_index": frame_index,
            "image_path": relpath(image_path, root),
            "image_width": width,
            "image_height": height,
            "mask_path": relpath(mask_path, root),
            "filename_tip_state": args.filename_tip_state,
            "filename_tip_u_x": filename_tip_u[0],
            "filename_tip_u_y": filename_tip_u[1],
            "filename_tip_u_z": filename_tip_u[2],
            "tip_unfired_u_x": tip_unfired_u[0],
            "tip_unfired_u_y": tip_unfired_u[1],
            "tip_unfired_u_z": tip_unfired_u[2],
            "vec_u_x": vec_u[0],
            "vec_u_y": vec_u[1],
            "vec_u_z": vec_u[2],
            "biopsy_center_u_x": biopsy_center_u[0],
            "biopsy_center_u_y": biopsy_center_u[1],
            "biopsy_center_u_z": biopsy_center_u[2],
            "tip_px_u": tip_px[0],
            "tip_px_v": tip_px[1],
            "biopsy_center_px_u": biopsy_center_px[0],
            "biopsy_center_px_v": biopsy_center_px[1],
            "projected_tip_inside_image": int(projected_tip_inside),
            "box_half_width_px": args.box_half_width_px,
            "box_tip_extension_mm": args.box_tip_extension_mm,
            "label_status": "",
            "in_plane_label": "",
        }

        for idx in range(4):
            if box is None:
                row[f"box_x{idx + 1}"] = ""
                row[f"box_y{idx + 1}"] = ""
            else:
                row[f"box_x{idx + 1}"] = box[idx][0]
                row[f"box_y{idx + 1}"] = box[idx][1]

        for idx, value in enumerate(pose_e):
            row[f"tcp_e_{idx}"] = value

        rows.append(row)

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else []
    with output_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    label_csv = (root / args.label_csv).resolve()
    if not label_csv.exists():
        with label_csv.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "sample_id",
                    "image_path",
                    "sequence_id",
                    "frame_index",
                    "in_plane_label",
                    "labeled_at",
                ],
            )
            writer.writeheader()

    inside_count = sum(int(row["projected_tip_inside_image"]) for row in rows)
    print(f"Wrote {len(rows)} rows to {output_csv}")
    print(f"Wrote prior masks to {mask_dir}")
    print(f"Projected tip inside image: {inside_count}/{len(rows)}")
    print(f"Label CSV: {label_csv}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=".")
    parser.add_argument("--image-dir", default="image")
    parser.add_argument("--output", default="phantom_manifest.csv")
    parser.add_argument("--mask-dir", default="phantom_prior_masks")
    parser.add_argument("--label-csv", default="phantom_in_plane_labels.csv")
    parser.add_argument(
        "--filename-tip-state",
        choices=("fired", "unfired"),
        default="fired",
        help="Existing captures before 2026-08-03 used fired-length TipU.",
    )
    parser.add_argument("--unfired-retraction-mm", type=float, default=UNFIRED_NEEDLE_RETRACTION_MM)
    parser.add_argument("--biopsy-center-offset-mm", type=float, default=BIOPSY_CENTER_OFFSET_MM)
    parser.add_argument("--box-half-width-px", type=float, default=KINEMATIC_BOX_HALF_WIDTH_PX)
    parser.add_argument("--box-tip-extension-mm", type=float, default=KINEMATIC_BOX_TIP_EXTENSION_MM)
    args = parser.parse_args()
    build_manifest(args)


if __name__ == "__main__":
    main()
