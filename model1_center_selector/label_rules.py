from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


YAW_OFFSETS_DEG: tuple[int, ...] = (-3, -2, -1, 0, 1, 2, 3)
CLASS_NAMES: tuple[str, ...] = (
    "YAW_M3",
    "YAW_M2",
    "YAW_M1",
    "YAW_0",
    "YAW_P1",
    "YAW_P2",
    "YAW_P3",
    "NO_VALID",
)
NO_VALID_CLASS = 7

# If several frames are IN_CENTER, choose the nearest to 0 deg.
# For equal distance, prefer the negative yaw direction.
SELECTION_PRIORITY: tuple[int, ...] = (3, 2, 4, 1, 5, 0, 6)


@dataclass(frozen=True)
class CenterFrameDecision:
    target_class: int
    target_name: str
    target_yaw_offset_deg: int | None


def normalize_center_label(label: str) -> str:
    normalized = label.strip().upper()
    aliases = {
        "IN": "IN_CENTER",
        "CENTER": "IN_CENTER",
        "1": "IN_CENTER",
        "TRUE": "IN_CENTER",
        "YES": "IN_CENTER",
        "Y": "IN_CENTER",
        "OUT": "OFF_CENTER",
        "OFF": "OFF_CENTER",
        "0": "OFF_CENTER",
        "FALSE": "OFF_CENTER",
        "NO": "OFF_CENTER",
        "N": "OFF_CENTER",
    }
    normalized = aliases.get(normalized, normalized)
    if normalized not in {"IN_CENTER", "OFF_CENTER"}:
        raise ValueError(f"Unknown center label: {label!r}")
    return normalized


def select_center_target(labels_by_class_index: Iterable[str]) -> CenterFrameDecision:
    labels = [normalize_center_label(label) for label in labels_by_class_index]
    if len(labels) != len(YAW_OFFSETS_DEG):
        raise ValueError(f"Expected 7 center labels, got {len(labels)}")

    for class_index in SELECTION_PRIORITY:
        if labels[class_index] == "IN_CENTER":
            return CenterFrameDecision(
                target_class=class_index,
                target_name=CLASS_NAMES[class_index],
                target_yaw_offset_deg=YAW_OFFSETS_DEG[class_index],
            )

    return CenterFrameDecision(
        target_class=NO_VALID_CLASS,
        target_name=CLASS_NAMES[NO_VALID_CLASS],
        target_yaw_offset_deg=None,
    )


def class_index_to_yaw_offset(class_index: int) -> int | None:
    if class_index == NO_VALID_CLASS:
        return None
    return YAW_OFFSETS_DEG[class_index]

