"""Discrete visual feedback for the needle pitch.

This module deliberately contains no model loading and no motor commands.  A
future inference adapter may feed a measured UV needle vector into this
controller and decide whether to execute the returned, bounded pitch increment.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Sequence

import numpy as np


class PitchFeedbackStatus(str, Enum):
    READY = "ready"
    MODEL_NOT_READY = "model_not_ready"
    ADJUST_PITCH = "adjust_pitch"
    WITHIN_DEADBAND = "within_deadband"
    TERMINATED_OUT_OF_PLANE = "terminated_out_of_plane"
    TERMINATED_INVALID_MEASUREMENT = "terminated_invalid_measurement"
    TERMINATED_STEP_LIMIT = "terminated_step_limit"
    INSERTION_PERMISSION_GRANTED = "insertion_permission_granted"
    INSERTION_PERMISSION_CONSUMED = "insertion_permission_consumed"


@dataclass(frozen=True)
class PitchFeedbackConfig:
    kp: float = 0.3
    ki: float = 0.0
    kd: float = 0.0
    deadband_deg: float = 1.0
    max_pitch_step_deg: float = 0.2
    max_feedback_steps: int = 10
    integral_limit_deg: float = 10.0
    min_in_plane_confidence: float = 0.8
    biopsy_center_offset_mm: float = 10.0


@dataclass(frozen=True)
class PitchFeedbackDecision:
    status: PitchFeedbackStatus
    delta_pitch_deg: float = 0.0
    measured_pitch_deg: Optional[float] = None
    target_pitch_deg: Optional[float] = None
    pitch_error_deg: Optional[float] = None
    measured_biopsy_center_p: Optional[np.ndarray] = None
    biopsy_center_error_mm: Optional[float] = None
    feedback_step: int = 0

    @property
    def should_move(self) -> bool:
        return (
            self.status == PitchFeedbackStatus.ADJUST_PITCH
            and self.delta_pitch_deg != 0.0
        )


def normalize_angle_deg(angle_deg: float) -> float:
    """Return an angle in [-180, 180)."""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def uv_vector_to_tcp_u(vector_uv: Sequence[float]) -> np.ndarray:
    """Convert an image vector (du, dv) to the TCP_U ultrasound plane.

    Image +u is TCP_U -z and image +v is TCP_U +y.  TCP_U x is
    perpendicular to the ultrasound plane and is therefore zero for a valid
    in-plane visual measurement.
    """
    vector_uv = np.asarray(vector_uv, dtype=float).reshape(-1)
    if vector_uv.size != 2 or not np.all(np.isfinite(vector_uv)):
        raise ValueError("The measured UV vector must contain two finite values.")

    du, dv = vector_uv
    vector_u = np.array([0.0, dv, -du], dtype=float)
    norm = np.linalg.norm(vector_u)
    if norm < 1e-9:
        raise ValueError("The measured UV vector is too small.")
    return vector_u / norm


def uv_point_to_tcp_u(
    point_uv: Sequence[float],
    *,
    origin_u_px: float,
    origin_v_px: float,
    mm_per_pixel: float,
) -> np.ndarray:
    """Convert an image point (u, v) on the ultrasound plane to TCP_U."""
    point_uv = np.asarray(point_uv, dtype=float).reshape(-1)
    if point_uv.size != 2 or not np.all(np.isfinite(point_uv)):
        raise ValueError("The measured UV point must contain two finite values.")
    if not np.isfinite(mm_per_pixel) or mm_per_pixel <= 0.0:
        raise ValueError("mm_per_pixel must be positive.")

    u, v = point_uv
    return np.array([
        0.0,
        (v - float(origin_v_px)) * float(mm_per_pixel),
        (float(origin_u_px) - u) * float(mm_per_pixel),
    ], dtype=float)


def transform_point_u_to_p(point_u: Sequence[float], transform_u_from_p: np.ndarray) -> np.ndarray:
    """Transform a TCP_U point back to TCP_P using a homogeneous transform."""
    point_u = np.asarray(point_u, dtype=float).reshape(-1)
    transform_u_from_p = np.asarray(transform_u_from_p, dtype=float)
    if point_u.size != 3 or not np.all(np.isfinite(point_u)):
        raise ValueError("The TCP_U point must contain three finite values.")
    if transform_u_from_p.shape != (4, 4) or not np.all(np.isfinite(transform_u_from_p)):
        raise ValueError("transform_u_from_p must be a finite 4x4 matrix.")

    transform_p_from_u = np.linalg.inv(transform_u_from_p)
    return (transform_p_from_u @ np.append(point_u, 1.0))[:3]


def biopsy_center_from_unfired_tip_p(
    unfired_tip_p: Sequence[float],
    needle_vector_p: Sequence[float],
    *,
    center_offset_mm: float = 10.0,
) -> np.ndarray:
    """Return the fired biopsy center from the visual unfired tip and vector."""
    unfired_tip_p = np.asarray(unfired_tip_p, dtype=float).reshape(-1)
    needle_vector_p = np.asarray(needle_vector_p, dtype=float).reshape(-1)
    if unfired_tip_p.size != 3 or not np.all(np.isfinite(unfired_tip_p)):
        raise ValueError("The unfired tip must contain three finite values.")
    if needle_vector_p.size != 3 or not np.all(np.isfinite(needle_vector_p)):
        raise ValueError("The needle vector must contain three finite values.")
    if not np.isfinite(center_offset_mm):
        raise ValueError("center_offset_mm must be finite.")

    norm = np.linalg.norm(needle_vector_p)
    if norm < 1e-9:
        raise ValueError("The needle vector is too small.")
    return unfired_tip_p + float(center_offset_mm) * needle_vector_p / norm


def vector_to_pitch_deg(vector_p: Sequence[float]) -> float:
    """Calculate needle pitch using the TCP_P convention used by the UI."""
    vector_p = np.asarray(vector_p, dtype=float).reshape(-1)
    if vector_p.size != 3 or not np.all(np.isfinite(vector_p)):
        raise ValueError("The TCP_P vector must contain three finite values.")

    norm = np.linalg.norm(vector_p)
    if norm < 1e-9:
        raise ValueError("The TCP_P vector is too small.")
    vx, vy, vz = vector_p / norm
    return float(np.degrees(np.arctan2(vz, np.hypot(vx, vy))))


def measured_pitch_from_uv_deg(
    vector_uv: Sequence[float],
    rotation_u_from_p: np.ndarray,
) -> float:
    """Transform a visual in-plane vector back to TCP_P and calculate pitch."""
    rotation_u_from_p = np.asarray(rotation_u_from_p, dtype=float)
    if rotation_u_from_p.shape != (3, 3):
        raise ValueError("rotation_u_from_p must be a 3x3 rotation matrix.")
    if not np.all(np.isfinite(rotation_u_from_p)):
        raise ValueError("rotation_u_from_p must contain finite values.")

    vector_u = uv_vector_to_tcp_u(vector_uv)
    vector_p = rotation_u_from_p.T @ vector_u
    return vector_to_pitch_deg(vector_p)


class NeedlePitchFeedbackController:
    """Step-indexed PID controller that only proposes bounded Pitch changes."""

    def __init__(self, config: Optional[PitchFeedbackConfig] = None):
        self.config = config or PitchFeedbackConfig()
        self.active = False
        self.feedback_step = 0
        self.integral_deg = 0.0
        self.previous_error_deg: Optional[float] = None
        self.insertion_permission_pending = False
        self.last_status = PitchFeedbackStatus.MODEL_NOT_READY

    def reset(self) -> None:
        self.active = False
        self.feedback_step = 0
        self.integral_deg = 0.0
        self.previous_error_deg = None
        self.insertion_permission_pending = False

    def begin(self, *, model_ready: bool) -> PitchFeedbackDecision:
        """Start a session only after a real model has explicitly been loaded."""
        self.reset()
        if not model_ready:
            self.last_status = PitchFeedbackStatus.MODEL_NOT_READY
            return PitchFeedbackDecision(status=self.last_status)

        self.active = True
        self.last_status = PitchFeedbackStatus.READY
        return PitchFeedbackDecision(status=self.last_status)

    def _terminate(self, status: PitchFeedbackStatus) -> PitchFeedbackDecision:
        step = self.feedback_step
        self.reset()
        self.last_status = status
        return PitchFeedbackDecision(status=status, feedback_step=step)

    def grant_single_insertion_permission(self) -> bool:
        """Arm one insertion after a valid in-plane feedback update.

        A boolean is used instead of a counter so repeated button presses can
        never queue multiple future insertions.
        """
        if (
            not self.active
            or self.feedback_step >= self.config.max_feedback_steps
            or self.last_status not in (
                PitchFeedbackStatus.ADJUST_PITCH,
                PitchFeedbackStatus.WITHIN_DEADBAND,
            )
        ):
            return False

        self.insertion_permission_pending = True
        self.last_status = PitchFeedbackStatus.INSERTION_PERMISSION_GRANTED
        return True

    def consume_single_insertion_permission(self) -> bool:
        """Consume one armed insertion; the caller may then move the needle."""
        if not self.active or not self.insertion_permission_pending:
            return False

        self.insertion_permission_pending = False
        self.feedback_step += 1
        self.last_status = PitchFeedbackStatus.INSERTION_PERMISSION_CONSUMED
        return True

    def update(
        self,
        *,
        measured_vector_uv: Sequence[float],
        target_vector_p: Sequence[float],
        rotation_u_from_p: np.ndarray,
        is_in_plane: bool,
        in_plane_confidence: float,
        dt: float = 1.0,
    ) -> PitchFeedbackDecision:
        """Return a Pitch proposal; never send a robot command.

        ``is_in_plane`` and ``in_plane_confidence`` must come from the future
        trained model.  An uncertain or out-of-plane result terminates the
        session conservatively.
        """
        if not self.active:
            return PitchFeedbackDecision(status=self.last_status)

        if (
            not is_in_plane
            or not np.isfinite(in_plane_confidence)
            or in_plane_confidence < self.config.min_in_plane_confidence
        ):
            return self._terminate(PitchFeedbackStatus.TERMINATED_OUT_OF_PLANE)

        if self.feedback_step >= self.config.max_feedback_steps:
            return self._terminate(PitchFeedbackStatus.TERMINATED_STEP_LIMIT)

        try:
            measured_pitch_deg = measured_pitch_from_uv_deg(
                measured_vector_uv,
                rotation_u_from_p,
            )
            target_pitch_deg = vector_to_pitch_deg(target_vector_p)
        except ValueError:
            return self._terminate(
                PitchFeedbackStatus.TERMINATED_INVALID_MEASUREMENT
            )

        pitch_error_deg = normalize_angle_deg(
            target_pitch_deg - measured_pitch_deg
        )
        if abs(pitch_error_deg) <= self.config.deadband_deg:
            self.integral_deg = 0.0
            self.previous_error_deg = pitch_error_deg
            self.last_status = PitchFeedbackStatus.WITHIN_DEADBAND
            return PitchFeedbackDecision(
                status=self.last_status,
                measured_pitch_deg=measured_pitch_deg,
                target_pitch_deg=target_pitch_deg,
                pitch_error_deg=pitch_error_deg,
                feedback_step=self.feedback_step,
            )

        if not np.isfinite(dt) or dt <= 0.0:
            return self._terminate(
                PitchFeedbackStatus.TERMINATED_INVALID_MEASUREMENT
            )

        self.integral_deg = float(np.clip(
            self.integral_deg + pitch_error_deg * dt,
            -self.config.integral_limit_deg,
            self.config.integral_limit_deg,
        ))
        derivative_deg = 0.0
        if self.previous_error_deg is not None:
            derivative_deg = (
                pitch_error_deg - self.previous_error_deg
            ) / dt

        delta_pitch_deg = (
            self.config.kp * pitch_error_deg
            + self.config.ki * self.integral_deg
            + self.config.kd * derivative_deg
        )
        delta_pitch_deg = float(np.clip(
            delta_pitch_deg,
            -self.config.max_pitch_step_deg,
            self.config.max_pitch_step_deg,
        ))

        self.previous_error_deg = pitch_error_deg
        self.last_status = PitchFeedbackStatus.ADJUST_PITCH
        return PitchFeedbackDecision(
            status=self.last_status,
            delta_pitch_deg=delta_pitch_deg,
            measured_pitch_deg=measured_pitch_deg,
            target_pitch_deg=target_pitch_deg,
            pitch_error_deg=pitch_error_deg,
            feedback_step=self.feedback_step,
        )

    def update_for_biopsy_center(
        self,
        *,
        measured_tip_uv: Sequence[float],
        measured_vector_uv: Sequence[float],
        target_biopsy_center_p: Sequence[float],
        transform_u_from_p: np.ndarray,
        origin_u_px: float,
        origin_v_px: float,
        mm_per_pixel: float,
        is_in_plane: bool,
        in_plane_confidence: float,
        dt: float = 1.0,
    ) -> PitchFeedbackDecision:
        """Return a Pitch proposal using the fired biopsy center as target.

        The model tip is the unfired needle tip.  The desired biopsy center is
        10 mm in front of that tip along the measured needle vector, so the
        pitch target is the direction from the visual unfired tip to the lesion
        point.  This still only proposes Pitch; it does not command insertion
        or use cross-track/along-track error as a controller input.
        """
        if not self.active:
            return PitchFeedbackDecision(status=self.last_status)

        try:
            transform_u_from_p = np.asarray(transform_u_from_p, dtype=float)
            if transform_u_from_p.shape != (4, 4) or not np.all(np.isfinite(transform_u_from_p)):
                raise ValueError("transform_u_from_p must be a finite 4x4 matrix.")
            rotation_u_from_p = transform_u_from_p[:3, :3]
            measured_vector_u = uv_vector_to_tcp_u(measured_vector_uv)
            measured_vector_p = rotation_u_from_p.T @ measured_vector_u
            measured_tip_u = uv_point_to_tcp_u(
                measured_tip_uv,
                origin_u_px=origin_u_px,
                origin_v_px=origin_v_px,
                mm_per_pixel=mm_per_pixel,
            )
            measured_tip_p = transform_point_u_to_p(
                measured_tip_u,
                transform_u_from_p,
            )
            measured_biopsy_center_p = biopsy_center_from_unfired_tip_p(
                measured_tip_p,
                measured_vector_p,
                center_offset_mm=self.config.biopsy_center_offset_mm,
            )
            target_biopsy_center_p = np.asarray(
                target_biopsy_center_p,
                dtype=float,
            ).reshape(-1)
            if target_biopsy_center_p.size != 3 or not np.all(np.isfinite(target_biopsy_center_p)):
                raise ValueError("The target biopsy center must contain three finite values.")
            target_vector_p = target_biopsy_center_p - measured_tip_p
            biopsy_center_error_mm = float(np.linalg.norm(
                target_biopsy_center_p - measured_biopsy_center_p
            ))
        except (ValueError, np.linalg.LinAlgError):
            return self._terminate(
                PitchFeedbackStatus.TERMINATED_INVALID_MEASUREMENT
            )

        decision = self.update(
            measured_vector_uv=measured_vector_uv,
            target_vector_p=target_vector_p,
            rotation_u_from_p=rotation_u_from_p,
            is_in_plane=is_in_plane,
            in_plane_confidence=in_plane_confidence,
            dt=dt,
        )

        return PitchFeedbackDecision(
            status=decision.status,
            delta_pitch_deg=decision.delta_pitch_deg,
            measured_pitch_deg=decision.measured_pitch_deg,
            target_pitch_deg=decision.target_pitch_deg,
            pitch_error_deg=decision.pitch_error_deg,
            measured_biopsy_center_p=measured_biopsy_center_p,
            biopsy_center_error_mm=biopsy_center_error_mm,
            feedback_step=decision.feedback_step,
        )
