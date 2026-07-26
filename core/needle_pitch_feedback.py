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


@dataclass(frozen=True)
class PitchFeedbackDecision:
    status: PitchFeedbackStatus
    delta_pitch_deg: float = 0.0
    measured_pitch_deg: Optional[float] = None
    target_pitch_deg: Optional[float] = None
    pitch_error_deg: Optional[float] = None
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

    Image +u is TCP_U -z and image +v is TCP_U -x.  TCP_U y is
    perpendicular to the ultrasound plane and is therefore zero for a valid
    in-plane visual measurement.
    """
    vector_uv = np.asarray(vector_uv, dtype=float).reshape(-1)
    if vector_uv.size != 2 or not np.all(np.isfinite(vector_uv)):
        raise ValueError("The measured UV vector must contain two finite values.")

    du, dv = vector_uv
    vector_u = np.array([-dv, 0.0, -du], dtype=float)
    norm = np.linalg.norm(vector_u)
    if norm < 1e-9:
        raise ValueError("The measured UV vector is too small.")
    return vector_u / norm


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
        self.last_status = PitchFeedbackStatus.MODEL_NOT_READY

    def reset(self) -> None:
        self.active = False
        self.feedback_step = 0
        self.integral_deg = 0.0
        self.previous_error_deg = None

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
        self.feedback_step += 1

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
