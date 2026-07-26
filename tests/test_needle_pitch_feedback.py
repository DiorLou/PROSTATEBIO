import unittest

import numpy as np

from core.needle_pitch_feedback import (
    NeedlePitchFeedbackController,
    PitchFeedbackConfig,
    PitchFeedbackStatus,
    measured_pitch_from_uv_deg,
)


# This rotation maps TCP_P +y (zero-pitch needle direction) to image +u,
# which is TCP_U -z. TCP_P +z maps to image +v, which is TCP_U -x.
ROTATION_U_FROM_P = np.array([
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


class NeedlePitchFeedbackTests(unittest.TestCase):
    def test_uv_measurement_recovers_physical_pitch(self):
        pitch_deg = 12.0
        vector_uv = [
            np.cos(np.radians(pitch_deg)),
            np.sin(np.radians(pitch_deg)),
        ]
        measured = measured_pitch_from_uv_deg(
            vector_uv,
            ROTATION_U_FROM_P,
        )
        self.assertAlmostEqual(measured, pitch_deg, places=7)

    def test_controller_cannot_start_without_model(self):
        controller = NeedlePitchFeedbackController()
        decision = controller.begin(model_ready=False)
        self.assertEqual(
            decision.status,
            PitchFeedbackStatus.MODEL_NOT_READY,
        )
        self.assertFalse(decision.should_move)

    def test_out_of_plane_prediction_terminates_without_motion(self):
        controller = NeedlePitchFeedbackController()
        controller.begin(model_ready=True)
        decision = controller.update(
            measured_vector_uv=[1.0, 0.0],
            target_vector_p=[0.0, 1.0, 0.0],
            rotation_u_from_p=ROTATION_U_FROM_P,
            is_in_plane=False,
            in_plane_confidence=0.99,
        )
        self.assertEqual(
            decision.status,
            PitchFeedbackStatus.TERMINATED_OUT_OF_PLANE,
        )
        self.assertFalse(controller.active)
        self.assertFalse(decision.should_move)
        self.assertEqual(decision.delta_pitch_deg, 0.0)

    def test_pitch_proposal_is_bounded_and_yaw_is_not_an_output(self):
        controller = NeedlePitchFeedbackController(
            PitchFeedbackConfig(
                kp=1.0,
                max_pitch_step_deg=0.2,
            )
        )
        controller.begin(model_ready=True)
        decision = controller.update(
            measured_vector_uv=[1.0, 0.0],
            target_vector_p=[
                0.0,
                np.cos(np.radians(10.0)),
                np.sin(np.radians(10.0)),
            ],
            rotation_u_from_p=ROTATION_U_FROM_P,
            is_in_plane=True,
            in_plane_confidence=0.99,
        )
        self.assertEqual(
            decision.status,
            PitchFeedbackStatus.ADJUST_PITCH,
        )
        self.assertAlmostEqual(decision.pitch_error_deg, 10.0)
        self.assertEqual(decision.delta_pitch_deg, 0.2)
        self.assertNotIn("yaw", decision.__dataclass_fields__)


if __name__ == "__main__":
    unittest.main()
