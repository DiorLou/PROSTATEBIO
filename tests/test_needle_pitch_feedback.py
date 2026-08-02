import unittest

import numpy as np

from core.needle_pitch_feedback import (
    NeedlePitchFeedbackController,
    PitchFeedbackConfig,
    PitchFeedbackStatus,
    biopsy_center_from_unfired_tip_p,
    measured_pitch_from_uv_deg,
)


# This rotation maps TCP_P +y (zero-pitch needle direction) to image +u,
# which is TCP_U -z. TCP_P +z maps to image +v, which is TCP_U +y.
ROTATION_U_FROM_P = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, -1.0, 0.0],
])
TRANSFORM_U_FROM_P = np.eye(4)
TRANSFORM_U_FROM_P[:3, :3] = ROTATION_U_FROM_P


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

    def test_biopsy_center_is_ahead_of_unfired_tip(self):
        center = biopsy_center_from_unfired_tip_p(
            [1.0, 2.0, 3.0],
            [0.0, 3.0, 4.0],
            center_offset_mm=10.0,
        )
        np.testing.assert_allclose(center, [1.0, 8.0, 11.0])

    def test_biopsy_center_target_drives_pitch_goal(self):
        controller = NeedlePitchFeedbackController(
            PitchFeedbackConfig(
                kp=1.0,
                max_pitch_step_deg=0.2,
                biopsy_center_offset_mm=10.0,
            )
        )
        controller.begin(model_ready=True)

        target_pitch_deg = 12.0
        target_biopsy_center_p = [
            0.0,
            10.0 * np.cos(np.radians(target_pitch_deg)),
            10.0 * np.sin(np.radians(target_pitch_deg)),
        ]
        decision = controller.update_for_biopsy_center(
            measured_tip_uv=[406.0, 420.0 + 10.0 / (66.0 / 420.0)],
            measured_vector_uv=[1.0, 0.0],
            target_biopsy_center_p=target_biopsy_center_p,
            transform_u_from_p=TRANSFORM_U_FROM_P,
            origin_u_px=406.0,
            origin_v_px=420.0 + 10.0 / (66.0 / 420.0),
            mm_per_pixel=66.0 / 420.0,
            is_in_plane=True,
            in_plane_confidence=0.99,
        )

        self.assertEqual(decision.status, PitchFeedbackStatus.ADJUST_PITCH)
        self.assertAlmostEqual(decision.measured_pitch_deg, 0.0)
        self.assertAlmostEqual(decision.target_pitch_deg, target_pitch_deg)
        self.assertAlmostEqual(decision.pitch_error_deg, target_pitch_deg)
        self.assertEqual(decision.delta_pitch_deg, 0.2)
        np.testing.assert_allclose(
            decision.measured_biopsy_center_p,
            [0.0, 10.0, 0.0],
            atol=1e-7,
        )
        self.assertGreater(decision.biopsy_center_error_mm, 0.0)

    def test_each_insertion_requires_a_new_external_permission(self):
        controller = NeedlePitchFeedbackController()
        controller.begin(model_ready=True)
        controller.update(
            measured_vector_uv=[1.0, 0.0],
            target_vector_p=[0.0, 1.0, 0.0],
            rotation_u_from_p=ROTATION_U_FROM_P,
            is_in_plane=True,
            in_plane_confidence=0.99,
        )

        self.assertTrue(controller.grant_single_insertion_permission())
        self.assertTrue(controller.consume_single_insertion_permission())
        self.assertFalse(controller.consume_single_insertion_permission())
        self.assertEqual(controller.feedback_step, 1)

        # A new valid model result and a new button press are both required.
        controller.update(
            measured_vector_uv=[1.0, 0.0],
            target_vector_p=[0.0, 1.0, 0.0],
            rotation_u_from_p=ROTATION_U_FROM_P,
            is_in_plane=True,
            in_plane_confidence=0.99,
        )
        self.assertFalse(controller.consume_single_insertion_permission())
        self.assertTrue(controller.grant_single_insertion_permission())
        self.assertTrue(controller.consume_single_insertion_permission())
        self.assertEqual(controller.feedback_step, 2)

    def test_out_of_plane_result_clears_an_unused_permission(self):
        controller = NeedlePitchFeedbackController()
        controller.begin(model_ready=True)
        controller.update(
            measured_vector_uv=[1.0, 0.0],
            target_vector_p=[0.0, 1.0, 0.0],
            rotation_u_from_p=ROTATION_U_FROM_P,
            is_in_plane=True,
            in_plane_confidence=0.99,
        )
        self.assertTrue(controller.grant_single_insertion_permission())

        controller.update(
            measured_vector_uv=[1.0, 0.0],
            target_vector_p=[0.0, 1.0, 0.0],
            rotation_u_from_p=ROTATION_U_FROM_P,
            is_in_plane=False,
            in_plane_confidence=0.99,
        )
        self.assertFalse(controller.consume_single_insertion_permission())


if __name__ == "__main__":
    unittest.main()
