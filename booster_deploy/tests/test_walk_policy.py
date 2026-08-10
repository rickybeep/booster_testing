from __future__ import annotations

import unittest

import numpy as np
import torch

from booster_deploy.controllers.base_controller import BoosterRobot
from booster_deploy.robots.booster import K1_CFG
from tasks.walk.walk import (
    HISTORY_FRAME_SIZE,
    HISTORY_LENGTH,
    WalkPolicy,
    WalkPolicyCfg,
)


class FakeController:
    def __init__(self) -> None:
        self.robot = BoosterRobot(K1_CFG)
        self.robot.data.root_quat_w = torch.tensor([1.0, 0.0, 0.0, 0.0])
        self.robot.data.joint_pos = self.robot.default_joint_pos.clone()
        self.squat_enabled = False
        self.velocity_command = (0.0, 0.0, 0.0)
        self.stopped = False

    def stop(self) -> None:
        self.stopped = True


class CapturingSession:
    def __init__(self) -> None:
        self.inputs: dict[str, np.ndarray] | None = None

    def run(
        self,
        output_names: list[str],
        inputs: dict[str, np.ndarray],
    ) -> list[np.ndarray]:
        self.inputs = inputs
        return [np.zeros((1, 22), dtype=np.float32)]


class WalkPolicyTest(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = FakeController()
        self.policy = WalkPolicy(
            WalkPolicyCfg(checkpoint_path="models/walk.onnx"),
            self.controller,
        )
        self.session = CapturingSession()
        self.policy.session = self.session

    def test_history_is_seeded_then_slides_oldest_to_newest(self) -> None:
        self.policy.inference()
        inputs = self.session.inputs
        assert inputs is not None
        history = inputs["history"]
        self.assertEqual(history.shape, (1, HISTORY_LENGTH, HISTORY_FRAME_SIZE))
        np.testing.assert_array_equal(history[0], np.repeat(history[:, :1], 50, axis=1)[0])

        first_frame = history[0, -1].copy()
        self.controller.robot.data.root_ang_vel_b[0] = 1.0
        self.policy.inference()
        inputs = self.session.inputs
        assert inputs is not None
        history = inputs["history"]
        np.testing.assert_array_equal(history[0, -2], first_frame)
        self.assertEqual(history[0, -1, 0], 1.0)

    def test_joystick_command_is_clipped_and_kept_float32(self) -> None:
        self.controller.velocity_command = (3.05, -2.0, 4.0)
        self.policy.inference()
        inputs = self.session.inputs
        assert inputs is not None
        command = inputs["instant"]
        self.assertEqual(command.dtype, np.float32)
        expected_translation = np.asarray([3.05, -2.0], dtype=np.float32)
        expected_translation *= 0.75 / np.linalg.norm(expected_translation)
        np.testing.assert_allclose(
            command,
            np.asarray(
                [[*expected_translation, 1.5]],
                dtype=np.float32,
            ),
            rtol=1e-6,
        )

        self.controller.velocity_command = (0.05, 0.0, 0.0)
        self.policy.inference()
        inputs = self.session.inputs
        assert inputs is not None
        np.testing.assert_array_equal(
            inputs["instant"],
            np.asarray([[0.2, 0.0, 0.0]], dtype=np.float32),
        )

        self.controller.velocity_command = (0.0, 0.0, 0.0)
        self.policy.inference()
        inputs = self.session.inputs
        assert inputs is not None
        np.testing.assert_array_equal(
            inputs["instant"],
            np.zeros((1, 3), dtype=np.float32),
        )

    def test_b_command_switches_to_squat_policy(self) -> None:
        walk_stiffness = self.policy.joint_stiffness.clone()
        self.controller.squat_enabled = True
        targets = self.policy.inference()
        self.assertTrue(self.policy.is_squat_active())
        self.assertEqual(tuple(targets.shape), (22,))
        self.assertTrue(bool(torch.isfinite(targets).all()))
        self.assertFalse(torch.equal(self.controller.robot.joint_stiffness, walk_stiffness))

        self.controller.squat_enabled = False
        self.policy.inference()
        self.assertTrue(self.policy.squat_cycle_complete())
        self.policy.inference()
        self.assertFalse(self.policy.is_squat_active())
        self.assertTrue(torch.equal(self.controller.robot.joint_stiffness, walk_stiffness))


if __name__ == "__main__":
    unittest.main()
