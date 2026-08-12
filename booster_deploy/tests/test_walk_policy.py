from __future__ import annotations

import unittest

import numpy as np
import torch

from booster_deploy.controllers.base_controller import BoosterRobot
from booster_deploy.robots.booster import K1_CFG
from tasks.walk.walk import (
    MAX_TRANSLATIONAL_SPEED,
    OBSERVATION_SIZE,
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
        self.head_target = (0.0, 0.0)
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
            WalkPolicyCfg(checkpoint_path="models/gait.onnx"),
            self.controller,
        )
        self.session = CapturingSession()
        self.policy.session = self.session

    def test_observation_is_history_free_and_contains_current_state(self) -> None:
        self.controller.robot.data.joint_pos[0] = 0.7
        self.controller.robot.data.joint_pos[1] = -0.2
        self.controller.robot.data.joint_vel[0] = 1.0
        self.controller.robot.data.joint_vel[1] = -1.0
        self.policy.inference()
        inputs = self.session.inputs
        assert inputs is not None
        observation = inputs["obs"]
        self.assertEqual(observation.shape, (1, OBSERVATION_SIZE))
        np.testing.assert_array_equal(observation[0, 6:8], [0.0, 0.0])
        np.testing.assert_array_equal(observation[0, 28:30], [0.0, 0.0])

        self.controller.robot.data.root_ang_vel_b[0] = 1.0
        self.policy.inference()
        inputs = self.session.inputs
        assert inputs is not None
        observation = inputs["obs"]
        self.assertEqual(observation[0, 0], 1.0)

    def test_joystick_command_is_clipped_in_observation(self) -> None:
        self.controller.velocity_command = (3.05, -2.0, 4.0)
        self.policy.inference()
        inputs = self.session.inputs
        assert inputs is not None
        command = inputs["obs"][0, -3:]
        self.assertEqual(command.dtype, np.float32)
        expected_translation = np.asarray([3.05, -2.0], dtype=np.float32)
        expected_translation *= MAX_TRANSLATIONAL_SPEED / np.linalg.norm(
            expected_translation
        )
        np.testing.assert_allclose(
            command,
            np.asarray([*expected_translation, 1.5], dtype=np.float32),
            rtol=1e-6,
        )

        self.controller.velocity_command = (0.05, 0.0, 0.0)
        self.policy.inference()
        inputs = self.session.inputs
        assert inputs is not None
        np.testing.assert_array_equal(
            inputs["obs"][0, -3:],
            np.asarray([0.2, 0.0, 0.0], dtype=np.float32),
        )

        self.controller.velocity_command = (0.0, 0.0, 0.0)
        self.policy.inference()
        inputs = self.session.inputs
        assert inputs is not None
        np.testing.assert_array_equal(
            inputs["obs"][0, -3:],
            np.zeros(3, dtype=np.float32),
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

    def test_walk_output_uses_manual_head_target(self) -> None:
        self.assertEqual(float(self.policy.joint_stiffness[0]), 8.0)
        self.assertEqual(float(self.policy.joint_stiffness[1]), 8.0)
        self.controller.head_target = (0.4, -0.2)
        targets = self.policy.inference()
        self.assertAlmostEqual(float(targets[0]), 0.4)
        self.assertAlmostEqual(float(targets[1]), -0.2)


if __name__ == "__main__":
    unittest.main()
