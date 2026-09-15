from __future__ import annotations

import json
import math
import tempfile
import unittest

import numpy as np
import pytest

core = pytest.importorskip("booster_policy_core")
ort = pytest.importorskip("onnxruntime")

from booster_deploy.policy_node import (  # noqa: E402
    DEPLOY_ROOT,
    make_policy_controller,
    policy_parameters,
)
from tasks.squat.squat import K1SquatControllerCfg  # noqa: E402
from tasks.walk.walk import K1WalkControllerCfg  # noqa: E402


HISTORY_LENGTH = core.WALK_HISTORY_LENGTH
OBSERVATION_SIZE = core.WALK_OBSERVATION_SIZE
DEFAULT_POS = np.asarray(K1WalkControllerCfg().robot.default_joint_pos, dtype=np.float32)
UPRIGHT = (0.0, 0.0, -1.0)


def _metadata_floats(session, key: str) -> np.ndarray:
    values = session.get_modelmeta().custom_metadata_map[key].split(",")
    return np.asarray([float(value) for value in values], dtype=np.float32)


class WalkPolicyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.session = ort.InferenceSession(
            str(DEPLOY_ROOT / "tasks/walk/models/gait_history.onnx"),
            providers=["CPUExecutionProvider"],
        )

    def setUp(self) -> None:
        self.policy = make_policy_controller(K1WalkControllerCfg())
        self.joint_pos = DEFAULT_POS.copy()
        self.joint_vel = np.zeros(22, dtype=np.float32)
        self.ang_vel = np.zeros(3, dtype=np.float32)
        self.velocity = (0.0, 0.0, 0.0)
        self.head = (0.0, 0.0)

    def step(self, squat: bool = False):
        return self.policy.step(
            self.ang_vel,
            UPRIGHT,
            self.joint_pos,
            self.joint_vel,
            self.velocity,
            self.head,
            squat,
        )

    def test_observation_history_is_seeded_and_contains_current_state(self) -> None:
        self.joint_pos[10] += 0.7
        self.joint_pos[11] -= 0.2
        self.joint_vel[10] = 1.0
        self.joint_vel[11] = -1.0
        self.step()
        history = self.policy.walk_history
        self.assertEqual(history.shape, (HISTORY_LENGTH, OBSERVATION_SIZE))
        np.testing.assert_array_equal(history[0], history[-1])
        np.testing.assert_allclose(history[-1, 16:18], [0.7, -0.2], atol=1e-6)
        np.testing.assert_array_equal(history[-1, 38:40], [1.0, -1.0])
        np.testing.assert_array_equal(history[-1, 3:6], UPRIGHT)

        self.ang_vel[0] = 1.0
        self.step()
        history = self.policy.walk_history
        self.assertEqual(history[-2, 0], 0.0)
        self.assertEqual(history[-1, 0], 1.0)

    def test_joystick_command_is_clipped_as_instant_input(self) -> None:
        self.velocity = (3.05, -2.0, 4.0)
        self.step()
        np.testing.assert_allclose(
            self.policy.walk_command_input,
            [core.MAX_TRANSLATIONAL_SPEED, -core.MAX_TRANSLATIONAL_SPEED, core.MAX_YAW_RATE],
        )
        self.velocity = (0.05, 0.0, 0.0)
        self.step()
        np.testing.assert_array_equal(
            self.policy.walk_command_input, np.asarray([0.05, 0.0, 0.0], np.float32)
        )

    def test_targets_match_onnxruntime_on_the_same_inputs(self) -> None:
        rng = np.random.default_rng(3)
        default = _metadata_floats(self.session, "default_joint_pos")
        scale = _metadata_floats(self.session, "action_scale")
        previous_actions = np.zeros(22, dtype=np.float32)
        for _ in range(5):
            self.joint_pos = (DEFAULT_POS + rng.normal(0, 0.1, 22)).astype(np.float32)
            self.joint_vel = rng.normal(0, 1, 22).astype(np.float32)
            self.velocity = (0.4, -0.1, 0.3)
            targets, _, _ = self.step()
            history = self.policy.walk_history
            np.testing.assert_array_equal(history[-1, 50:72], previous_actions)
            (actions,) = self.session.run(
                ["actions"],
                {
                    "history": history[None],
                    "instant": self.policy.walk_command_input[None],
                },
            )
            previous_actions = actions[0]
            np.testing.assert_allclose(targets[2:], (default + scale * actions[0])[2:], atol=1e-5)
        np.testing.assert_array_equal(targets[:2], [0.0, 0.0])

    def test_walk_gains_include_overrides(self) -> None:
        _, stiffness, damping = self.step()
        self.assertEqual(stiffness[0], 10.0)
        self.assertEqual(stiffness[1], 10.0)
        self.assertEqual(stiffness[10], 80.0)
        self.assertEqual(stiffness[15], 45.0)
        self.assertEqual(damping[14], 2.0)
        self.assertEqual(damping[21], 2.5)

    def test_history_rolls_and_resets_after_squat_cycle(self) -> None:
        for index in range(HISTORY_LENGTH + 2):
            self.ang_vel[0] = index
            self.step()
        np.testing.assert_array_equal(
            self.policy.walk_history[:, 0], np.arange(2, HISTORY_LENGTH + 2)
        )

        walk_stiffness, _ = self.policy.walk_gains
        _, stiffness, _ = self.step(squat=True)
        self.assertEqual(self.policy.active_policy, core.ActivePolicy.SQUAT)
        self.assertFalse(np.array_equal(stiffness, walk_stiffness))
        self.assertTrue(self.policy.squat_started)

        # The pose is still at the default stance, so standing completes at once.
        self.step(squat=False)
        self.assertTrue(self.policy.standing_pose_complete)
        _, stiffness, _ = self.step(squat=False)
        self.assertEqual(self.policy.active_policy, core.ActivePolicy.WALK)
        np.testing.assert_array_equal(stiffness, walk_stiffness)
        history = self.policy.walk_history
        np.testing.assert_array_equal(history[:, 50:], 0)
        np.testing.assert_array_equal(history, np.broadcast_to(history[-1], history.shape))

    def test_walk_output_uses_manual_head_target(self) -> None:
        self.head = (0.4, -0.2)
        targets, _, _ = self.step()
        self.assertAlmostEqual(float(targets[0]), 0.4, places=6)
        self.assertAlmostEqual(float(targets[1]), -0.2, places=6)

    def test_tilt_triggers_upright_fault(self) -> None:
        self.assertFalse(self.policy.upright_fault)
        gravity = core.projected_gravity_from_rpy(1.2, 0.0, 0.0)
        self.policy.step(
            self.ang_vel, gravity, self.joint_pos, self.joint_vel, self.velocity, self.head, False
        )
        self.assertTrue(self.policy.upright_fault)
        self.policy.reset()
        self.assertFalse(self.policy.upright_fault)

    def test_invalid_state_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.policy.step(
                self.ang_vel, UPRIGHT, self.joint_pos[:21], self.joint_vel, self.velocity,
                self.head, False,
            )


class SquatPolicyTest(unittest.TestCase):
    def test_squat_cycle_events_and_onnx_parity(self) -> None:
        policy = make_policy_controller(K1SquatControllerCfg())
        session = ort.InferenceSession(
            str(DEPLOY_ROOT / "tasks/squat/models/squat.onnx"),
            providers=["CPUExecutionProvider"],
        )
        default = _metadata_floats(session, "default_joint_pos")
        scale = _metadata_floats(session, "action_scale")
        scale[:2] *= 0.1  # Head actions are damped in deployment.

        joint_pos = DEFAULT_POS.copy()
        joint_pos[13] += 0.8  # Knee bent: not a standing pose.
        joint_vel = np.zeros(22, dtype=np.float32)
        last_action = np.zeros(22, dtype=np.float32)
        for squat in (True, True, False):
            targets, _, _ = policy.step(
                np.zeros(3), UPRIGHT, joint_pos, joint_vel, (0, 0, 0), (0, 0), squat
            )
            obs = np.concatenate(
                [np.zeros(3), UPRIGHT, joint_pos - default, joint_vel, last_action, [squat]]
            ).astype(np.float32)
            (actions,) = session.run(["actions"], {"obs": obs[None]})
            last_action = actions[0]
            np.testing.assert_allclose(targets, default + scale * actions[0], atol=1e-5)
            self.assertEqual(policy.active_policy, core.ActivePolicy.SQUAT)
        self.assertTrue(policy.squat_started)
        self.assertFalse(policy.standing_pose_complete)

        joint_pos = DEFAULT_POS.copy()
        policy.step(np.zeros(3), UPRIGHT, joint_pos, joint_vel, (0, 0, 0), (0, 0), False)
        self.assertTrue(policy.standing_pose_complete)

        # A new crouch request clears the previous cycle.
        policy.step(np.zeros(3), UPRIGHT, joint_pos, joint_vel, (0, 0, 0), (0, 0), True)
        self.assertTrue(policy.squat_started)
        self.assertFalse(policy.standing_pose_complete)


class ConfigTest(unittest.TestCase):
    def test_projected_gravity_matches_rpy_rotation(self) -> None:
        roll, pitch, yaw = 0.3, -0.2, 1.1
        expected = [
            math.sin(pitch),
            -math.cos(pitch) * math.sin(roll),
            -math.cos(pitch) * math.cos(roll),
        ]
        np.testing.assert_allclose(core.projected_gravity_from_rpy(roll, pitch, yaw), expected, atol=1e-6)
        half = yaw / 2
        np.testing.assert_allclose(
            core.projected_gravity_from_quaternion(math.cos(half), 0, 0, math.sin(half)),
            [0, 0, -1],
            atol=1e-6,
        )

    def test_node_parameters_resolve_task_paths(self) -> None:
        params = policy_parameters(K1WalkControllerCfg())
        self.assertEqual(params["mode"], "walk")
        self.assertEqual(
            params["walk_model_path"], str(DEPLOY_ROOT / "tasks/walk/models/gait_history.onnx")
        )
        self.assertEqual(len(params["joint_names"]), 22)
        self.assertTrue(all(isinstance(v, float) for v in params["default_joint_pos"]))
        squat = policy_parameters(K1SquatControllerCfg())
        self.assertEqual((squat["mode"], squat["walk_model_path"]), ("squat", ""))

    def test_invalid_gain_overrides_are_rejected(self) -> None:
        cfg = K1WalkControllerCfg()
        with tempfile.NamedTemporaryFile("w", suffix=".json") as overrides:
            json.dump({"stiffness": {"Not_A_Joint": 1.0}}, overrides)
            overrides.flush()
            cfg.policy.walk_gain_overrides_path = overrides.name
            with self.assertRaisesRegex(ValueError, "Not_A_Joint"):
                make_policy_controller(cfg)

    def test_missing_model_is_rejected(self) -> None:
        cfg = K1WalkControllerCfg()
        cfg.policy.walk_checkpoint_path = "tasks/walk/models/missing.onnx"
        with self.assertRaisesRegex(ValueError, "not found"):
            make_policy_controller(cfg)


if __name__ == "__main__":
    unittest.main()
