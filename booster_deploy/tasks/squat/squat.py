from __future__ import annotations

from dataclasses import MISSING
import json
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch

from booster_deploy.controllers.base_controller import BaseController, Policy
from booster_deploy.controllers.controller_cfg import (
    ControllerCfg,
    MujocoControllerCfg,
    PolicyCfg,
)
from booster_deploy.robots.booster import K1_CFG
from booster_deploy.utils.isaaclab import math as lab_math
from booster_deploy.utils.isaaclab.configclass import configclass


INPUT_NAMES = ("obs", "squat_enabled", "squat_state_in")
OBSERVATION_SIZE = 122
OUTPUT_NAMES = (
    "actions",
    "squat_state_out",
    "joint_pos",
    "joint_vel",
    "body_pos_w",
    "body_quat_w",
    "body_lin_vel_w",
    "body_ang_vel_w",
)
EXPECTED_OBSERVATIONS = (
    "command",
    "motion_anchor_ori_b",
    "base_ang_vel",
    "joint_pos",
    "joint_vel",
    "actions",
    "projected_gravity",
)
JOINT_ALIASES = {
    "Head_Yaw": "AAHead_yaw",
    "Head_Pitch": "Head_pitch",
    "Left_Shoulder_Pitch": "ALeft_Shoulder_Pitch",
    "Right_Shoulder_Pitch": "ARight_Shoulder_Pitch",
}
HEAD_ACTION_SCALE_MULTIPLIER = 0.1


def _csv(metadata: dict[str, str], key: str) -> list[str]:
    try:
        return metadata[key].split(",")
    except KeyError as exc:
        raise ValueError(f"Squat ONNX metadata is missing '{key}'") from exc


def _float_csv(metadata: dict[str, str], key: str) -> np.ndarray:
    return np.asarray([float(value) for value in _csv(metadata, key)], np.float32)


class SquatPolicy(Policy):
    """Stateful deployment wrapper for the toggle-controlled squat ONNX."""

    def __init__(self, cfg: SquatPolicyCfg, controller: BaseController):
        super().__init__(cfg, controller)
        self.cfg = cfg
        self.robot = controller.robot
        model_path = Path(self.task_path, cfg.checkpoint_path)
        if not model_path.is_file():
            raise FileNotFoundError(f"Squat policy not found: {model_path}")

        self.session = ort.InferenceSession(
            str(model_path), providers=["CPUExecutionProvider"]
        )
        self.metadata = self.session.get_modelmeta().custom_metadata_map
        self._validate_model()

        self.policy_joint_names = _csv(self.metadata, "joint_names")
        self.policy_body_names = _csv(self.metadata, "body_names")
        self.anchor_index = self.policy_body_names.index(
            self.metadata["anchor_body_name"]
        )
        self.default_joint_pos = _float_csv(self.metadata, "default_joint_pos")
        self.action_scale = _float_csv(self.metadata, "action_scale")
        for joint_name in ("Head_Yaw", "Head_Pitch"):
            self.action_scale[self.policy_joint_names.index(joint_name)] *= (
                HEAD_ACTION_SCALE_MULTIPLIER
            )
        self.policy_to_robot = np.asarray(
            [
                self.robot.cfg.joint_names.index(JOINT_ALIASES.get(name, name))
                for name in self.policy_joint_names
            ],
            dtype=np.int64,
        )
        self._validate_robot_config()
        self._apply_gain_overrides()
        self.robot.data.to("cpu")
        self.reset()

    def _validate_model(self) -> None:
        inputs = {item.name: item for item in self.session.get_inputs()}
        outputs = {item.name: item for item in self.session.get_outputs()}
        if tuple(inputs) != INPUT_NAMES:
            raise ValueError(
                f"Unexpected squat ONNX inputs: {tuple(inputs)}; expected {INPUT_NAMES}"
            )
        if tuple(outputs) != OUTPUT_NAMES:
            raise ValueError(
                f"Unexpected squat ONNX outputs: {tuple(outputs)}; expected {OUTPUT_NAMES}"
            )
        if inputs["obs"].shape != [1, OBSERVATION_SIZE]:
            raise ValueError(
                "Squat ONNX obs must be "
                f"[1, {OBSERVATION_SIZE}], got {inputs['obs'].shape}"
            )
        if inputs["squat_state_in"].shape != [1, 3]:
            raise ValueError("Squat ONNX state must be [1, 3]")
        observations = tuple(_csv(self.metadata, "observation_names"))
        if observations != EXPECTED_OBSERVATIONS:
            raise ValueError(
                f"Unexpected squat observation layout: {observations}"
            )
        if len(_csv(self.metadata, "joint_names")) != self.robot.num_joints:
            raise ValueError("Squat ONNX joint count does not match the robot")

    def _validate_robot_config(self) -> None:
        resolved_names = [
            JOINT_ALIASES.get(name, name) for name in self.policy_joint_names
        ]
        if resolved_names != self.robot.cfg.joint_names:
            raise ValueError("Squat ONNX joint order does not match the K1 config")
        if not np.allclose(
            self.default_joint_pos,
            self.robot.cfg.default_joint_pos,
            rtol=0.0,
            atol=1e-6,
        ):
            raise ValueError("K1 default_joint_pos does not match squat ONNX metadata")

        # The policy artifact owns its deployment gains. Applying them here
        # prevents stale robot config values from changing policy behavior.
        self.robot.joint_stiffness = torch.from_numpy(
            _float_csv(self.metadata, "joint_stiffness")
        )
        self.robot.joint_damping = torch.from_numpy(
            _float_csv(self.metadata, "joint_damping")
        )
        if self.action_scale.shape != (self.robot.num_joints,):
            raise ValueError("Squat ONNX action scale must contain 22 values")

    def _apply_gain_overrides(self) -> None:
        if self.cfg.gain_overrides_path is None:
            return
        path = Path(self.task_path, self.cfg.gain_overrides_path)
        try:
            overrides = json.loads(path.read_text())
        except FileNotFoundError as exc:
            raise FileNotFoundError(f"Gain overrides not found: {path}") from exc
        if not isinstance(overrides, dict):
            raise ValueError(f"Gain overrides must be a JSON object: {path}")

        gain_tensors = {
            "stiffness": self.robot.joint_stiffness,
            "damping": self.robot.joint_damping,
        }
        unknown_sections = set(overrides) - set(gain_tensors)
        if unknown_sections:
            raise ValueError(
                f"Unknown gain override sections: {sorted(unknown_sections)}"
            )
        for section, joint_values in overrides.items():
            if not isinstance(joint_values, dict):
                raise ValueError(f"Gain override '{section}' must be an object")
            gain_tensor = gain_tensors[section]
            for joint_name, value in joint_values.items():
                try:
                    joint_index = self.robot.cfg.joint_names.index(joint_name)
                except ValueError as exc:
                    raise ValueError(
                        f"Unknown joint in {section} overrides: {joint_name}"
                    ) from exc
                if not isinstance(value, (int, float)) or value < 0:
                    raise ValueError(
                        f"Invalid {section} override for {joint_name}: {value}"
                    )
                gain_tensor[joint_index] = float(value)

    def reset(self) -> None:
        self.squat_state = np.asarray([[0, 0, 1]], dtype=np.int64)
        self.last_action = np.zeros((22,), dtype=np.float32)
        self.init_root_yaw_quat_w_inv = lab_math.quat_inv(
            lab_math.yaw_quat(self.robot.data.root_quat_w)
        )

        # A disabled standing call returns the exact frame-zero reference
        # embedded in the model. Its action is deliberately discarded.
        seeded = self.session.run(
            list(OUTPUT_NAMES),
            {
                "obs": np.zeros((1, OBSERVATION_SIZE), dtype=np.float32),
                "squat_enabled": np.zeros((1, 1), dtype=np.float32),
                "squat_state_in": self.squat_state,
            },
        )
        self.squat_state = seeded[1]
        self._set_reference(seeded[2:])
        self.init_reference_yaw_quat_w_inv = lab_math.quat_inv(
            lab_math.yaw_quat(
                torch.from_numpy(self.ref_body_quat_w[self.anchor_index])
            )
        )

    def _set_reference(self, arrays: list[np.ndarray]) -> None:
        (
            joint_pos,
            joint_vel,
            body_pos_w,
            body_quat_w,
            body_lin_vel_w,
            body_ang_vel_w,
        ) = arrays
        self.ref_joint_pos = joint_pos[0].astype(np.float32, copy=False)
        self.ref_joint_vel = joint_vel[0].astype(np.float32, copy=False)
        self.ref_body_pos_w = body_pos_w[0].astype(np.float32, copy=False)
        self.ref_body_quat_w = body_quat_w[0].astype(np.float32, copy=False)
        self.ref_body_lin_vel_w = body_lin_vel_w[0].astype(np.float32, copy=False)
        self.ref_body_ang_vel_w = body_ang_vel_w[0].astype(np.float32, copy=False)

    def get_initial_qpos(self) -> np.ndarray:
        """Return the embedded standing reference in MuJoCo joint order."""
        robot_ref_joints = np.empty(self.robot.num_joints, dtype=np.float32)
        robot_ref_joints[self.policy_to_robot] = self.ref_joint_pos
        return np.concatenate(
            (
                self.ref_body_pos_w[self.anchor_index],
                self.ref_body_quat_w[self.anchor_index],
                robot_ref_joints,
            )
        )

    def compute_observation(self) -> np.ndarray:
        current_quat = lab_math.quat_mul(
            self.init_root_yaw_quat_w_inv, self.robot.data.root_quat_w
        )
        reference_quat = torch.from_numpy(
            self.ref_body_quat_w[self.anchor_index]
        )
        reference_quat = lab_math.quat_mul(
            self.init_reference_yaw_quat_w_inv, reference_quat
        )

        _, anchor_quat = lab_math.subtract_frame_transforms(
            torch.zeros(3),
            current_quat,
            torch.zeros(3),
            reference_quat,
        )

        anchor_ori = lab_math.matrix_from_quat(anchor_quat)[..., :2].flatten()
        projected_gravity = lab_math.quat_apply_inverse(
            self.robot.data.root_quat_w,
            torch.tensor([0.0, 0.0, -1.0], dtype=torch.float32),
        )
        joint_pos = self.robot.data.joint_pos[self.policy_to_robot].cpu().numpy()
        joint_vel = self.robot.data.joint_vel[self.policy_to_robot].cpu().numpy()

        observation = np.concatenate(
            (
                self.ref_joint_pos,
                self.ref_joint_vel,
                anchor_ori.cpu().numpy(),
                self.robot.data.root_ang_vel_b.cpu().numpy(),
                joint_pos - self.default_joint_pos,
                joint_vel,
                self.last_action,
                projected_gravity.cpu().numpy(),
            )
        ).astype(np.float32, copy=False)
        if observation.shape != (OBSERVATION_SIZE,):
            raise RuntimeError(f"Built invalid squat observation {observation.shape}")
        return observation[None, :]

    def inference(self) -> torch.Tensor:
        observation = self.compute_observation()
        current_ref_pos = self.ref_body_pos_w[self.anchor_index].copy()
        current_ref_quat = self.ref_body_quat_w[self.anchor_index].copy()
        current_ref_joints = self.ref_joint_pos.copy()
        results = self.session.run(
            list(OUTPUT_NAMES),
            {
                "obs": observation,
                "squat_enabled": np.asarray(
                    [[float(self.controller.squat_enabled)]], dtype=np.float32
                ),
                "squat_state_in": self.squat_state,
            },
        )
        action = results[0][0].astype(np.float32, copy=False)
        self.squat_state = results[1]
        self._set_reference(results[2:])

        if hasattr(self.controller, "set_reference_qpos"):
            robot_ref_joints = np.empty(self.robot.num_joints, dtype=np.float32)
            robot_ref_joints[self.policy_to_robot] = current_ref_joints
            self.controller.set_reference_qpos(  # type: ignore[attr-defined]
                np.concatenate((current_ref_pos, current_ref_quat, robot_ref_joints))
            )

        if self.cfg.enable_safety_fallback:
            gravity = torch.tensor([0.0, 0.0, -1.0], dtype=torch.float32)
            actual_gravity = lab_math.quat_apply_inverse(
                self.robot.data.root_quat_w, gravity
            )
            reference_gravity = lab_math.quat_apply_inverse(
                torch.from_numpy(current_ref_quat), gravity
            )
            if torch.dot(actual_gravity, reference_gravity) < 0.5:
                print("\nLarge squat orientation error detected; stopping policy.")
                self.controller.stop()

        self.last_action = action.copy()
        policy_targets = self.default_joint_pos + self.action_scale * action
        robot_targets = self.robot.default_joint_pos.clone()
        robot_targets[self.policy_to_robot] = torch.from_numpy(policy_targets)
        return robot_targets


@configclass
class SquatPolicyCfg(PolicyCfg):
    constructor = SquatPolicy
    checkpoint_path: str = MISSING
    gain_overrides_path: str | None = "gain_overrides.json"


@configclass
class K1SquatControllerCfg(ControllerCfg):
    robot = K1_CFG
    policy: SquatPolicyCfg = SquatPolicyCfg(
        checkpoint_path="models/squat.onnx",
    )
    mujoco = MujocoControllerCfg(
        init_pos=[0.0, 0.0, 0.518],
        visualize_reference_ghost=True,
    )
