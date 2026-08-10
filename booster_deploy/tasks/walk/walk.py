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
from tasks.squat.squat import JOINT_ALIASES, SquatPolicy, SquatPolicyCfg


INPUT_NAMES = ("history", "instant")
OUTPUT_NAMES = ("actions",)
HISTORY_LENGTH = 50
HISTORY_FRAME_SIZE = 72
COMMAND_SIZE = 3
MIN_TRANSLATIONAL_SPEED = 0.2
MAX_TRANSLATIONAL_SPEED = 1.0
MAX_YAW_RATE = 1.5
EXPECTED_OBSERVATIONS = (
    "base_ang_vel",
    "projected_gravity",
    "joint_pos",
    "joint_vel",
    "actions",
    "command",
)
EXPECTED_COMMANDS = ("twist",)
HEAD_JOINTS = ("Head_Yaw", "Head_Pitch")


def _csv(metadata: dict[str, str], key: str) -> list[str]:
    try:
        return metadata[key].split(",")
    except KeyError as exc:
        raise ValueError(f"Walk ONNX metadata is missing '{key}'") from exc


def _float_csv(metadata: dict[str, str], key: str) -> np.ndarray:
    return np.asarray([float(value) for value in _csv(metadata, key)], np.float32)


class WalkPolicy(Policy):
    """History-stacked joystick gait with an in-process squat policy."""

    def __init__(self, cfg: WalkPolicyCfg, controller: BaseController):
        super().__init__(cfg, controller)
        self.cfg = cfg
        self.robot = controller.robot
        model_path = Path(self.task_path, cfg.checkpoint_path)
        if not model_path.is_file():
            raise FileNotFoundError(f"Walk policy not found: {model_path}")

        self.session = ort.InferenceSession(
            str(model_path), providers=["CPUExecutionProvider"]
        )
        self.metadata = self.session.get_modelmeta().custom_metadata_map
        self._validate_model()

        self.policy_joint_names = _csv(self.metadata, "joint_names")
        self.default_joint_pos = _float_csv(self.metadata, "default_joint_pos")
        self.action_scale = _float_csv(self.metadata, "action_scale")
        self.policy_to_robot = np.asarray(
            [
                self.robot.cfg.joint_names.index(JOINT_ALIASES.get(name, name))
                for name in self.policy_joint_names
            ],
            dtype=np.int64,
        )
        self.head_indices = np.asarray(
            [self.policy_joint_names.index(name) for name in HEAD_JOINTS],
            dtype=np.int64,
        )
        self._validate_robot_config()
        self.joint_stiffness = torch.from_numpy(
            _float_csv(self.metadata, "joint_stiffness")
        )
        self.joint_damping = torch.from_numpy(
            _float_csv(self.metadata, "joint_damping")
        )
        self._apply_gain_overrides()

        squat_cfg = SquatPolicyCfg(
            checkpoint_path=cfg.squat_checkpoint_path,
            gain_overrides_path=cfg.squat_gain_overrides_path,
            enable_safety_fallback=cfg.enable_safety_fallback,
            min_upright_projection=cfg.min_upright_projection,
        )
        self.squat_policy = SquatPolicy(squat_cfg, controller)
        self.robot.data.to("cpu")
        self.reset()

    def _validate_model(self) -> None:
        inputs = {item.name: item for item in self.session.get_inputs()}
        outputs = {item.name: item for item in self.session.get_outputs()}
        if tuple(inputs) != INPUT_NAMES:
            raise ValueError(
                f"Unexpected walk ONNX inputs: {tuple(inputs)}; expected {INPUT_NAMES}"
            )
        if tuple(outputs) != OUTPUT_NAMES:
            raise ValueError(
                f"Unexpected walk ONNX outputs: {tuple(outputs)}; "
                f"expected {OUTPUT_NAMES}"
            )
        if outputs["actions"].shape != [1, self.robot.num_joints]:
            raise ValueError(
                f"Walk ONNX actions must be [1, {self.robot.num_joints}], "
                f"got {outputs['actions'].shape}"
            )
        if inputs["history"].shape != [1, HISTORY_LENGTH, HISTORY_FRAME_SIZE]:
            raise ValueError(
                "Walk ONNX history must be "
                f"[1, {HISTORY_LENGTH}, {HISTORY_FRAME_SIZE}], "
                f"got {inputs['history'].shape}"
            )
        if inputs["instant"].shape != [1, COMMAND_SIZE]:
            raise ValueError(
                f"Walk ONNX instant must be [1, {COMMAND_SIZE}], "
                f"got {inputs['instant'].shape}"
            )
        if tuple(_csv(self.metadata, "observation_names")) != EXPECTED_OBSERVATIONS:
            raise ValueError("Unexpected walk observation layout")
        if tuple(_csv(self.metadata, "command_names")) != EXPECTED_COMMANDS:
            raise ValueError("Unexpected walk command layout")
        if not np.all(_float_csv(self.metadata, "observation_terms_scale") == 1.0):
            raise ValueError("Walk ONNX expects unscaled observations")
        expected_history = np.asarray([50, 50, 50, 50, 50, 0], np.float32)
        history = _float_csv(self.metadata, "observation_terms_history_length")
        if not np.array_equal(history, expected_history):
            raise ValueError(f"Unexpected walk history layout: {history}")
        expected_flatten = np.asarray([0, 0, 0, 0, 0, 1], np.float32)
        flatten = _float_csv(self.metadata, "observation_terms_flatten_history_dim")
        if not np.array_equal(flatten, expected_flatten):
            raise ValueError(f"Unexpected walk history flattening: {flatten}")
        clips = _csv(self.metadata, "observation_terms_clip")
        if any(clip != "-inf;inf" for clip in clips):
            raise ValueError(f"Walk ONNX expects observation clipping: {clips}")
        if len(_csv(self.metadata, "joint_names")) != self.robot.num_joints:
            raise ValueError("Walk ONNX joint count does not match the robot")

    def _validate_robot_config(self) -> None:
        resolved_names = [
            JOINT_ALIASES.get(name, name) for name in self.policy_joint_names
        ]
        if resolved_names != self.robot.cfg.joint_names:
            raise ValueError("Walk ONNX joint order does not match the K1 config")
        if not np.allclose(
            self.default_joint_pos,
            self.robot.cfg.default_joint_pos,
            rtol=0.0,
            atol=1e-6,
        ):
            raise ValueError("K1 default_joint_pos does not match walk ONNX metadata")
        if self.action_scale.shape != (self.robot.num_joints,):
            raise ValueError("Walk ONNX action scale must contain 22 values")

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
            "stiffness": self.joint_stiffness,
            "damping": self.joint_damping,
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

    def _activate_walk(self) -> None:
        self.robot.joint_stiffness = self.joint_stiffness.clone()
        self.robot.joint_damping = self.joint_damping.clone()

    def reset(self) -> None:
        self.last_action = np.zeros((self.robot.num_joints,), dtype=np.float32)
        self.history = np.zeros((HISTORY_LENGTH, HISTORY_FRAME_SIZE), dtype=np.float32)
        self.history_initialized = False
        self.active_policy = "walk"
        self.squat_started = False
        self.squat_complete = False
        self.return_to_walk = False
        self.squat_policy.reset()
        self._activate_walk()

    def compute_history_frame(self) -> np.ndarray:
        projected_gravity = lab_math.quat_apply_inverse(
            self.robot.data.root_quat_w,
            torch.tensor([0.0, 0.0, -1.0], dtype=torch.float32),
        )
        joint_pos = self.robot.data.joint_pos[self.policy_to_robot].cpu().numpy()
        joint_vel = self.robot.data.joint_vel[self.policy_to_robot].cpu().numpy()
        joint_pos = (joint_pos - self.default_joint_pos).copy()
        joint_vel = joint_vel.copy()

        # Maelstrom's gait wrapper masks the head state while retaining all 22
        # action slots expected by this particular exported model.
        joint_pos[self.head_indices] = 0.0
        joint_vel[self.head_indices] = 0.0
        frame = np.concatenate(
            (
                self.robot.data.root_ang_vel_b.cpu().numpy(),
                projected_gravity.cpu().numpy(),
                joint_pos,
                joint_vel,
                self.last_action,
            )
        ).astype(np.float32, copy=False)
        if frame.shape != (HISTORY_FRAME_SIZE,):
            raise RuntimeError(f"Built invalid walk history frame {frame.shape}")
        return frame

    def _walk_inference(self) -> torch.Tensor:
        frame = self.compute_history_frame()
        if self.history_initialized:
            self.history[:-1] = self.history[1:]
            self.history[-1] = frame
        else:
            self.history[:] = frame
            self.history_initialized = True

        command = np.asarray(self.controller.velocity_command, dtype=np.float32).copy()
        if command.shape != (COMMAND_SIZE,):
            raise RuntimeError(f"Built invalid walk command {command.shape}")
        translational_speed = float(np.linalg.norm(command[:2]))
        if translational_speed > MAX_TRANSLATIONAL_SPEED:
            command[:2] *= MAX_TRANSLATIONAL_SPEED / translational_speed
        elif 0.0 < translational_speed < MIN_TRANSLATIONAL_SPEED:
            command[:2] *= MIN_TRANSLATIONAL_SPEED / translational_speed
        command[2] = np.clip(command[2], -MAX_YAW_RATE, MAX_YAW_RATE)
        results = self.session.run(
            list(OUTPUT_NAMES),
            {"history": self.history[None, :], "instant": command[None, :]},
        )
        action = results[0][0].astype(np.float32, copy=False)

        if self.cfg.enable_safety_fallback:
            upright = -float(frame[5])
            if upright < self.cfg.min_upright_projection:
                print("\nLarge walk orientation error detected; stopping policy.")
                self.controller.stop()

        self.last_action = action.copy()
        policy_targets = self.default_joint_pos + self.action_scale * action
        robot_targets = self.robot.default_joint_pos.clone()
        robot_targets[self.policy_to_robot] = torch.from_numpy(policy_targets)
        return robot_targets

    def _start_squat(self) -> None:
        self.active_policy = "squat"
        self.squat_started = False
        self.squat_complete = False
        self.return_to_walk = False
        self.squat_policy.reset()
        self.squat_policy.activate()

    def _resume_walk(self) -> None:
        self.active_policy = "walk"
        self.last_action.fill(0.0)
        self.history.fill(0.0)
        self.history_initialized = False
        self.return_to_walk = False
        self.squat_started = False
        self._activate_walk()

    def inference(self) -> torch.Tensor:
        if self.return_to_walk:
            self._resume_walk()
        if self.active_policy == "walk":
            if self.controller.squat_enabled:
                self._start_squat()
            else:
                return self._walk_inference()

        targets = self.squat_policy.inference()
        standing = self.squat_policy.is_standing_pose()
        if not standing:
            self.squat_started = True
        if not self.controller.squat_enabled and standing:
            self.squat_complete = True
            self.return_to_walk = True
        return targets

    def is_squat_active(self) -> bool:
        return self.active_policy == "squat"

    def squat_has_started(self) -> bool:
        return self.squat_started

    def squat_cycle_complete(self) -> bool:
        return self.squat_complete

    def is_standing_pose(self) -> bool:
        if self.active_policy == "walk":
            return True
        return self.squat_policy.is_standing_pose()


@configclass
class WalkPolicyCfg(PolicyCfg):
    constructor = WalkPolicy
    start_on_walking: bool = True
    checkpoint_path: str = MISSING
    gain_overrides_path: str | None = "gain_overrides.json"
    squat_checkpoint_path: str = "models/squat.onnx"
    squat_gain_overrides_path: str | None = "gain_overrides.json"
    min_upright_projection: float = 0.5


@configclass
class K1WalkControllerCfg(ControllerCfg):
    robot = K1_CFG
    policy: WalkPolicyCfg = WalkPolicyCfg(checkpoint_path="models/walk.onnx")
    mujoco = MujocoControllerCfg(init_pos=[0.0, 0.0, 0.518])
