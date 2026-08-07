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


INPUT_NAMES = ("obs",)
OUTPUT_NAMES = ("actions",)
OBSERVATION_SIZE = 73
EXPECTED_OBSERVATIONS = (
    "base_ang_vel",
    "projected_gravity",
    "joint_pos",
    "joint_vel",
    "actions",
    "command",
)
EXPECTED_COMMANDS = ("squat",)
JOINT_ALIASES = {
    "Head_Yaw": "AAHead_yaw",
    "Head_Pitch": "Head_pitch",
    "Left_Shoulder_Pitch": "ALeft_Shoulder_Pitch",
    "Right_Shoulder_Pitch": "ARight_Shoulder_Pitch",
}
HEAD_ACTION_SCALE_MULTIPLIER = 0.1
# Squat depth shows up almost entirely in these joints: measured in MuJoCo they
# sit within 0.11 rad of the default pose while standing and 0.85 rad away at
# the bottom of a squat. Roll and ankle joints drift with stance and are a poor
# depth signal, so they are deliberately excluded.
STANDING_JOINT_PATTERNS = ("_Hip_Pitch", "_Knee_Pitch")


def _csv(metadata: dict[str, str], key: str) -> list[str]:
    try:
        return metadata[key].split(",")
    except KeyError as exc:
        raise ValueError(f"Squat ONNX metadata is missing '{key}'") from exc


def _float_csv(metadata: dict[str, str], key: str) -> np.ndarray:
    return np.asarray([float(value) for value in _csv(metadata, key)], np.float32)


class SquatPolicy(Policy):
    """Deployment wrapper for the binary-command squat ONNX.

    The policy is feed-forward: a single observation vector in, joint position
    offsets out. Squatting is requested with a scalar command appended to the
    observation, so this wrapper only owns the previous action and the
    observation layout.
    """

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
        self.standing_joint_indices = np.asarray(
            [
                index
                for index, name in enumerate(self.policy_joint_names)
                if any(pattern in name for pattern in STANDING_JOINT_PATTERNS)
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
                f"Unexpected squat ONNX outputs: {tuple(outputs)}; "
                f"expected {OUTPUT_NAMES}"
            )
        if inputs["obs"].shape != [1, OBSERVATION_SIZE]:
            raise ValueError(
                "Squat ONNX obs must be "
                f"[1, {OBSERVATION_SIZE}], got {inputs['obs'].shape}"
            )
        observations = tuple(_csv(self.metadata, "observation_names"))
        if observations != EXPECTED_OBSERVATIONS:
            raise ValueError(f"Unexpected squat observation layout: {observations}")
        commands = tuple(_csv(self.metadata, "command_names"))
        if commands != EXPECTED_COMMANDS:
            raise ValueError(f"Unexpected squat command layout: {commands}")

        # This wrapper builds raw, unscaled, history-free observations. Anything
        # else in the artifact would silently change what the policy sees.
        scales = _float_csv(self.metadata, "observation_terms_scale")
        if not np.all(scales == 1.0):
            raise ValueError(f"Squat ONNX expects observation scaling: {scales}")
        history = _float_csv(self.metadata, "observation_terms_history_length")
        if not np.all(history == 0.0):
            raise ValueError(f"Squat ONNX expects observation history: {history}")
        clips = _csv(self.metadata, "observation_terms_clip")
        if any(clip != "-inf;inf" for clip in clips):
            raise ValueError(f"Squat ONNX expects observation clipping: {clips}")
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
        self.last_action = np.zeros((self.robot.num_joints,), dtype=np.float32)
        self.squat_commanded = False

    def compute_observation(self) -> np.ndarray:
        projected_gravity = lab_math.quat_apply_inverse(
            self.robot.data.root_quat_w,
            torch.tensor([0.0, 0.0, -1.0], dtype=torch.float32),
        )
        joint_pos = self.robot.data.joint_pos[self.policy_to_robot].cpu().numpy()
        joint_vel = self.robot.data.joint_vel[self.policy_to_robot].cpu().numpy()

        observation = np.concatenate(
            (
                self.robot.data.root_ang_vel_b.cpu().numpy(),
                projected_gravity.cpu().numpy(),
                joint_pos - self.default_joint_pos,
                joint_vel,
                self.last_action,
                np.asarray([float(self.squat_commanded)]),
            )
        ).astype(np.float32, copy=False)
        if observation.shape != (OBSERVATION_SIZE,):
            raise RuntimeError(f"Built invalid squat observation {observation.shape}")
        return observation[None, :]

    def inference(self) -> torch.Tensor:
        self.squat_commanded = bool(self.controller.squat_enabled)
        observation = self.compute_observation()
        results = self.session.run(list(OUTPUT_NAMES), {"obs": observation})
        action = results[0][0].astype(np.float32, copy=False)

        if self.cfg.enable_safety_fallback:
            # observation[0, 3:6] is projected gravity; its z component is -1
            # when upright and rises toward zero as the trunk tips over.
            upright = -float(observation[0, 5])
            if upright < self.cfg.min_upright_projection:
                print("\nLarge squat orientation error detected; stopping policy.")
                self.controller.stop()

        self.last_action = action.copy()
        policy_targets = self.default_joint_pos + self.action_scale * action
        robot_targets = self.robot.default_joint_pos.clone()
        robot_targets[self.policy_to_robot] = torch.from_numpy(policy_targets)
        return robot_targets

    def is_standing_pose(self) -> bool:
        """Whether standing is commanded and the legs are back at their stance."""
        if self.squat_commanded:
            return False
        joint_pos = self.robot.data.joint_pos[self.policy_to_robot].cpu().numpy()
        error = np.abs(joint_pos - self.default_joint_pos)
        return bool(
            np.max(error[self.standing_joint_indices])
            <= self.cfg.standing_joint_pos_tolerance
        )


@configclass
class SquatPolicyCfg(PolicyCfg):
    constructor = SquatPolicy
    checkpoint_path: str = MISSING
    gain_overrides_path: str | None = "gain_overrides.json"
    # Smallest upright gravity projection tolerated before the policy stops;
    # 0.5 is roughly 60 degrees of trunk tilt.
    min_upright_projection: float = 0.5
    standing_joint_pos_tolerance: float = 0.3


@configclass
class K1SquatControllerCfg(ControllerCfg):
    robot = K1_CFG
    policy: SquatPolicyCfg = SquatPolicyCfg(
        checkpoint_path="models/squat.onnx",
    )
    mujoco = MujocoControllerCfg(
        init_pos=[0.0, 0.0, 0.518],
    )
