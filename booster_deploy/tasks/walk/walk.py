from __future__ import annotations

from booster_deploy.controllers.controller_cfg import (
    ControllerCfg,
    MujocoControllerCfg,
    PolicyCfg,
)
from booster_deploy.robots.booster import K1_CFG
from booster_deploy.utils.isaaclab.configclass import configclass


@configclass
class K1WalkControllerCfg(ControllerCfg):
    """History-encoder joystick gait with in-process squat and sit policies.

    Inference runs in the C++ `booster_policy` node; see `PolicyCfg`.
    """

    robot = K1_CFG
    policy: PolicyCfg = PolicyCfg(
        mode="walk",
        walk_checkpoint_path="tasks/walk/models/gait.onnx",
        walk_gain_overrides_path="tasks/walk/gain_overrides.json",
    )
    mujoco = MujocoControllerCfg(init_pos=[0.0, 0.0, 0.518])
