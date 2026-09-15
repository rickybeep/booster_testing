from __future__ import annotations

from booster_deploy.controllers.controller_cfg import (
    ControllerCfg,
    MujocoControllerCfg,
    PolicyCfg,
)
from booster_deploy.robots.booster import K1_CFG
from booster_deploy.utils.isaaclab.configclass import configclass


@configclass
class K1SquatControllerCfg(ControllerCfg):
    """Binary-command squat policy by itself.

    Inference runs in the C++ `booster_policy` node; see `PolicyCfg`.
    """

    robot = K1_CFG
    policy: PolicyCfg = PolicyCfg(mode="squat")
    mujoco = MujocoControllerCfg(
        init_pos=[0.0, 0.0, 0.518],
    )
