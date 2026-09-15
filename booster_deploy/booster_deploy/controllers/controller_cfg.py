from typing import Callable, List, Optional
from dataclasses import MISSING

from ..utils.isaaclab.configclass import configclass


@configclass
class PrepareStateCfg:
    stiffness: List[float] = MISSING
    damping: List[float] = MISSING
    joint_pos: List[float] = MISSING


@configclass
class MujocoControllerCfg:
    init_pos: List[float] = [0.0, 0.0, 0.6]
    init_quat: List[float] = [1.0, 0.0, 0.0, 0.0]
    decimation: int = 10
    # physics_dt will automatically be set by ControllerCfg
    physics_dt: float = None  # type: ignore
    log_states: Optional[str] = None
    visualize_reference_ghost: bool = False
    ghost_rgba: List[float] = [0.2, 0.8, 0.2, 0.25]


@configclass
class BoosterRobotControllerCfg:
    low_state_dt: float = 0.002
    standing_joint_velocity_tolerance: float = 0.25
    standing_stable_ticks: int = 5


@configclass
class RobotCfg:
    name: str = MISSING

    joint_names: list[str] = MISSING
    body_names: list[str] = MISSING

    sim_joint_names: list[str] = MISSING
    sim_body_names: list[str] = MISSING

    joint_stiffness: List[float] = MISSING
    joint_damping: List[float] = MISSING

    default_joint_pos: List[float] = MISSING
    effort_limit: List[float] = MISSING

    mjcf_path: str = MISSING

    prepare_state: PrepareStateCfg = MISSING

    def __post_init__(self):
        assert (
            len(self.joint_names)
            == len(self.joint_stiffness)
            == len(self.joint_damping)
            == len(self.default_joint_pos)
            == len(self.effort_limit)
        )


@configclass
class PolicyCfg:
    """Parameters for the C++ policy node.

    Paths are relative to the deploy root. A null gain override path disables
    file-based overrides for that policy.
    """

    # "walk" runs the joystick gait and switches to squat on command; "squat"
    # runs the squat policy by itself.
    mode: str = MISSING
    walk_checkpoint_path: Optional[str] = None
    walk_gain_overrides_path: Optional[str] = None
    squat_checkpoint_path: str = "tasks/squat/models/squat.onnx"
    squat_gain_overrides_path: Optional[str] = "tasks/squat/gain_overrides.json"
    enable_safety_fallback: bool = True
    # Smallest upright gravity projection tolerated before the policy stops;
    # 0.5 is roughly 60 degrees of trunk tilt.
    min_upright_projection: float = 0.5
    standing_joint_pos_tolerance: float = 0.3
    # The node zeroes the velocity when no command arrives for this long.
    command_timeout: float = 0.5
    onnx_threads: int = 1


@configclass
class EvaluatorCfg:
    constructor: Callable = MISSING
    # Rendering
    render: bool = True


@configclass
class ControllerCfg:
    """Controller configuration class.
    """

    policy_dt: float = 0.02
    robot: RobotCfg = MISSING
    policy: PolicyCfg = MISSING

    mujoco: MujocoControllerCfg = MujocoControllerCfg()
    booster: BoosterRobotControllerCfg = BoosterRobotControllerCfg()
    evaluator: Optional[EvaluatorCfg] = None

    def __post_init__(self):
        self.mujoco.physics_dt = self.policy_dt / self.mujoco.decimation
