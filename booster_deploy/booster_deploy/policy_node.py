"""Configure and launch the C++ `booster_policy` inference node."""
from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
import tempfile
from pathlib import Path
from typing import Any

from .controllers.controller_cfg import ControllerCfg


DEPLOY_ROOT = Path(__file__).resolve().parents[1]
POLICY_MODES = ("walk", "squat")

logger = logging.getLogger("booster_deploy")


def _resolve_path(path: str | None) -> str:
    if path is None:
        return ""
    return str((DEPLOY_ROOT / path).resolve())


def policy_parameters(
    cfg: ControllerCfg,
    *,
    use_low_state_clock: bool = False,
) -> dict[str, Any]:
    """Return the `booster_policy` node parameters for a task config."""
    policy = cfg.policy
    if policy.mode not in POLICY_MODES:
        raise ValueError(f"Policy mode must be one of {POLICY_MODES}, got {policy.mode!r}")
    if policy.mode == "walk" and policy.walk_checkpoint_path is None:
        raise ValueError("Walk policy config needs walk_checkpoint_path")
    robot = cfg.robot
    return {
        "mode": policy.mode,
        "walk_model_path": _resolve_path(policy.walk_checkpoint_path),
        "walk_gain_overrides_path": _resolve_path(policy.walk_gain_overrides_path),
        "squat_model_path": _resolve_path(policy.squat_checkpoint_path),
        "squat_gain_overrides_path": _resolve_path(policy.squat_gain_overrides_path),
        "sit_model_path": _resolve_path(policy.sit_checkpoint_path),
        "enable_safety_fallback": bool(policy.enable_safety_fallback),
        "min_upright_projection": float(policy.min_upright_projection),
        "standing_joint_pos_tolerance": float(policy.standing_joint_pos_tolerance),
        "onnx_threads": int(policy.onnx_threads),
        "joint_names": list(robot.joint_names),
        # Floats keep the ROS parameter types as double arrays.
        "default_joint_pos": [float(value) for value in robot.default_joint_pos],
        "joint_stiffness": [float(value) for value in robot.joint_stiffness],
        "joint_damping": [float(value) for value in robot.joint_damping],
        "policy_dt": float(cfg.policy_dt),
        "low_state_dt": float(cfg.booster.low_state_dt),
        "use_low_state_clock": bool(use_low_state_clock),
        "command_timeout": float(policy.command_timeout),
    }


def make_policy_controller(cfg: ControllerCfg):
    """Build the in-process C++ policy controller used by MuJoCo and tests."""
    import booster_policy_core as core

    parameters = policy_parameters(cfg)
    config = core.PolicyConfig()
    config.mode = core.PolicyMode.WALK if parameters["mode"] == "walk" else core.PolicyMode.SQUAT
    for name in (
        "walk_model_path",
        "walk_gain_overrides_path",
        "squat_model_path",
        "squat_gain_overrides_path",
        "sit_model_path",
        "enable_safety_fallback",
        "min_upright_projection",
        "standing_joint_pos_tolerance",
        "onnx_threads",
    ):
        setattr(config, name, parameters[name])
    robot = core.RobotConfig()
    robot.joint_names = parameters["joint_names"]
    robot.default_joint_pos = parameters["default_joint_pos"]
    robot.joint_stiffness = parameters["joint_stiffness"]
    robot.joint_damping = parameters["joint_damping"]
    config.robot = robot
    return core.PolicyController(config)


class PolicyNodeProcess:
    """Run the `booster_policy` node as a child process."""

    def __init__(
        self,
        cfg: ControllerCfg,
        *,
        use_low_state_clock: bool = False,
        extra_ros_args: list[str] | None = None,
    ) -> None:
        self.parameters = policy_parameters(
            cfg, use_low_state_clock=use_low_state_clock
        )
        self.extra_ros_args = list(extra_ros_args or [])
        self.process: subprocess.Popen | None = None
        self._params_path: str | None = None

    def start(self) -> None:
        if self.is_alive():
            return
        from ament_index_python.packages import get_package_prefix

        executable = Path(
            get_package_prefix("booster_policy"), "lib", "booster_policy", "policy_node"
        )
        # JSON is valid YAML for the ROS parameter file parser.
        fd, self._params_path = tempfile.mkstemp(prefix="booster_policy_", suffix=".yaml")
        with os.fdopen(fd, "w") as params_file:
            json.dump({"booster_policy": {"ros__parameters": self.parameters}}, params_file)
        # A separate session keeps terminal Ctrl-C away from the node, so the
        # deploy process can leave CUSTOM mode before the node stops publishing.
        self.process = subprocess.Popen(
            [
                str(executable),
                "--ros-args",
                "--params-file",
                self._params_path,
                *self.extra_ros_args,
            ],
            start_new_session=True,
        )
        logger.info("Started policy node (pid %d)", self.process.pid)

    def is_alive(self) -> bool:
        return self.process is not None and self.process.poll() is None

    @property
    def returncode(self) -> int | None:
        return None if self.process is None else self.process.poll()

    def wait(self, timeout: float | None = None) -> int:
        if self.process is None:
            raise RuntimeError("Policy node has not been started")
        return self.process.wait(timeout)

    def stop(self, timeout: float = 3.0) -> None:
        process = self.process
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout)
            except subprocess.TimeoutExpired:
                logger.warning("Policy node did not stop, terminating")
                process.terminate()
                try:
                    process.wait(1.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        if self._params_path is not None:
            Path(self._params_path).unlink(missing_ok=True)
            self._params_path = None

    def __enter__(self) -> PolicyNodeProcess:
        self.start()
        return self

    def __exit__(self, *args) -> None:
        self.stop()


__all__ = [
    "DEPLOY_ROOT",
    "PolicyNodeProcess",
    "make_policy_controller",
    "policy_parameters",
]
