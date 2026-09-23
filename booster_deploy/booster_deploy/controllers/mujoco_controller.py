from __future__ import annotations

import os
from pathlib import Path
from time import sleep
import numpy as np
import mujoco
import mujoco.viewer
import booster_policy_core
from ..policy_node import make_policy_controller
from ..utils.remote_control_service import SIT_POLICY, RemoteControlService
from .controller_cfg import ControllerCfg


class MujocoController:
    """Simulate the K1 in MuJoCo with the C++ policy core in-process."""

    def __init__(self, cfg: ControllerCfg):
        self.cfg = cfg
        self._step_count = 0
        self.is_running = False
        self.policy = make_policy_controller(cfg)
        self.default_joint_pos = np.asarray(cfg.robot.default_joint_pos, dtype=np.float32)
        self.effort_limit = np.asarray(cfg.robot.effort_limit, dtype=np.float32)
        self.remote_control = RemoteControlService()
        self.remote_control.arm_squat_toggle()
        self.remote_control.print_controls(real_robot=False)

        mjcf_path = self._expand_assets_placeholder(cfg.robot.mjcf_path)
        self.mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.mj_model.opt.timestep = self.cfg.mujoco.physics_dt
        self.decimation = self.cfg.mujoco.decimation
        self.mj_data = mujoco.MjData(self.mj_model)
        mujoco.mj_resetData(self.mj_model, self.mj_data)

        initial_qpos = np.concatenate(
            [
                np.array(self.cfg.mujoco.init_pos, dtype=np.float32),
                np.array(self.cfg.mujoco.init_quat, dtype=np.float32),
                self.default_joint_pos,
            ]
        )
        if initial_qpos.shape != (int(self.mj_model.nq),):
            raise ValueError(
                f"Initial qpos has shape {initial_qpos.shape}; "
                f"expected ({int(self.mj_model.nq)},)"
            )
        self.mj_data.qpos[:] = initial_qpos
        self.mj_data.qvel[:] = 0.0
        mujoco.mj_forward(self.mj_model, self.mj_data)

        # render a second "ghost" robot (kinematic only) without
        # modifying the MuJoCo XML. This uses a second MjData to compute FK from
        # generalized coordinates and draws a duplicated set of geoms via
        # viewer.user_scn.
        self._ghost_mj_data = mujoco.MjData(self.mj_model)
        # Keep ghost initialized to the current simulated pose so it is valid
        # even before any policy calls set_reference_qpos().
        self._ghost_mj_data.qpos[:] = self.mj_data.qpos
        self._ghost_mj_data.qvel[:] = 0.0
        mujoco.mj_forward(self.mj_model, self._ghost_mj_data)
        self._ghost_rgba = np.array(
            self.cfg.mujoco.ghost_rgba, dtype=np.float32)
        self._ghost_scene_option = mujoco.MjvOption()

        # Reference qpos can be set explicitly by the policy.
        self._reference_qpos: np.ndarray | None = None

    def start(self):
        self._reference_qpos = None
        self._step_count = 0
        self.is_running = True
        self.policy.reset()

    def stop(self) -> None:
        self.is_running = False

    def render_reference_robot(
        self,
        viewer,
        # mj_data: mujoco.MjData,
        *,
        rgba: np.ndarray | None = None,
    ) -> None:
        """Render a kinematic robot pose into viewer.user_scn using mj_data."""
        mujoco.mjv_updateScene(
            self.mj_model,
            self._ghost_mj_data,
            self._ghost_scene_option,
            None,
            viewer.cam,
            int(mujoco.mjtCatBit.mjCAT_DYNAMIC),
            viewer.user_scn,
        )
        if rgba is None:
            rgba = self._ghost_rgba

        for i in range(viewer.user_scn.ngeom):
            viewer.user_scn.geoms[i].rgba[:] = rgba

    def set_reference_qpos(self, qpos: np.ndarray | None) -> None:
        """Set the reference generalized coordinates (qpos) for ghost rendering.

        Pass None to clear the reference.
        """
        if qpos is None:
            self._reference_qpos = None
            return

        qpos_np = np.asarray(qpos).astype(np.float32, copy=False).reshape(-1)
        if qpos_np.shape[0] != int(self.mj_model.nq):
            raise ValueError(
                f"reference qpos must have shape (nq,), got {qpos_np.shape} (nq={int(self.mj_model.nq)})"
            )
        self._reference_qpos = qpos_np.copy()
        # FK + offset
        self._ghost_mj_data.qpos[:] = self._reference_qpos
        self._ghost_mj_data.qvel[:] = 0.0
        mujoco.mj_forward(self.mj_model, self._ghost_mj_data)

    def _expand_assets_placeholder(self, path: str) -> str:
        """Replace {BOOSTER_ASSETS_DIR} placeholder in a path string.
        """
        assets_dir = os.environ.get("BOOSTER_ASSETS_DIR")
        if not assets_dir:
            raise RuntimeError(
                "BOOSTER_ASSETS_DIR must point to a booster_assets checkout "
                "when using --mujoco"
            )
        expanded = path.replace("{BOOSTER_ASSETS_DIR}", assets_dir)
        if not Path(expanded).is_file():
            raise FileNotFoundError(f"MuJoCo model not found: {expanded}")
        return expanded

    def policy_step(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Run the C++ policy on the current simulator state."""
        qpos = self.mj_data.qpos.astype(np.float32)
        qvel = self.mj_data.qvel.astype(np.float32)
        gravity = booster_policy_core.projected_gravity_from_quaternion(
            *(float(value) for value in qpos[3:7])
        )
        pose = (
            booster_policy_core.PosePolicy.SIT
            if self.remote_control.get_pose_policy() == SIT_POLICY
            else booster_policy_core.PosePolicy.SQUAT
        )
        self._step_count += 1
        targets = self.policy.step(
            qvel[3:6],
            gravity,
            qpos[7:],
            qvel[6:],
            (0.0, 0.0, 0.0),
            (0.0, 0.0),
            self.remote_control.get_squat_enabled(),
            pose,
            qpos[3:7],
        )
        if self.policy.upright_fault:
            print("\nLarge orientation error detected; stopping policy.")
            self.stop()
        return targets

    def log_states(self, dof_targets: np.ndarray) -> None:
        if self.cfg.mujoco.log_states is not None:
            if not hasattr(self, '_states'):
                self._states = {
                    'root_pos_w': [],
                    'root_quat_w': [],
                    'root_lin_vel_b': [],
                    'root_ang_vel_b': [],
                    'joint_pos': [],
                    'joint_vel': [],
                    'joint_torque': [],
                    'dof_targets': [],
                }
            base_pos_w = self.mj_data.qpos.astype(np.float32)[:3]
            base_quat = self.mj_data.qpos.astype(np.float32)[3:7]
            base_lin_vel_b = self.mj_data.qvel.astype(np.float32)[:3]
            base_ang_vel_b = self.mj_data.qvel.astype(np.float32)[3:6]
            dof_pos = self.mj_data.qpos.astype(np.float32)[7:]
            dof_vel = self.mj_data.qvel.astype(np.float32)[6:]
            dof_torque = self.mj_data.qfrc_actuator[6:].astype(np.float32)

            self._states['root_pos_w'].append(base_pos_w)
            self._states['root_quat_w'].append(base_quat)
            self._states['root_lin_vel_b'].append(base_lin_vel_b)
            self._states['root_ang_vel_b'].append(base_ang_vel_b)
            self._states['joint_pos'].append(dof_pos)
            self._states['joint_vel'].append(dof_vel)
            self._states['joint_torque'].append(dof_torque)
            self._states['dof_targets'].append(dof_targets)
            if len(self._states['root_pos_w']) % 100 == 0:
                _states = {k: np.stack(v) for k, v in self._states.items()}
                np.savez(f'{self.cfg.mujoco.log_states}.npz', **_states)
                print(f'saved {self.cfg.mujoco.log_states}.npz '
                      f'at {self._step_count} steps')

    def ctrl_step(self, dof_targets: np.ndarray, kp: np.ndarray, kd: np.ndarray):
        self.log_states(dof_targets)
        dof_pos = self.mj_data.qpos.astype(np.float32)[7:]
        dof_vel = self.mj_data.qvel.astype(np.float32)[6:]
        # ctrl_limit = [
        #     np.minimum(self.mj_model.actuator_forcerange[:, 0],
        #                self.mj_model.actuator_ctrlrange[:, 0]),
        #     np.maximum(self.mj_model.actuator_forcerange[:, 1],
        #                self.mj_model.actuator_ctrlrange[:, 1]),
        # ]
        ctrl_limit = self.effort_limit
        for i in range(self.decimation):
            self.mj_data.ctrl = np.clip(
                kp * (dof_targets - dof_pos) - kd * dof_vel,
                -ctrl_limit,
                ctrl_limit,
            )
            mujoco.mj_step(self.mj_model, self.mj_data)
            dof_pos = self.mj_data.qpos.astype(np.float32)[7:]
            dof_vel = self.mj_data.qvel.astype(np.float32)[6:]

    def run(self):
        try:
            self._run_viewer()
        finally:
            self.remote_control.close()

    def _run_viewer(self):
        with mujoco.viewer.launch_passive(
                self.mj_model, self.mj_data) as viewer:

            self.viewer = viewer
            viewer.cam.elevation = -20
            self.start()
            while viewer.is_running() and self.is_running:
                sleep(self.cfg.mujoco.physics_dt * self.cfg.mujoco.decimation)
                self.ctrl_step(*self.policy_step())

                if self.cfg.mujoco.visualize_reference_ghost:
                    # Render kinematic "ghost" robot from generalized coordinates.
                    self.render_reference_robot(
                        viewer,
                        rgba=self._ghost_rgba,
                    )

                self.viewer.cam.lookat[:] = self.mj_data.qpos.astype(np.float32)[0:3]
                self.viewer.sync()
