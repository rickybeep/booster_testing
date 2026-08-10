from __future__ import annotations
import logging
import signal
import time
import threading
import multiprocessing as mp
from multiprocessing import synchronize

import numpy as np
import torch

import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from booster_interface.msg import LowState, LowCmd, MotorCmd, RemoteControllerState

from booster_sdk.client.booster import BoosterClient, RobotMode

from .controller_cfg import ControllerCfg
from .base_controller import BaseController, BoosterRobot
from .booster_workflow import create_squat_workflow, create_walk_squat_workflow
from ..utils.synced_array import SyncedArray
from ..utils.metrics import SyncedMetrics
from ..utils.isaaclab import math as lab_math
from ..utils.remote_control_service import RemoteControlService


logger = logging.getLogger("booster_deploy")
logging.basicConfig(
    level=logging.INFO, format="[%(asctime)s] %(levelname)s %(message)s")


class CountTimer:
    def __init__(self, dt: float = 0.002, use_sim_time: bool = False):
        self.dt = dt
        # Use multiprocessing.Value for inter-process communication
        self.counter = mp.Value('L', 0)
        self.use_sim_time = use_sim_time

    def tick_timer_if_sim(self):
        if self.use_sim_time:
            with self.counter.get_lock():
                self.counter.value += 1

    def get_time(self):
        if self.use_sim_time:
            with self.counter.get_lock():
                return self.counter.value * self.dt
        else:
            return time.perf_counter()


class BoosterRobotPortal:
    synced_state: SyncedArray
    synced_command: SyncedArray
    synced_action: SyncedArray
    exit_event: synchronize.Event

    def __init__(self, cfg: ControllerCfg, use_sim_time: bool = False) -> None:
        self.cfg = cfg

        self.robot = BoosterRobot(cfg.robot)

        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        self.remoteControlService = RemoteControlService(
            controller_available=True,
            workflow_controls=True,
        )
        self.remoteControlService.print_controls(real_robot=True)
        # Use multiprocessing.Event for inter-process communication
        self.exit_event = mp.Event()
        self.inference_ready_event = mp.Event()
        self.command_published_event = mp.Event()
        self.policy_stop_event = mp.Event()
        self.squat_started_event = mp.Event()
        self.standing_pose_event = mp.Event()
        self.low_state_ready_event = mp.Event()
        self.is_running = True
        self.timer = CountTimer(
            self.cfg.booster.low_state_dt, use_sim_time=use_sim_time)

        def signal_handler(sig, frame):
            if mp.current_process().name == "MainProcess":
                print("\nKeyboard interrupt received. Shutting down...")
            self.exit_event.set()

        # Register signal handler
        signal.signal(signal.SIGINT, signal_handler)

        self._init_synced_buffer()
        self._init_metrics()

        self._cleanup_done = False
        self.inference_process = None  # Inference process reference
        self.low_cmd_publisher: rclpy.publisher.Publisher = None
        self.low_state_thread = None
        self.low_cmd_thread = None
        self.current_mode = RobotMode.UNKNOWN

        rclpy.init()
        # Initialize communication. Callbacks may start immediately and
        # reference `is_running` and `exit_event`, so ensure those are set.
        self._init_communication()
        if self.cfg.policy.start_on_walking:
            self.workflow = create_walk_squat_workflow(
                self,
                walking_mode=RobotMode.WALKING,
                custom_mode=RobotMode.CUSTOM,
            )
        else:
            self.workflow = create_squat_workflow(
                self,
                walking_mode=RobotMode.WALKING,
                custom_mode=RobotMode.CUSTOM,
                standing_stable_ticks=self.cfg.booster.standing_stable_ticks,
            )

    def _init_synced_buffer(self):
        action_dtype = np.dtype(
            [
                ("dof_target", float, (self.robot.num_joints,)),
                ("stiffness", float, (self.robot.num_joints,)),
                ("damping", float, (self.robot.num_joints,)),
            ]
        )
        self.synced_action = SyncedArray(
            "action",
            shape=(1,),
            dtype=action_dtype,
        )
        self._action_buf = np.zeros((1,), dtype=action_dtype)

        state_dtype = np.dtype(
            [
                ("root_rpy_w", float, (3,)),
                ("root_ang_vel_b", float, (3,)),
                ("root_pos_w", float, (3,)),
                ("root_lin_vel_w", float, (3,)),
                ("joint_pos", float, (self.robot.num_joints,)),
                ("joint_vel", float, (self.robot.num_joints,)),
                ("feedback_torque", float, (self.robot.num_joints,)),
            ]
        )
        self.synced_state = SyncedArray(
            "state",
            shape=(1,),
            dtype=state_dtype
        )
        self._state_buf = np.zeros((1,), dtype=state_dtype)

        command_dtype = np.dtype(
            [
                ("squat_enabled", np.bool_),
                ("velocity", np.float32, (3,)),
                ("head_target", np.float32, (2,)),
            ]
        )
        self.synced_command = SyncedArray(
            "command",
            shape=(1,),
            dtype=command_dtype,
        )

    def _init_metrics(self):
        # initialize cross-process synced metrics
        max_events = self.cfg.booster.metrics_max_events
        self.metrics = {
            "low_state_handler": SyncedMetrics(
                "low_state_handler", max_events=max_events
            ),
            "policy_step": SyncedMetrics(
                "policy_step", max_events=max_events
            ),
        }

    def _init_communication(self) -> None:
        try:
            self.client = BoosterClient()
            self.create_low_cmd_publisher("booster_deploy_low_cmd_pub")
            self._start_low_state_subscription()
            self._start_low_cmd_publisher()
        except Exception as e:
            self.logger.error(f"Failed to initialize communication: {e}")
            raise

    def _start_low_state_subscription(self) -> None:
        """Start ROS 2 subscription loop on a dedicated thread.

        The subscriptions are run on a dedicated thread and spin a
        SingleThreadedExecutor for the `/low_state` and controller topics.
        """

        def low_state_service_executor():
            self.logger.info("Low state subscription started")
            low_state_node = rclpy.create_node("booster_deploy_low_state_sub")
            low_state_node.create_subscription(
                LowState,
                "/low_state",
                self._low_state_handler,
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                ),
            )
            low_state_node.create_subscription(
                RemoteControllerState,
                "/remote_controller_state",
                self.remoteControlService.handle_controller_state,
                10,
            )

            executor = SingleThreadedExecutor()
            executor.add_node(low_state_node)

            try:
                # loop: check exit_event and rclpy.ok()
                while rclpy.ok() and not self.exit_event.is_set():
                    executor.spin_once(timeout_sec=0.1)
            except ExternalShutdownException:
                pass
            except Exception as exc:
                # Suppress RCLError if we are shutting down
                is_rcl_error = "RCLError" in type(exc).__name__
                is_shutting_down = self.exit_event.is_set() or not rclpy.ok()

                if is_rcl_error and is_shutting_down:
                    pass
                else:
                    self.logger.error(
                        "Low state subscription executor stopped: %s",
                        exc,
                        exc_info=True
                    )
            finally:
                executor.shutdown()
                low_state_node.destroy_node()
            self.logger.info("Low state subscription stopped")

        self.low_state_thread = threading.Thread(
            target=low_state_service_executor,
            name="low_state_executor",
            daemon=True,
        )
        self.low_state_thread.start()

    def _low_state_handler(self, low_state_msg: LowState):
        self.metrics["low_state_handler"].mark()
        try:
            if not self.is_running or self.exit_event.is_set():
                return

            # simulator tick
            self.timer.tick_timer_if_sim()

            # collect state data
            rpy = np.array(low_state_msg.imu_state.rpy, dtype=np.float32)
            gyro = np.array(low_state_msg.imu_state.gyro, dtype=np.float32)
            dof_pos = np.zeros(self.robot.num_joints, dtype=np.float32)
            dof_vel = np.zeros(self.robot.num_joints, dtype=np.float32)
            fb_torque = np.zeros(self.robot.num_joints, dtype=np.float32)

            for i, motor in enumerate(low_state_msg.motor_state_serial):
                dof_pos[i] = motor.q
                dof_vel[i] = motor.dq
                fb_torque[i] = motor.tau_est

            self._state_buf[0]["root_rpy_w"][:] = rpy
            self._state_buf[0]["root_ang_vel_b"][:] = gyro
            self._state_buf[0]["root_pos_w"][:] = np.zeros(
                3, dtype=np.float32
            )
            self._state_buf[0]["root_lin_vel_w"][:] = np.zeros(
                3, dtype=np.float32
            )
            self._state_buf[0]["joint_pos"][:] = dof_pos
            self._state_buf[0]["joint_vel"][:] = dof_vel
            self._state_buf[0]["feedback_torque"][:] = fb_torque
            self.synced_state.write(self._state_buf)
            self.low_state_ready_event.set()

            # Publish the latched operator command to the inference process.
            cmd = np.zeros((1,), dtype=self.synced_command.dtype)
            cmd[0]["squat_enabled"] = (
                self.remoteControlService.get_squat_enabled()
            )
            cmd[0]["head_target"] = self.remoteControlService.get_head_target()
            if self.current_mode == RobotMode.CUSTOM:
                cmd[0]["velocity"] = (
                    self.remoteControlService.get_velocity_command()
                )
            self.synced_command.write(cmd)

        except Exception as e:
            self.logger.error(f"Error in _low_state_handler: {e}")
            self.is_running = False
            self.exit_event.set()

    def create_low_cmd_publisher(self, name):
        self.publish_node = rclpy.create_node(name)
        publisher = self.publish_node.create_publisher(
            LowCmd,
            "joint_ctrl",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST
            )
        )
        self.low_cmd_publisher = publisher

        # construct low_cmd struct
        self.low_cmd = LowCmd()  # type: ignore
        self.low_cmd.cmd_type = LowCmd.CMD_TYPE_SERIAL   # type: ignore
        motor_cmd_buf = [
            MotorCmd() for _ in range(self.robot.num_joints)
        ]  # type: ignore
        for i in range(self.robot.num_joints):
            motor_cmd_buf[i].q = 0.0
            motor_cmd_buf[i].dq = 0.0
            motor_cmd_buf[i].tau = 0.0
            motor_cmd_buf[i].kp = 0.0
            motor_cmd_buf[i].kd = 0.0
            motor_cmd_buf[i].weight = 0.0
        self.low_cmd.motor_cmd.extend(motor_cmd_buf)
        self.motor_cmd = self.low_cmd.motor_cmd

        return publisher

    def _start_low_cmd_publisher(self) -> None:
        """Publish shared policy actions without forking ROS middleware."""
        def publish_commands() -> None:
            self.logger.info("Low command publisher started")
            while self.is_running and not self.exit_event.is_set():
                if not self.inference_ready_event.wait(timeout=0.1):
                    continue
                action = self.synced_action.read()[0]
                for i in range(self.robot.num_joints):
                    self.motor_cmd[i].q = float(action["dof_target"][i])
                    self.motor_cmd[i].kp = float(action["stiffness"][i])
                    self.motor_cmd[i].kd = float(action["damping"][i])
                self.low_cmd_publisher.publish(self.low_cmd)
                self.command_published_event.set()
                time.sleep(self.cfg.policy_dt)
            self.logger.info("Low command publisher stopped")

        self.low_cmd_thread = threading.Thread(
            target=publish_commands,
            name="low_cmd_publisher",
            daemon=True,
        )
        self.low_cmd_thread.start()

    def _reset_policy_session(self) -> None:
        """Clear cross-process state before starting learned walking."""
        self._set_squat_command(False)
        self.inference_ready_event.clear()
        self.command_published_event.clear()
        self.squat_started_event.clear()
        self.standing_pose_event.clear()
        self._action_buf.fill(0)
        self.synced_action.write(self._action_buf)

    def begin_policy(self) -> bool:
        """Start learned walking with a zero velocity command."""
        if not self.low_state_ready_event.is_set():
            return False
        if self.inference_process is not None and self.inference_process.is_alive():
            return True
        self._reset_policy_session()
        self.policy_stop_event.clear()
        self.inference_process = mp.Process(
            target=BoosterRobotPortal.inference_process_func,
            args=(
                self.cfg,
                self,
            ),
            daemon=True,
        )
        self.inference_process.start()
        self.logger.info("B pressed in Booster walking mode; learned walk policy starting")
        return True

    def policy_is_ready(self) -> bool:
        process = self.inference_process
        if process is not None and not process.is_alive():
            self.logger.error("Inference process died during initialization")
            self.exit_event.set()
            return False
        return (
            self.inference_ready_event.is_set()
            and self.command_published_event.is_set()
            and self.low_cmd_publisher.get_subscription_count() > 0
        )

    def enter_custom_mode(self) -> None:
        self.logger.info("Learned walk command ready; requesting custom mode")
        self.client.change_mode(RobotMode.CUSTOM)

    def request_crouch(self) -> None:
        self.squat_started_event.clear()
        self.standing_pose_event.clear()
        self._set_squat_command(True)
        self.logger.info("B pressed; switching from walk policy to squat policy")

    def request_stand(self) -> None:
        self._set_squat_command(False)
        self.logger.info("B pressed; standing before resuming learned walking")

    def _set_squat_command(self, enabled: bool) -> None:
        """Update both command sources before the next inference frame."""
        self.remoteControlService.set_squat_enabled(enabled)
        command = np.zeros((1,), dtype=self.synced_command.dtype)
        command[0]["squat_enabled"] = enabled
        command[0]["head_target"] = self.remoteControlService.get_head_target()
        if self.current_mode == RobotMode.CUSTOM:
            command[0]["velocity"] = (
                self.remoteControlService.get_velocity_command()
            )
        self.synced_command.write(command)

    def squat_is_commanded(self) -> bool:
        return self.remoteControlService.get_squat_enabled()

    def squat_has_started(self) -> bool:
        return self.squat_started_event.is_set()

    def standing_pose_complete(self) -> bool:
        return self.standing_pose_event.is_set()

    def robot_is_standing(self) -> bool:
        if not self.low_state_ready_event.is_set():
            return False
        state = self.synced_state.read()[0]
        max_velocity = np.max(np.abs(state["joint_vel"]))
        return bool(
            max_velocity <= self.cfg.booster.standing_joint_velocity_tolerance
        )

    def consume_crouch_request(self) -> bool:
        return self.remoteControlService.consume_crouch_request()

    def discard_crouch_request(self) -> None:
        self.remoteControlService.discard_crouch_requests()

    def _stop_inference(self) -> None:
        process = self.inference_process
        if process is None:
            return
        self.policy_stop_event.set()
        self.inference_ready_event.clear()
        self.command_published_event.clear()
        if process.is_alive():
            process.join(timeout=2.0)
        if process.is_alive():
            self.logger.warning("Inference process did not stop, terminating")
            process.terminate()
            process.join(timeout=1.0)
        self.inference_process = None

    def cancel_policy(self) -> None:
        self._set_squat_command(False)
        self._stop_inference()

    def finish_squat(self) -> None:
        self.logger.info("Robot is fully standing; returning to walking mode")
        self.client.change_mode(RobotMode.WALKING)
        self.current_mode = RobotMode.WALKING
        self.cancel_policy()

    # Compatibility aliases for callers from the earlier squat-only workflow.
    begin_squat = begin_policy
    cancel_squat = cancel_policy

    def cleanup(self) -> None:
        """Clean up resources (idempotent)."""
        if self._cleanup_done:
            return
        self._cleanup_done = True

        self.logger.info("Doing cleanup...")

        # stop threads and processes
        self.is_running = False
        self.exit_event.set()
        self.policy_stop_event.set()

        # wait for inference process
        if (
            self.inference_process is not None
            and self.inference_process.is_alive()
        ):
            self.logger.info("Waiting for inference process...")
            self.inference_process.join(timeout=2.0)
            if self.inference_process.is_alive():
                self.logger.warning(
                    "Inference process did not stop, terminating...")
                self.inference_process.terminate()
                self.inference_process.join(timeout=1.0)

        # close communications
        try:
            self.remoteControlService.close()
        except Exception as e:
            self.logger.error(f"Error closing remote control: {e}")

        if self.low_cmd_thread is not None and self.low_cmd_thread.is_alive():
            self.low_cmd_thread.join(timeout=2.0)

        try:
            thread = self.low_state_thread
            if thread is not None and thread.is_alive():
                thread.join(timeout=2.0)

        except Exception as e:
            self.logger.error(f"Error waiting for low state thread: {e}")

        if rclpy.ok():
            rclpy.shutdown()

        self.logger.info("Cleanup complete")

        # Print synced metrics summary to stdout
        for name, metric in self.metrics.items():
            stats = metric.compute()
            print(
                f"METRICS {name}: count={stats['count']}, "
                f"freq={stats['freq_hz']:.3f}Hz, "
                f"mean_period={stats['mean_period_s']}, "
                f"min={stats['min_period_s']}, max={stats['max_period_s']}"
            )

    def run(self):
        """Start learned walking and tick policy switches at 10 Hz."""

        print("Initialization complete.")
        print(self.remoteControlService.get_operation_hint())
        while self.is_running and not self.exit_event.is_set():
            try:
                response = self.client.get_mode()
                self.current_mode = response.mode_enum() or RobotMode.UNKNOWN
                self.workflow.tick()
            except Exception:
                self.logger.exception("Workflow tick failed")
                self.exit_event.set()
                break
            time.sleep(0.1)

        # Shutdown never changes the robot's high-level mode implicitly.
        self.cancel_policy()

    def __enter__(self) -> BoosterRobotPortal:
        return self

    def __exit__(self, *args) -> None:
        self.cleanup()

    @staticmethod
    def inference_process_func(
        cfg: ControllerCfg,
        portal: BoosterRobotPortal,
    ) -> None:
        BoosterRobotController(cfg, portal).run()
        portal.logger.info("Inference process stopped.")


class BoosterRobotController(BaseController):
    '''Controller for Booster robots. Note that this controller runs in a
    separate process forked by BoosterRobotPortal.
    '''
    def __init__(self, cfg: ControllerCfg, portal: BoosterRobotPortal) -> None:
        super().__init__(cfg)
        self.portal = portal

    def update_policy_command(self):
        cmd = self.portal.synced_command.read()[0]
        self.squat_enabled = bool(cmd["squat_enabled"])
        self.velocity_command = tuple(float(value) for value in cmd["velocity"])
        self.head_target = tuple(float(value) for value in cmd["head_target"])

    def update_state(self) -> None:
        state = self.portal.synced_state.read()[0]

        self.robot.data.joint_pos = torch.from_numpy(
            state["joint_pos"]).to(dtype=torch.float32).to(
                self.robot.data.device)
        self.robot.data.joint_vel = torch.from_numpy(
            state["joint_vel"]).to(dtype=torch.float32).to(
                self.robot.data.device)
        self.robot.data.feedback_torque = torch.from_numpy(
            state["feedback_torque"]).to(dtype=torch.float32).to(
                self.robot.data.device)
        self.robot.data.root_pos_w = torch.from_numpy(
            state["root_pos_w"]).to(dtype=torch.float32).to(
                self.robot.data.device)
        rpy_t = torch.from_numpy(state["root_rpy_w"]).to(
            dtype=torch.float32).to(self.robot.data.device)
        self.robot.data.root_quat_w = lab_math.quat_from_euler_xyz(
            *rpy_t
        ).squeeze()
        self.robot.data.root_lin_vel_b = lab_math.quat_apply_inverse(
            self.robot.data.root_quat_w,
            torch.from_numpy(
                state["root_lin_vel_w"]).to(dtype=torch.float32).to(
                    self.robot.data.device)
        )
        self.robot.data.root_ang_vel_b = torch.from_numpy(
            state["root_ang_vel_b"]).to(dtype=torch.float32).to(
                self.robot.data.device)

    def ctrl_step(self, dof_targets: torch.Tensor) -> None:
        action = np.zeros((1,), dtype=self.portal.synced_action.dtype)
        action[0]["dof_target"] = dof_targets.cpu().numpy()
        action[0]["stiffness"] = self.robot.joint_stiffness.cpu().numpy()
        action[0]["damping"] = self.robot.joint_damping.cpu().numpy()
        self.portal.synced_action.write(action)

    def stop(self):
        super().stop()
        self.portal.exit_event.set()

    def run(self):
        self.update_state()
        self.update_policy_command()
        self.start()

        next_inference_time = self.portal.timer.get_time()
        first_command_published = False
        while self.is_running and not self.portal.exit_event.is_set():
            if self.portal.policy_stop_event.is_set():
                break
            if self.portal.timer.get_time() < next_inference_time:
                time.sleep(0.0002)
                continue
            next_inference_time += self.cfg.policy_dt

            self.update_state()
            self.update_policy_command()
            self.portal.metrics["policy_step"].mark()
            dof_targets = self.policy_step()
            policy_reports_started = getattr(
                self.policy, "squat_has_started", lambda: False
            )()
            policy_reports_complete = getattr(
                self.policy, "squat_cycle_complete", lambda: False
            )()
            is_standing = getattr(
                self.policy, "is_standing_pose", lambda: False
            )()
            if policy_reports_complete:
                self.portal.standing_pose_event.set()
            elif policy_reports_started or (
                self.squat_enabled and not is_standing
            ):
                self.portal.squat_started_event.set()
                self.portal.standing_pose_event.clear()
            elif (
                not self.squat_enabled
                and self.portal.squat_started_event.is_set()
                and is_standing
            ):
                self.portal.standing_pose_event.set()
            self.ctrl_step(dof_targets)
            if not first_command_published:
                # The parent publisher will acknowledge this shared action;
                # CUSTOM is gated on both events.
                self.portal.inference_ready_event.set()
                first_command_published = True

        if not self.portal.policy_stop_event.is_set():
            self.portal.exit_event.set()
