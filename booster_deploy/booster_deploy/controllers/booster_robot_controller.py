from __future__ import annotations
import logging
import signal
import time
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from booster_interface.msg import RemoteControllerState

from booster_sdk.client.booster import BoosterClient, RobotMode

from .controller_cfg import ControllerCfg
from .booster_workflow import create_squat_workflow, create_walk_squat_workflow
from ..policy_client import PolicyClient
from ..policy_node import PolicyNodeProcess
from ..utils.remote_control_service import RemoteControlService


logger = logging.getLogger("booster_deploy")
logging.basicConfig(
    level=logging.INFO, format="[%(asctime)s] %(levelname)s %(message)s")


class BoosterRobotPortal:
    """Joystick node and firmware-mode workflow around the C++ policy node.

    The `booster_policy` child process runs inference and publishes
    `joint_ctrl`. This process reads the controller, commands the policy
    through `PolicyClient`, and switches the firmware between WALKING and
    CUSTOM.
    """

    def __init__(self, cfg: ControllerCfg, use_sim_time: bool = False) -> None:
        self.cfg = cfg

        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        self.remoteControlService = RemoteControlService(
            controller_available=True,
            workflow_controls=True,
        )
        self.remoteControlService.print_controls(real_robot=True)
        self.exit_event = threading.Event()
        # Separate from exit_event: shutdown still needs service responses.
        self._spin_stop_event = threading.Event()
        self.is_running = True
        self.current_mode = RobotMode.UNKNOWN
        self._cleanup_done = False
        self._policy_start_logged = False

        def signal_handler(sig, frame):
            print(f"\n{signal.Signals(sig).name} received. Shutting down...")
            self.exit_event.set()

        # Register signal handlers; SIGTERM also has to stop the policy node.
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        # Simulators publishing /low_state at simulated time drive policy steps
        # from the state stream instead of the wall clock.
        self.policy_node = PolicyNodeProcess(cfg, use_low_state_clock=use_sim_time)
        self.node = None
        self.policy: PolicyClient | None = None
        self.executor = None
        self.spin_thread = None
        try:
            self.policy_node.start()
            # Keep ROS usable after Ctrl-C so shutdown can still stop the policy.
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            self._init_communication()
        except Exception:
            self.cleanup()
            raise

        if self.cfg.policy.mode == "walk":
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

    def _init_communication(self) -> None:
        self.client = BoosterClient()
        self.node = rclpy.create_node("booster_deploy_joystick")
        self.policy = PolicyClient(self.node, heartbeat_period=None)
        self.node.create_subscription(
            RemoteControllerState,
            "/remote_controller_state",
            self.remoteControlService.handle_controller_state,
            10,
        )
        self.node.create_timer(self.cfg.policy_dt, self._publish_joystick_command)

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(
            target=self._spin,
            name="joystick_executor",
            daemon=True,
        )
        self.spin_thread.start()

    def _spin(self) -> None:
        try:
            while rclpy.ok() and not self._spin_stop_event.is_set():
                self.executor.spin_once(timeout_sec=0.1)
        except ExternalShutdownException:
            pass
        except Exception:
            if not self._spin_stop_event.is_set():
                self.logger.exception("Joystick executor stopped")
                self.exit_event.set()

    def _publish_joystick_command(self) -> None:
        """Forward the latched operator command to the policy node."""
        service = self.remoteControlService
        if self.current_mode == RobotMode.CUSTOM:
            self.policy.set_velocity(*service.get_velocity_command())
        else:
            self.policy.set_velocity(0.0, 0.0, 0.0)
        self.policy.set_head_target(*service.get_head_target())
        self.policy.publish_command()

    def _check_policy_node(self) -> bool:
        if not self.policy_node.is_alive():
            self.logger.error(
                "Policy node exited with code %s", self.policy_node.returncode
            )
            self.exit_event.set()
            return False
        fault = self.policy.fault
        if fault is not None:
            self.logger.error("Policy node fault: %s", fault)
            self.exit_event.set()
            return False
        return True

    def begin_policy(self) -> bool:
        """Start learned walking with a zero velocity command."""
        if not self.policy.request_start():
            return False
        if not self._policy_start_logged:
            self.logger.info("B pressed in Booster walking mode; learned walk policy starting")
            self._policy_start_logged = True
        return True

    def policy_is_ready(self) -> bool:
        return self._check_policy_node() and self.policy.is_ready()

    def enter_custom_mode(self) -> None:
        self.logger.info("Learned walk command ready; requesting custom mode")
        self.client.change_mode(RobotMode.CUSTOM)

    def request_crouch(self) -> None:
        self.policy.squat()
        self.logger.info("B pressed; switching from walk policy to squat policy")

    def request_stand(self) -> None:
        self.policy.stand()
        self.logger.info("B pressed; standing before resuming learned walking")

    def squat_is_commanded(self) -> bool:
        return self.policy.squat_commanded

    def squat_has_started(self) -> bool:
        return self.policy.squat_started

    def standing_pose_complete(self) -> bool:
        return self.policy.standing_pose_complete

    def robot_is_standing(self) -> bool:
        status = self.policy.status
        return bool(
            status is not None
            and status.max_joint_speed
            <= self.cfg.booster.standing_joint_velocity_tolerance
        )

    def consume_crouch_request(self) -> bool:
        return self.remoteControlService.consume_crouch_request()

    def discard_crouch_request(self) -> None:
        self.remoteControlService.discard_crouch_requests()

    def cancel_policy(self) -> None:
        if self.policy is not None:
            self.policy.stop()
        self._policy_start_logged = False
        self._exit_custom_mode()

    def _exit_custom_mode(self) -> None:
        """Return firmware control to WALKING after learned CUSTOM control ends."""
        if self.current_mode != RobotMode.CUSTOM:
            return
        try:
            self.logger.info("Leaving custom mode; requesting walking mode")
            self.client.change_mode(RobotMode.WALKING)
            self.current_mode = RobotMode.WALKING
        except Exception:
            self.logger.exception("Failed to leave custom mode")

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

        self.is_running = False
        self.exit_event.set()
        self._exit_custom_mode()

        if self.policy is not None:
            try:
                self.policy.stop()
            except Exception as e:
                self.logger.error(f"Error stopping policy: {e}")

        # close communications
        try:
            self.remoteControlService.close()
        except Exception as e:
            self.logger.error(f"Error closing remote control: {e}")

        self._spin_stop_event.set()
        if self.spin_thread is not None and self.spin_thread.is_alive():
            self.spin_thread.join(timeout=2.0)
        if self.policy is not None:
            self.policy.close()
        if self.executor is not None:
            self.executor.shutdown()
        if self.node is not None:
            self.node.destroy_node()
        rclpy.try_shutdown()

        self.policy_node.stop()
        self.logger.info("Cleanup complete")

    def run(self):
        """Tick the walk/squat workflow at 10 Hz."""

        print("Initialization complete.")
        print(self.remoteControlService.get_operation_hint())
        while self.is_running and not self.exit_event.is_set():
            try:
                if not self._check_policy_node():
                    break
                response = self.client.get_mode()
                self.current_mode = response.mode_enum() or RobotMode.UNKNOWN
                self.workflow.tick()
            except Exception:
                self.logger.exception("Workflow tick failed")
                self.exit_event.set()
                break
            time.sleep(0.1)

        # Do not leave the robot in firmware CUSTOM mode after learned control
        # has stopped, including Ctrl-C and workflow errors.
        self.cancel_policy()

    def __enter__(self) -> BoosterRobotPortal:
        return self

    def __exit__(self, *args) -> None:
        self.cleanup()
