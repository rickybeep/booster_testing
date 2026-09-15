"""Python API for the C++ `booster_policy` walk and squat node."""
from __future__ import annotations

import logging
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from booster_policy.msg import PolicyCommand, PolicyStatus
from booster_policy.srv import StartPolicy


# Status older than this is treated as missing, e.g. after the node exits.
STATUS_TIMEOUT = 0.5

logger = logging.getLogger("booster_deploy")


class PolicyClient:
    """Start, stop, and command the `booster_policy` node.

    The client latches the velocity, head, and squat command and publishes it
    every `heartbeat_period`; the node zeroes the velocity when commands stop
    arriving. Pass `heartbeat_period=None` and call `publish_command()` to
    publish on your own schedule instead.

    Without a `node`, the client creates and spins its own. With one, the
    caller spins it; blocking calls (`start`, `stop`) must then run outside
    that executor's thread.

    Only one client should command the node at a time. The robot must be put in
    CUSTOM mode separately for the published joint commands to take effect.

    Example::

        with PolicyClient() as policy:
            policy.start()
            policy.set_velocity(0.3, 0.0, 0.0)
            time.sleep(2.0)
            policy.squat()
    """

    def __init__(
        self,
        node: Node | None = None,
        *,
        policy_node: str = "/booster_policy",
        heartbeat_period: float | None = 0.02,
    ) -> None:
        self._owns_context = False
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        if node is None:
            if not rclpy.ok():
                rclpy.init()
                self._owns_context = True
            node = rclpy.create_node("booster_policy_client")
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(node)
            self._spin_thread = threading.Thread(
                target=self._executor.spin, name="policy_client", daemon=True
            )
        self.node = node

        self._lock = threading.Lock()
        self._command = PolicyCommand()
        self._status: PolicyStatus | None = None
        self._status_time = 0.0
        self._session: int | None = None
        self._start_future = None
        self._last_start_error: str | None = None
        self._closed = False

        policy_node = policy_node.rstrip("/")
        self._command_publisher = node.create_publisher(
            PolicyCommand,
            f"{policy_node}/command",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        self._status_subscription = node.create_subscription(
            PolicyStatus, f"{policy_node}/status", self._on_status, 10
        )
        self._start_client = node.create_client(StartPolicy, f"{policy_node}/start")
        self._stop_client = node.create_client(Trigger, f"{policy_node}/stop")
        self._heartbeat = None
        if heartbeat_period is not None:
            self._heartbeat = node.create_timer(heartbeat_period, self.publish_command)
        if self._spin_thread is not None:
            self._spin_thread.start()

    # Session control -----------------------------------------------------

    def request_start(self) -> bool:
        """Ask the node to reset and start a session without waiting.

        Returns True while a start is pending or a session is active; a new
        request is only sent when neither is the case. Call `stop()` first to
        restart an active session.
        """
        with self._lock:
            if self._session is not None or self._start_future is not None:
                return True
            if not self._start_client.service_is_ready():
                self._last_start_error = "policy node start service is not available"
                return False
            # Every session begins standing still and out of the squat.
            self._command.vx = self._command.vy = self._command.yaw_rate = 0.0
            self._command.squat = False
            future = self._start_client.call_async(StartPolicy.Request())
            self._start_future = future
        future.add_done_callback(self._on_start_response)
        return True

    def start(self, timeout: float = 5.0) -> None:
        """Start a session and block until the node publishes joint commands."""
        deadline = time.monotonic() + timeout
        while not self.is_ready():
            fault = self.fault
            if fault is not None:
                raise RuntimeError(f"Policy faulted during startup: {fault}")
            self.request_start()
            if time.monotonic() >= deadline:
                reason = self._last_start_error or "no ready status received"
                raise TimeoutError(f"Policy did not start within {timeout} s: {reason}")
            time.sleep(0.02)

    def stop(self, timeout: float = 1.0) -> bool:
        """Stop publishing joint commands; returns whether the node confirmed."""
        with self._lock:
            active = self._session is not None or self._start_future is not None
            self._session = None
            self._start_future = None
            self._command.vx = self._command.vy = self._command.yaw_rate = 0.0
            self._command.squat = False
        if not active:
            return True
        return self._call_stop(timeout)

    def _call_stop(self, timeout: float | None) -> bool:
        if not self._stop_client.service_is_ready():
            logger.warning("Policy node stop service is not available")
            return False
        future = self._stop_client.call_async(Trigger.Request())
        if timeout is None:
            return True
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            logger.warning("Policy node did not confirm stop within %.1f s", timeout)
            return False
        result = future.result()
        return result is not None and result.success

    def _on_start_response(self, future) -> None:
        try:
            response = future.result()
            error = None
        except Exception as exc:  # noqa: BLE001 - reported to the caller
            response = None
            error = str(exc)
        with self._lock:
            previous_error = self._last_start_error
            current = future is self._start_future
            if current:
                self._start_future = None
                if response is not None and response.success:
                    self._session = response.session
                    self._last_start_error = None
                else:
                    self._last_start_error = error or response.message
        if not current:
            # stop() ran while this start was in flight; do not leave it running.
            if response is not None and response.success:
                self._call_stop(timeout=None)
            return
        # Starts are retried until accepted; report each distinct reason once.
        if self._last_start_error not in (None, previous_error):
            logger.warning("Policy start rejected: %s", self._last_start_error)

    # Commands --------------------------------------------------------------

    def set_velocity(self, vx: float, vy: float, yaw_rate: float) -> None:
        """Latch the base velocity command (m/s, m/s, rad/s)."""
        with self._lock:
            self._command.vx = float(vx)
            self._command.vy = float(vy)
            self._command.yaw_rate = float(yaw_rate)

    def set_head_target(self, yaw: float, pitch: float) -> None:
        """Latch the head target (rad); positive pitch looks down."""
        with self._lock:
            self._command.head_yaw = float(yaw)
            self._command.head_pitch = float(pitch)

    def set_squat(self, enabled: bool) -> None:
        """Switch to the squat policy and crouch, or stand and resume walking."""
        with self._lock:
            self._command.squat = bool(enabled)
        self.publish_command()

    def squat(self) -> None:
        self.set_squat(True)

    def stand(self) -> None:
        self.set_squat(False)

    @property
    def squat_commanded(self) -> bool:
        with self._lock:
            return self._command.squat

    def publish_command(self) -> None:
        with self._lock:
            if self._closed:
                return
            command = PolicyCommand(
                vx=self._command.vx,
                vy=self._command.vy,
                yaw_rate=self._command.yaw_rate,
                head_yaw=self._command.head_yaw,
                head_pitch=self._command.head_pitch,
                squat=self._command.squat,
            )
        self._command_publisher.publish(command)

    # Status ------------------------------------------------------------------

    def _on_status(self, msg: PolicyStatus) -> None:
        with self._lock:
            self._status = msg
            self._status_time = time.monotonic()

    @property
    def status(self) -> PolicyStatus | None:
        """Latest fresh status of this client's session, if any."""
        with self._lock:
            status = self._status
            if (
                status is None
                or self._session is None
                or status.session != self._session
                or time.monotonic() - self._status_time > STATUS_TIMEOUT
            ):
                return None
            return status

    def is_running(self) -> bool:
        status = self.status
        return status is not None and status.state == PolicyStatus.STATE_RUNNING

    def is_ready(self) -> bool:
        """Whether the session is publishing joint commands to a subscriber."""
        status = self.status
        return (
            status is not None
            and status.state == PolicyStatus.STATE_RUNNING
            and status.ready
        )

    @property
    def fault(self) -> str | None:
        status = self.status
        if status is None or status.state != PolicyStatus.STATE_FAULT:
            return None
        return status.fault

    @property
    def squat_active(self) -> bool:
        status = self.status
        return status is not None and status.active_policy == PolicyStatus.POLICY_SQUAT

    @property
    def squat_started(self) -> bool:
        status = self.status
        return status is not None and status.squat_started

    @property
    def standing_pose_complete(self) -> bool:
        status = self.status
        return status is not None and status.standing_pose_complete

    # Lifecycle -------------------------------------------------------------

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
        if self._heartbeat is not None:
            self.node.destroy_timer(self._heartbeat)
        self.node.destroy_subscription(self._status_subscription)
        self.node.destroy_publisher(self._command_publisher)
        self.node.destroy_client(self._start_client)
        self.node.destroy_client(self._stop_client)
        if self._executor is not None:
            self._executor.shutdown()
            if self._spin_thread is not None:
                self._spin_thread.join(timeout=1.0)
            self.node.destroy_node()
            if self._owns_context:
                rclpy.try_shutdown()

    def __enter__(self) -> PolicyClient:
        return self

    def __exit__(self, *args) -> None:
        try:
            self.stop()
        finally:
            self.close()


__all__ = ["PolicyClient", "PolicyCommand", "PolicyStatus"]
