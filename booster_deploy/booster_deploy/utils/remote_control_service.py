from __future__ import annotations

import atexit
import select
import sys
import termios
import threading
import tty
from collections import deque


# Button -> pose policy name. While learned walking runs, each button switches
# to its own pose policy; any button then stands and resumes walking.
SQUAT_POLICY = "squat"
SIT_POLICY = "sit"
POSE_POLICIES = (SQUAT_POLICY, SIT_POLICY)
KEYBOARD_POLICY_KEYS = {"s": SQUAT_POLICY, "m": SIT_POLICY}
CONTROLLER_POLICY_BUTTONS = {"b": SQUAT_POLICY, "x": SIT_POLICY}

JOYSTICK_DEAD_ZONE = 0.1
MIN_TRANSLATIONAL_SPEED = 0.2
MAX_TRANSLATIONAL_SPEED = 0.75
MAX_YAW = 1.5
HEAD_ANGULAR_SPEED = 0.8
HEAD_UPDATE_PERIOD = 0.02
HEAD_YAW_LIMIT = 1.0
HEAD_PITCH_MIN = -0.349
HEAD_PITCH_MAX = 0.855


class RemoteControlService:
    """Track joystick gait commands and walk/squat/sit button edges."""

    def __init__(
        self,
        *,
        controller_available: bool = False,
        workflow_controls: bool = False,
        start_head_thread: bool = True,
    ):
        self.controller_available = controller_available
        self.workflow_controls = workflow_controls
        self._lock = threading.Lock()
        self._running = True
        self._custom_mode_requested = False
        self._squat_enabled = False
        self._pose_policy = SQUAT_POLICY
        self._toggle_armed = False
        self._suppress_toggle_until_release = False
        self._controller_a_pressed = False
        self._controller_pressed = {
            button: False for button in CONTROLLER_POLICY_BUTTONS
        }
        self._crouch_requests: deque[str] = deque()
        self._velocity_command = (0.0, 0.0, 0.0)
        self._head_target = (0.0, 0.0)
        self._dpad_direction = (0.0, 0.0)
        self._stdin_tty = False
        self._old_termios = None
        self.keyboard_runner = None
        self.head_runner = None
        self._head_stop_event = threading.Event()

        self._start_keyboard_thread()
        if start_head_thread:
            self._start_head_thread()
        atexit.register(self.close)

    def get_operation_hint(self) -> str:
        if self.workflow_controls:
            if self.controller_available:
                return (
                    "Press controller B or keyboard 's' to enable learned walking, "
                    "then use the left joystick. Press B again to squat or X to "
                    "sit; press either to stand."
                )
            return (
                "Press keyboard 's' to enable learned walking, then 's' to "
                "squat or 'm' to sit."
            )
        if self.controller_available:
            return (
                "Press controller B / keyboard 's' to toggle squat, "
                "X / 'm' to toggle sit."
            )
        return "Press keyboard 's' to toggle squat, 'm' to toggle sit."

    def get_custom_mode_operation_hint(self) -> str:
        if self.controller_available:
            return "Press controller A or keyboard 'x' to enter custom mode."
        return "Press keyboard 'x' to enter custom mode."

    def print_controls(self, *, real_robot: bool) -> None:
        """Print the controls available for the selected inputs."""
        if self.controller_available:
            if real_robot and self.workflow_controls:
                controls = (
                    "  First controller B / s       Enable learned walk and enter CUSTOM",
                    "  Left stick                   Walk (0.2-0.75 m/s outside dead zone)",
                    "  Right stick horizontal       Turn",
                    "  D-pad left/right, up/down    Head yaw, pitch",
                    "  Later controller B / s       Squat, then stand and resume walk",
                    "  Later controller X / m       Sit, then stand and resume walk",
                    "  PREP/DAMP                     No deployment action",
                )
            elif real_robot:
                controls = (
                    "  Controller A / keyboard x  Enter custom mode and start policy",
                    "  Controller B / keyboard s  Toggle squat after policy startup",
                )
            else:
                controls = (
                    "  Controller B / keyboard s  Toggle squat on/off",
                    "  Controller X / keyboard m  Toggle sit on/off",
                )
        elif real_robot:
            controls = (
                "  x  Enter custom mode and start policy",
                "  s  Toggle squat after policy startup",
            )
        else:
            controls = (
                "  s  Toggle squat on/off",
                "  m  Toggle sit on/off",
            )

        print("\nControls:")
        print("\n".join(controls))
        print("  Ctrl-C  Stop\n")

    def start_custom_mode(self) -> bool:
        with self._lock:
            return self._custom_mode_requested

    def arm_squat_toggle(self) -> None:
        """Enable toggle handling after the policy has started."""
        with self._lock:
            self._toggle_armed = True
            self._suppress_toggle_until_release = any(
                self._controller_pressed.values()
            )

    def get_squat_enabled(self) -> bool:
        with self._lock:
            return self._squat_enabled

    def get_pose_policy(self) -> str:
        """Return the pose policy the squat command applies to."""
        with self._lock:
            return self._pose_policy

    def get_velocity_command(self) -> tuple[float, float, float]:
        with self._lock:
            return self._velocity_command

    def get_head_target(self) -> tuple[float, float]:
        """Return the latched `(yaw, pitch)` target in radians."""
        with self._lock:
            return self._head_target

    def consume_crouch_request(self) -> str | None:
        """Consume one workflow button edge, if one is pending.

        Returns the pose policy name the pressed button maps to, or None.
        """
        with self._lock:
            if not self._crouch_requests:
                return None
            return self._crouch_requests.popleft()

    def discard_crouch_requests(self) -> None:
        with self._lock:
            self._crouch_requests.clear()

    def set_squat_enabled(
        self, enabled: bool, pose_policy: str | None = None
    ) -> None:
        with self._lock:
            self._squat_enabled = enabled
            if pose_policy is not None:
                self._pose_policy = pose_policy

    def _toggle_locked(self, pose_policy: str) -> str | None:
        """Apply one button edge; returns a status message to print."""
        if self.workflow_controls:
            self._crouch_requests.append(pose_policy)
            return None
        if not self._toggle_armed:
            return None
        if self._squat_enabled:
            # Any button ends the active pose; the selection is latched.
            self._squat_enabled = False
            return f"{self._pose_policy.capitalize()} disabled"
        self._squat_enabled = True
        self._pose_policy = pose_policy
        return f"{pose_policy.capitalize()} enabled"

    def _toggle_squat(self, pose_policy: str = SQUAT_POLICY) -> None:
        with self._lock:
            message = self._toggle_locked(pose_policy)
        if message is not None:
            print(message)

    def _handle_keyboard_press(self, key: str) -> None:
        with self._lock:
            if key == "x":
                self._custom_mode_requested = True
        pose_policy = KEYBOARD_POLICY_KEYS.get(key)
        if pose_policy is not None:
            self._toggle_squat(pose_policy)

    def handle_controller_state(self, msg) -> None:
        """Handle a `/remote_controller_state` snapshot using rising edges."""
        messages: list[str] = []
        with self._lock:
            a_pressed = bool(msg.a)
            direction_x = -float(getattr(msg, "ly", 0.0))
            direction_y = -float(getattr(msg, "lx", 0.0))
            stick_magnitude = (direction_x**2 + direction_y**2) ** 0.5
            if stick_magnitude <= JOYSTICK_DEAD_ZONE:
                vx = 0.0
                vy = 0.0
            else:
                speed = min(
                    max(
                        stick_magnitude * MAX_TRANSLATIONAL_SPEED,
                        MIN_TRANSLATIONAL_SPEED,
                    ),
                    MAX_TRANSLATIONAL_SPEED,
                )
                vx = direction_x / stick_magnitude * speed
                vy = direction_y / stick_magnitude * speed
            yaw = -float(getattr(msg, "rx", 0.0)) * MAX_YAW
            self._velocity_command = (vx, vy, yaw)

            dpad_left = any(
                bool(getattr(msg, name, False))
                for name in ("hat_l", "hat_lu", "hat_ld")
            )
            dpad_right = any(
                bool(getattr(msg, name, False))
                for name in ("hat_r", "hat_ru", "hat_rd")
            )
            dpad_up = any(
                bool(getattr(msg, name, False))
                for name in ("hat_u", "hat_lu", "hat_ru")
            )
            dpad_down = any(
                bool(getattr(msg, name, False))
                for name in ("hat_d", "hat_ld", "hat_rd")
            )
            self._dpad_direction = (
                float(dpad_left) - float(dpad_right),
                float(dpad_down) - float(dpad_up),
            )

            if a_pressed and not self._controller_a_pressed:
                self._custom_mode_requested = True

            self._controller_a_pressed = a_pressed

            pressed = {
                button: bool(getattr(msg, button, False))
                for button in CONTROLLER_POLICY_BUTTONS
            }
            if not any(pressed.values()):
                self._suppress_toggle_until_release = False
            if not self._suppress_toggle_until_release:
                for button, pose_policy in CONTROLLER_POLICY_BUTTONS.items():
                    if pressed[button] and not self._controller_pressed[button]:
                        message = self._toggle_locked(pose_policy)
                        if message is not None:
                            messages.append(message)
            self._controller_pressed = pressed

        for message in messages:
            print(message)

    def _advance_head_target(self, dt: float) -> None:
        """Integrate the latched D-pad direction for one time interval."""
        with self._lock:
            yaw_direction, pitch_direction = self._dpad_direction
            head_yaw, head_pitch = self._head_target
            head_yaw += yaw_direction * HEAD_ANGULAR_SPEED * dt
            head_pitch += pitch_direction * HEAD_ANGULAR_SPEED * dt
            head_yaw = min(max(head_yaw, -HEAD_YAW_LIMIT), HEAD_YAW_LIMIT)
            head_pitch = min(max(head_pitch, HEAD_PITCH_MIN), HEAD_PITCH_MAX)
            self._head_target = (head_yaw, head_pitch)

    def _start_head_thread(self) -> None:
        self.head_runner = threading.Thread(
            target=self._head_control_loop,
            daemon=True,
            name="dpad-head-control",
        )
        self.head_runner.start()

    def _head_control_loop(self) -> None:
        while not self._head_stop_event.wait(HEAD_UPDATE_PERIOD):
            self._advance_head_target(HEAD_UPDATE_PERIOD)

    def _start_keyboard_thread(self) -> None:
        try:
            self._stdin_tty = sys.stdin.isatty()
            if self._stdin_tty:
                self._old_termios = termios.tcgetattr(sys.stdin.fileno())
        except Exception:
            self._stdin_tty = False
        self.keyboard_runner = threading.Thread(
            target=self._keyboard_listener, daemon=True, name="squat-keyboard"
        )
        self.keyboard_runner.start()

    def _keyboard_listener(self) -> None:
        if not self._stdin_tty:
            return
        fd = sys.stdin.fileno()
        try:
            tty.setcbreak(fd)
            while self._running:
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if readable:
                    key = sys.stdin.read(1).lower()
                    if key != "\x03":
                        self._handle_keyboard_press(key)
        finally:
            if self._old_termios is not None:
                termios.tcsetattr(fd, termios.TCSADRAIN, self._old_termios)

    def close(self) -> None:
        if not self._running:
            return
        self._running = False
        self._head_stop_event.set()
        if self._stdin_tty and self._old_termios is not None:
            try:
                termios.tcsetattr(
                    sys.stdin.fileno(), termios.TCSADRAIN, self._old_termios
                )
            except Exception:
                pass
        if (
            self.keyboard_runner is not None
            and self.keyboard_runner is not threading.current_thread()
        ):
            self.keyboard_runner.join(timeout=1.0)
        if (
            self.head_runner is not None
            and self.head_runner is not threading.current_thread()
        ):
            self.head_runner.join(timeout=1.0)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
