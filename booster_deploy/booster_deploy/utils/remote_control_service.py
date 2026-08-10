from __future__ import annotations

import atexit
import select
import sys
import termios
import threading
import tty


JOYSTICK_DEAD_ZONE = 0.1
MIN_TRANSLATIONAL_SPEED = 0.2
MAX_TRANSLATIONAL_SPEED = 0.75
MAX_YAW = 1.5
HEAD_ANGLE_STEP = 0.1
HEAD_YAW_LIMIT = 1.0
HEAD_PITCH_MIN = -0.349
HEAD_PITCH_MAX = 0.855


class RemoteControlService:
    """Track joystick gait commands and walk/squat button edges."""

    def __init__(
        self,
        *,
        controller_available: bool = False,
        workflow_controls: bool = False,
    ):
        self.controller_available = controller_available
        self.workflow_controls = workflow_controls
        self._lock = threading.Lock()
        self._running = True
        self._custom_mode_requested = False
        self._squat_enabled = False
        self._toggle_armed = False
        self._suppress_toggle_until_release = False
        self._controller_a_pressed = False
        self._controller_b_pressed = False
        self._crouch_requests = 0
        self._velocity_command = (0.0, 0.0, 0.0)
        self._head_target = (0.0, 0.0)
        self._dpad_pressed = (False, False, False, False)
        self._stdin_tty = False
        self._old_termios = None
        self.keyboard_runner = None

        self._start_keyboard_thread()
        atexit.register(self.close)

    def get_operation_hint(self) -> str:
        if self.workflow_controls:
            if self.controller_available:
                return (
                    "Press controller B or keyboard 's' to enable learned walking, "
                    "then use the left joystick. Press B again to squat."
                )
            return "Press keyboard 's' to enable learned walking, then to squat."
        if self.controller_available:
            return "Press controller B or keyboard 's' to toggle squat on/off."
        return "Press keyboard 's' to toggle squat on/off."

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
                    "  PREP/DAMP                     No deployment action",
                )
            elif real_robot:
                controls = (
                    "  Controller A / keyboard x  Enter custom mode and start policy",
                    "  Controller B / keyboard s  Toggle squat after policy startup",
                )
            else:
                controls = ("  Controller B / keyboard s  Toggle squat on/off",)
        elif real_robot:
            controls = (
                "  x  Enter custom mode and start policy",
                "  s  Toggle squat after policy startup",
            )
        else:
            controls = ("  s  Toggle squat on/off",)

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
            self._suppress_toggle_until_release = self._controller_b_pressed

    def get_squat_enabled(self) -> bool:
        with self._lock:
            return self._squat_enabled

    def get_velocity_command(self) -> tuple[float, float, float]:
        with self._lock:
            return self._velocity_command

    def get_head_target(self) -> tuple[float, float]:
        """Return the latched `(yaw, pitch)` target in radians."""
        with self._lock:
            return self._head_target

    def consume_crouch_request(self) -> bool:
        """Consume one workflow button edge, if one is pending."""
        with self._lock:
            if self._crouch_requests == 0:
                return False
            self._crouch_requests -= 1
            return True

    def discard_crouch_requests(self) -> None:
        with self._lock:
            self._crouch_requests = 0

    def set_squat_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._squat_enabled = enabled

    def _toggle_squat(self) -> None:
        with self._lock:
            if self.workflow_controls:
                self._crouch_requests += 1
                return
            if not self._toggle_armed:
                return
            self._squat_enabled = not self._squat_enabled
            state = "enabled" if self._squat_enabled else "disabled"
        print(f"Squat {state}")

    def _handle_keyboard_press(self, key: str) -> None:
        with self._lock:
            if key == "x":
                self._custom_mode_requested = True
        if key == "s":
            self._toggle_squat()

    def handle_controller_state(self, msg) -> None:
        """Handle a `/remote_controller_state` snapshot using rising edges."""
        squat_state = None
        with self._lock:
            a_pressed = bool(msg.a)
            b_pressed = bool(msg.b)
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
            dpad_pressed = (dpad_left, dpad_right, dpad_up, dpad_down)
            previous_dpad = self._dpad_pressed
            head_yaw, head_pitch = self._head_target
            if dpad_left and not previous_dpad[0]:
                head_yaw = min(head_yaw + HEAD_ANGLE_STEP, HEAD_YAW_LIMIT)
            if dpad_right and not previous_dpad[1]:
                head_yaw = max(head_yaw - HEAD_ANGLE_STEP, -HEAD_YAW_LIMIT)
            if dpad_up and not previous_dpad[2]:
                head_pitch = max(head_pitch - HEAD_ANGLE_STEP, HEAD_PITCH_MIN)
            if dpad_down and not previous_dpad[3]:
                head_pitch = min(head_pitch + HEAD_ANGLE_STEP, HEAD_PITCH_MAX)
            self._head_target = (head_yaw, head_pitch)
            self._dpad_pressed = dpad_pressed

            if a_pressed and not self._controller_a_pressed:
                self._custom_mode_requested = True

            if not b_pressed:
                self._suppress_toggle_until_release = False
            elif (
                not self._controller_b_pressed
                and not self._suppress_toggle_until_release
            ):
                if self.workflow_controls:
                    self._crouch_requests += 1
                elif self._toggle_armed:
                    self._squat_enabled = not self._squat_enabled
                    squat_state = (
                        "enabled" if self._squat_enabled else "disabled"
                    )

            self._controller_a_pressed = a_pressed
            self._controller_b_pressed = b_pressed

        if squat_state is not None:
            print(f"Squat {squat_state}")

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

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
