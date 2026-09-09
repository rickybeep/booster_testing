from __future__ import annotations

import atexit
import select
import sys
import termios
import threading
import tty
from collections import deque

# Button -> policy name. Each button requests a crouch cycle with its own
# policy while walking; any button requests standing once a cycle is active.
SQUAT_POLICY = "squat"
KNEEL_POLICY = "kneel"
KEYBOARD_POLICY_KEYS = {"s": SQUAT_POLICY, "m": KNEEL_POLICY}
CONTROLLER_POLICY_BUTTONS = {"b": SQUAT_POLICY, "x": KNEEL_POLICY}


class RemoteControlService:
    """Track the ROS controller and keyboard controls used by the squat task."""

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
        self._controller_pressed = {button: False for button in CONTROLLER_POLICY_BUTTONS}
        self._crouch_requests: deque[str] = deque()
        self._stdin_tty = False
        self._old_termios = None
        self.keyboard_runner = None

        self._start_keyboard_thread()
        atexit.register(self.close)

    def get_operation_hint(self) -> str:
        if self.workflow_controls:
            if self.controller_available:
                return (
                    "Press controller B or keyboard 's' to crouch with the squat "
                    "policy, controller X or keyboard 'm' to crouch with the "
                    "kneel policy; press either to stand. "
                    "Crouch is accepted only while walking."
                )
            return (
                "Press keyboard 's' (squat) or 'm' (kneel) to "
                "crouch/stand while walking."
            )
        if self.controller_available:
            return "Press controller B/X or keyboard 's'/'m' to toggle squat on/off."
        return "Press keyboard 's' or 'm' to toggle squat on/off."

    def get_custom_mode_operation_hint(self) -> str:
        if self.controller_available:
            return "Press controller A or keyboard 'x' to enter custom mode."
        return "Press keyboard 'x' to enter custom mode."

    def print_controls(self, *, real_robot: bool) -> None:
        """Print the controls available for the selected inputs."""
        if self.controller_available:
            if real_robot and self.workflow_controls:
                controls = (
                    "  Controller B / keyboard s  Crouch (squat policy), then stand",
                    "  Controller X / keyboard m  Crouch (kneel), then stand",
                    "  PREP/DAMP                   No deployment action",
                )
            elif real_robot:
                controls = (
                    "  Controller A / keyboard x  Enter custom mode and start policy",
                    "  Controller B / keyboard s  Toggle squat after policy startup",
                )
            else:
                controls = ("  Controller B/X / keyboard s/m  Toggle squat on/off",)
        elif real_robot:
            controls = (
                "  x  Enter custom mode and start policy",
                "  s  Toggle squat after policy startup",
            )
        else:
            controls = ("  s / m  Toggle squat on/off",)

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

    def consume_crouch_request(self) -> str | None:
        """Consume one workflow button edge, if one is pending.

        Returns the policy name the pressed button maps to, or None.
        """
        with self._lock:
            if not self._crouch_requests:
                return None
            return self._crouch_requests.popleft()

    def discard_crouch_requests(self) -> None:
        with self._lock:
            self._crouch_requests.clear()

    def set_squat_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._squat_enabled = enabled

    def _toggle_squat(self, policy: str = SQUAT_POLICY) -> None:
        with self._lock:
            if self.workflow_controls:
                self._crouch_requests.append(policy)
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
        policy = KEYBOARD_POLICY_KEYS.get(key)
        if policy is not None:
            self._toggle_squat(policy)

    def handle_controller_state(self, msg) -> None:
        """Handle a `/remote_controller_state` snapshot using rising edges."""
        squat_state = None
        with self._lock:
            a_pressed = bool(msg.a)
            if a_pressed and not self._controller_a_pressed:
                self._custom_mode_requested = True
            self._controller_a_pressed = a_pressed

            for button, policy in CONTROLLER_POLICY_BUTTONS.items():
                pressed = bool(getattr(msg, button, False))
                was_pressed = self._controller_pressed[button]
                suppressed = button == "b" and self._suppress_toggle_until_release
                if button == "b" and not pressed:
                    self._suppress_toggle_until_release = False
                if pressed and not was_pressed and not suppressed:
                    if self.workflow_controls:
                        self._crouch_requests.append(policy)
                    elif self._toggle_armed:
                        self._squat_enabled = not self._squat_enabled
                        squat_state = (
                            "enabled" if self._squat_enabled else "disabled"
                        )
                self._controller_pressed[button] = pressed
            self._controller_b_pressed = self._controller_pressed["b"]

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
