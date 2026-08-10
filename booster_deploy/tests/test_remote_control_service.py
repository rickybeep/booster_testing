from __future__ import annotations

import math
import unittest
from types import SimpleNamespace

from booster_deploy.utils.remote_control_service import RemoteControlService


class RemoteControlServiceTest(unittest.TestCase):
    def test_workflow_controls_queue_rising_edges_without_toggling(self) -> None:
        service = RemoteControlService(workflow_controls=True)
        self.addCleanup(service.close)

        service._handle_keyboard_press("s")
        self.assertTrue(service.consume_crouch_request())
        self.assertFalse(service.consume_crouch_request())
        self.assertFalse(service.get_squat_enabled())

        service.handle_controller_state(SimpleNamespace(a=False, b=True))
        service.handle_controller_state(SimpleNamespace(a=False, b=True))
        self.assertTrue(service.consume_crouch_request())
        self.assertFalse(service.consume_crouch_request())

    def test_mujoco_toggle_behavior_is_unchanged(self) -> None:
        service = RemoteControlService()
        self.addCleanup(service.close)

        service._handle_keyboard_press("s")
        self.assertFalse(service.get_squat_enabled())
        service.arm_squat_toggle()
        service._handle_keyboard_press("s")
        self.assertTrue(service.get_squat_enabled())

    def test_joystick_axes_match_walk_policy_commands(self) -> None:
        service = RemoteControlService(controller_available=True)
        self.addCleanup(service.close)

        service.handle_controller_state(
            SimpleNamespace(a=False, b=False, ly=-1.0, lx=0.5, rx=-0.25)
        )
        vx, vy, yaw = service.get_velocity_command()
        self.assertAlmostEqual(math.hypot(vx, vy), 0.75)
        self.assertAlmostEqual(vx, 0.670820393)
        self.assertAlmostEqual(vy, -0.335410197)
        self.assertAlmostEqual(yaw, 0.375)

        service.handle_controller_state(
            SimpleNamespace(a=False, b=False, ly=1.0, lx=0.0, rx=0.0)
        )
        self.assertEqual(service.get_velocity_command(), (-0.75, -0.0, -0.0))


if __name__ == "__main__":
    unittest.main()
