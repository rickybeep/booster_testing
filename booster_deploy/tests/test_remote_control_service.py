from __future__ import annotations

import unittest
from types import SimpleNamespace

from booster_deploy.utils.remote_control_service import RemoteControlService


class RemoteControlServiceTest(unittest.TestCase):
    def test_workflow_controls_queue_rising_edges_without_toggling(self) -> None:
        service = RemoteControlService(workflow_controls=True)
        self.addCleanup(service.close)

        service._handle_keyboard_press("s")
        self.assertEqual(service.consume_crouch_request(), "squat")
        self.assertIsNone(service.consume_crouch_request())
        self.assertFalse(service.get_squat_enabled())

        service.handle_controller_state(SimpleNamespace(a=False, b=True, x=False))
        service.handle_controller_state(SimpleNamespace(a=False, b=True, x=False))
        self.assertEqual(service.consume_crouch_request(), "squat")
        self.assertIsNone(service.consume_crouch_request())

    def test_kneel_button_and_key_request_that_policy(self) -> None:
        service = RemoteControlService(workflow_controls=True)
        self.addCleanup(service.close)

        service._handle_keyboard_press("m")
        self.assertEqual(service.consume_crouch_request(), "kneel")

        service.handle_controller_state(SimpleNamespace(a=False, b=False, x=True))
        service.handle_controller_state(SimpleNamespace(a=False, b=False, x=True))
        service.handle_controller_state(SimpleNamespace(a=False, b=False, x=False))
        self.assertEqual(service.consume_crouch_request(), "kneel")
        self.assertIsNone(service.consume_crouch_request())

        # Simultaneous edges are queued in a stable order.
        service.handle_controller_state(SimpleNamespace(a=False, b=True, x=True))
        self.assertEqual(service.consume_crouch_request(), "squat")
        self.assertEqual(service.consume_crouch_request(), "kneel")
        service.discard_crouch_requests()
        self.assertIsNone(service.consume_crouch_request())

    def test_mujoco_toggle_behavior_is_unchanged(self) -> None:
        service = RemoteControlService()
        self.addCleanup(service.close)

        service._handle_keyboard_press("s")
        self.assertFalse(service.get_squat_enabled())
        service.arm_squat_toggle()
        service._handle_keyboard_press("s")
        self.assertTrue(service.get_squat_enabled())


if __name__ == "__main__":
    unittest.main()
