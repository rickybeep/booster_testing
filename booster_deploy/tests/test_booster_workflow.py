from __future__ import annotations

import unittest
from enum import Enum

from booster_deploy.controllers.booster_workflow import (
    create_walk_squat_workflow,
)


class Mode(Enum):
    DAMPING = 0
    PREPARE = 1
    WALKING = 2
    CUSTOM = 3


class FakeContext:
    def __init__(self) -> None:
        self.current_mode = Mode.PREPARE
        self.requests = 0
        self.ready = False
        self.policy_started = False
        self.squat_commanded = False
        self.calls: list[str] = []

    def discard_crouch_request(self) -> None:
        self.requests = 0
        self.calls.append("discard")

    def consume_crouch_request(self) -> bool:
        if not self.requests:
            return False
        self.requests -= 1
        return True

    def begin_policy(self) -> bool:
        if not self.policy_started:
            self.calls.append("begin")
            self.policy_started = True
        return True

    def policy_is_ready(self) -> bool:
        return self.ready

    def enter_custom_mode(self) -> None:
        self.calls.append("custom")

    def squat_is_commanded(self) -> bool:
        return self.squat_commanded

    def request_crouch(self) -> None:
        self.squat_commanded = True
        self.calls.append("crouch")

    def request_stand(self) -> None:
        self.squat_commanded = False
        self.calls.append("stand")

    def cancel_policy(self) -> None:
        self.policy_started = False
        self.calls.append("cancel")


class WalkSquatWorkflowTest(unittest.TestCase):
    def setUp(self) -> None:
        self.context = FakeContext()
        self.tree = create_walk_squat_workflow(
            self.context,
            walking_mode=Mode.WALKING,
            custom_mode=Mode.CUSTOM,
        )

    def tick(self, count: int = 1) -> None:
        for _ in range(count):
            self.tree.tick()

    def test_prep_and_damping_are_inert_and_discard_requests(self) -> None:
        for mode in (Mode.PREPARE, Mode.DAMPING):
            self.context.current_mode = mode
            self.context.requests = 1
            self.tick()
            self.assertEqual(self.context.requests, 0)
        self.assertNotIn("begin", self.context.calls)
        self.assertNotIn("custom", self.context.calls)

    def test_walk_requires_b_before_starting_policy_and_requesting_custom(self) -> None:
        self.context.current_mode = Mode.WALKING
        self.tick()
        self.assertNotIn("begin", self.context.calls)
        self.assertNotIn("custom", self.context.calls)

        self.context.requests = 1
        self.tick()
        self.assertEqual(self.context.calls[-1], "begin")
        self.assertNotIn("custom", self.context.calls)

        self.context.ready = True
        self.tick()
        self.assertEqual(self.context.calls[-1], "custom")

        self.context.current_mode = Mode.CUSTOM
        self.tick()
        self.assertEqual(self.context.calls[-1], "discard")

    def test_b_toggles_squat_only_after_custom_is_confirmed(self) -> None:
        self.context.current_mode = Mode.WALKING
        self.context.ready = True
        self.context.requests = 1
        self.tick()
        self.assertEqual(self.context.requests, 0)
        self.assertEqual(self.context.calls[-1], "custom")

        self.context.current_mode = Mode.CUSTOM
        self.tick()
        self.assertEqual(self.context.requests, 0)
        self.assertNotIn("crouch", self.context.calls)

        self.context.requests = 1
        self.tick()
        self.assertEqual(self.context.calls[-1], "crouch")
        self.context.requests = 1
        self.tick()
        self.assertEqual(self.context.calls[-1], "stand")

    def test_external_inactive_mode_stops_policy(self) -> None:
        self.context.current_mode = Mode.WALKING
        self.context.ready = True
        self.context.requests = 1
        self.tick()
        self.context.current_mode = Mode.CUSTOM
        self.tick()
        self.context.current_mode = Mode.DAMPING
        self.tick()
        self.assertEqual(self.context.calls[-1], "cancel")


if __name__ == "__main__":
    unittest.main()
