from __future__ import annotations

import unittest
from enum import Enum

from booster_deploy.controllers.booster_workflow import create_squat_workflow


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
        self.started = False
        self.reference_complete = False
        self.measured_standing = False
        self.calls: list[str] = []
        self.policy_started = False
        self.progress: list[tuple[int, int]] = []

    def discard_crouch_request(self) -> None:
        self.requests = 0
        self.calls.append("discard")

    def consume_crouch_request(self) -> bool:
        if not self.requests:
            return False
        self.requests -= 1
        return True

    def begin_squat(self) -> bool:
        if not self.policy_started:
            self.calls.append("begin")
            self.policy_started = True
        return True

    def policy_is_ready(self) -> bool:
        return self.ready

    def enter_custom_mode(self) -> None:
        self.calls.append("custom")
        self.current_mode = Mode.CUSTOM

    def request_stand(self) -> None:
        self.calls.append("stand")

    def squat_has_started(self) -> bool:
        return self.started

    def standing_reference_complete(self) -> bool:
        return self.reference_complete

    def robot_is_standing(self) -> bool:
        return self.measured_standing

    def log_standing_progress(
        self, stable_ticks: int, required_stable_ticks: int
    ) -> None:
        self.progress.append((stable_ticks, required_stable_ticks))

    def finish_squat(self) -> None:
        self.calls.append("walk")
        self.current_mode = Mode.WALKING

    def cancel_squat(self) -> None:
        self.calls.append("cancel")


class SquatWorkflowTest(unittest.TestCase):
    def setUp(self) -> None:
        self.context = FakeContext()
        self.tree = create_squat_workflow(
            self.context,
            walking_mode=Mode.WALKING,
            custom_mode=Mode.CUSTOM,
            standing_stable_ticks=3,
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
        self.assertNotIn("walk", self.context.calls)

    def test_full_cycle_waits_for_reference_and_measured_standing(self) -> None:
        self.context.current_mode = Mode.WALKING
        self.context.requests = 1
        self.tick()
        self.assertEqual(self.context.calls, ["begin"])

        self.context.ready = True
        self.tick()
        self.assertEqual(self.context.calls[-1], "custom")

        self.context.started = True
        self.context.requests = 1
        self.tick()
        self.assertEqual(self.context.calls[-1], "stand")
        self.assertEqual(self.context.progress[-1], (0, 3))

        self.context.measured_standing = True
        self.tick(5)
        self.assertNotIn("walk", self.context.calls)

        self.context.reference_complete = True
        self.tick(2)
        self.assertEqual(self.context.progress[-1], (2, 3))
        self.assertNotIn("walk", self.context.calls)
        self.tick()
        self.assertEqual(self.context.calls[-1], "walk")

    def test_external_inactive_mode_cancels_active_cycle(self) -> None:
        self.context.current_mode = Mode.WALKING
        self.context.requests = 1
        self.tick()
        self.context.current_mode = Mode.DAMPING
        self.tick()
        self.assertEqual(self.context.calls[-1], "cancel")


if __name__ == "__main__":
    unittest.main()
