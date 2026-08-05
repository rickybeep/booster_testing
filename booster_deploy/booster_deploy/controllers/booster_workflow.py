from __future__ import annotations

from typing import Any, Protocol

import py_trees


class SquatWorkflowContext(Protocol):
    current_mode: Any

    def discard_crouch_request(self) -> None: ...

    def consume_crouch_request(self) -> bool: ...

    def begin_squat(self) -> bool: ...

    def policy_is_ready(self) -> bool: ...

    def enter_custom_mode(self) -> None: ...

    def request_stand(self) -> None: ...

    def squat_has_started(self) -> bool: ...

    def standing_reference_complete(self) -> bool: ...

    def robot_is_standing(self) -> bool: ...

    def log_standing_progress(
        self, stable_ticks: int, required_stable_ticks: int
    ) -> None: ...

    def finish_squat(self) -> None: ...

    def cancel_squat(self) -> None: ...


class _InactiveMode(py_trees.behaviour.Behaviour):
    def __init__(self, context: SquatWorkflowContext, walking: Any, custom: Any):
        super().__init__(name="Inactive in prep/damp/other mode")
        self.context = context
        self.walking = walking
        self.custom = custom

    def update(self) -> py_trees.common.Status:
        if self.context.current_mode in (self.walking, self.custom):
            return py_trees.common.Status.FAILURE
        self.context.discard_crouch_request()
        self.context.cancel_squat()
        return py_trees.common.Status.RUNNING


class _WaitForCrouch(py_trees.behaviour.Behaviour):
    def __init__(self, context: SquatWorkflowContext, walking: Any):
        super().__init__(name="Wait for crouch in walking mode")
        self.context = context
        self.walking = walking

    def update(self) -> py_trees.common.Status:
        if self.context.current_mode != self.walking:
            self.context.discard_crouch_request()
            return py_trees.common.Status.RUNNING
        if self.context.consume_crouch_request():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class _StartPolicy(py_trees.behaviour.Behaviour):
    def __init__(self, context: SquatWorkflowContext):
        super().__init__(name="Start squat policy")
        self.context = context

    def update(self) -> py_trees.common.Status:
        self.context.begin_squat()
        if self.context.policy_is_ready():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class _EnterCustom(py_trees.behaviour.Behaviour):
    def __init__(self, context: SquatWorkflowContext):
        super().__init__(name="Enter custom mode")
        self.context = context

    def update(self) -> py_trees.common.Status:
        self.context.enter_custom_mode()
        return py_trees.common.Status.SUCCESS


class _RunSquatUntilStanding(py_trees.behaviour.Behaviour):
    def __init__(self, context: SquatWorkflowContext, custom: Any, stable_ticks: int):
        super().__init__(name="Squat and wait for measured standing")
        self.context = context
        self.custom = custom
        self.required_stable_ticks = stable_ticks
        self.stable_ticks = 0
        self.stand_requested = False

    def initialise(self) -> None:
        self.stable_ticks = 0
        self.stand_requested = False

    def update(self) -> py_trees.common.Status:
        if self.context.current_mode != self.custom:
            self.stable_ticks = 0
            if self.stand_requested:
                self.context.log_standing_progress(
                    self.stable_ticks, self.required_stable_ticks
                )
            return py_trees.common.Status.RUNNING

        if self.context.consume_crouch_request():
            self.context.request_stand()
            self.stand_requested = True

        started = self.context.squat_has_started()
        reference_complete = self.context.standing_reference_complete()
        robot_standing = self.context.robot_is_standing()
        complete = (
            self.stand_requested
            and started
            and reference_complete
            and robot_standing
        )
        self.stable_ticks = self.stable_ticks + 1 if complete else 0
        if self.stand_requested:
            self.context.log_standing_progress(
                self.stable_ticks, self.required_stable_ticks
            )
        if self.stable_ticks >= self.required_stable_ticks:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class _ReturnToWalking(py_trees.behaviour.Behaviour):
    def __init__(self, context: SquatWorkflowContext):
        super().__init__(name="Return to walking mode")
        self.context = context

    def update(self) -> py_trees.common.Status:
        self.context.finish_squat()
        return py_trees.common.Status.SUCCESS


def create_squat_workflow(
    context: SquatWorkflowContext,
    *,
    walking_mode: Any,
    custom_mode: Any,
    standing_stable_ticks: int,
) -> py_trees.trees.BehaviourTree:
    """Create the real-robot mode/squat workflow.

    Modes other than walking and custom are deliberately an inert, higher
    priority branch. The memory sequence only advances after a crouch request
    observed while walking.
    """
    if standing_stable_ticks < 1:
        raise ValueError("standing_stable_ticks must be at least one")

    squat = py_trees.composites.Sequence(
        name="Walking squat cycle",
        memory=True,
        children=[
            _WaitForCrouch(context, walking_mode),
            _StartPolicy(context),
            _EnterCustom(context),
            _RunSquatUntilStanding(
                context, custom_mode, standing_stable_ticks
            ),
            _ReturnToWalking(context),
        ],
    )
    root = py_trees.composites.Selector(
        name="Booster deploy workflow",
        memory=False,
        children=[
            _InactiveMode(context, walking_mode, custom_mode),
            squat,
        ],
    )
    return py_trees.trees.BehaviourTree(root)
