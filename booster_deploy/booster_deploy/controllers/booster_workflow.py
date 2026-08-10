from __future__ import annotations

from typing import Any, Protocol

import py_trees


class WalkSquatWorkflowContext(Protocol):
    current_mode: Any

    def discard_crouch_request(self) -> None: ...

    def consume_crouch_request(self) -> bool: ...

    def begin_policy(self) -> bool: ...

    def policy_is_ready(self) -> bool: ...

    def enter_custom_mode(self) -> None: ...

    def squat_is_commanded(self) -> bool: ...

    def request_crouch(self) -> None: ...

    def request_stand(self) -> None: ...

    def cancel_policy(self) -> None: ...


class SquatWorkflowContext(Protocol):
    current_mode: Any

    def discard_crouch_request(self) -> None: ...

    def consume_crouch_request(self) -> bool: ...

    def begin_squat(self) -> bool: ...

    def policy_is_ready(self) -> bool: ...

    def enter_custom_mode(self) -> None: ...

    def request_crouch(self) -> None: ...

    def request_stand(self) -> None: ...

    def squat_has_started(self) -> bool: ...

    def standing_pose_complete(self) -> bool: ...

    def robot_is_standing(self) -> bool: ...

    def finish_squat(self) -> None: ...

    def cancel_squat(self) -> None: ...


class _InactiveMode(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        context: WalkSquatWorkflowContext,
        walking: Any,
        custom: Any,
    ):
        super().__init__(name="Inactive in prep/damp/other mode")
        self.context = context
        self.walking = walking
        self.custom = custom

    def update(self) -> py_trees.common.Status:
        if self.context.current_mode in (self.walking, self.custom):
            return py_trees.common.Status.FAILURE
        self.context.discard_crouch_request()
        self.context.cancel_policy()
        return py_trees.common.Status.RUNNING


class _WaitForWalkActivation(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        context: WalkSquatWorkflowContext,
        walking: Any,
    ):
        super().__init__(name="Wait for B in walking mode")
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
    def __init__(self, context: WalkSquatWorkflowContext):
        super().__init__(name="Start learned walk policy")
        self.context = context

    def update(self) -> py_trees.common.Status:
        self.context.begin_policy()
        if self.context.policy_is_ready():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class _EnterCustom(py_trees.behaviour.Behaviour):
    def __init__(self, context: WalkSquatWorkflowContext, custom: Any):
        super().__init__(name="Enter custom mode")
        self.context = context
        self.custom = custom
        self.requested = False

    def initialise(self) -> None:
        self.requested = False

    def update(self) -> py_trees.common.Status:
        if self.context.current_mode == self.custom:
            return py_trees.common.Status.SUCCESS
        if not self.requested:
            self.context.enter_custom_mode()
            self.requested = True
        return py_trees.common.Status.RUNNING


class _RunPolicies(py_trees.behaviour.Behaviour):
    def __init__(self, context: WalkSquatWorkflowContext, custom: Any):
        super().__init__(name="Run learned walk and squat policies")
        self.context = context
        self.custom = custom

    def initialise(self) -> None:
        # A B edge from the firmware-to-CUSTOM transition must not immediately
        # select squat; switching policies requires a fresh press in CUSTOM.
        self.context.discard_crouch_request()

    def update(self) -> py_trees.common.Status:
        if self.context.current_mode != self.custom:
            return py_trees.common.Status.FAILURE
        if self.context.consume_crouch_request():
            if self.context.squat_is_commanded():
                self.context.request_stand()
            else:
                self.context.request_crouch()
        return py_trees.common.Status.RUNNING


def create_walk_squat_workflow(
    context: WalkSquatWorkflowContext,
    *,
    walking_mode: Any,
    custom_mode: Any,
) -> py_trees.trees.BehaviourTree:
    """Enable learned walking on B, then keep learned policies in CUSTOM."""
    policies = py_trees.composites.Sequence(
        name="Learned walk and squat lifecycle",
        memory=True,
        children=[
            _WaitForWalkActivation(context, walking_mode),
            _StartPolicy(context),
            _EnterCustom(context, custom_mode),
            _RunPolicies(context, custom_mode),
        ],
    )
    root = py_trees.composites.Selector(
        name="Booster deploy workflow",
        memory=False,
        children=[
            _InactiveMode(context, walking_mode, custom_mode),
            policies,
        ],
    )
    return py_trees.trees.BehaviourTree(root)


class _SquatInactiveMode(py_trees.behaviour.Behaviour):
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


class _StartSquatPolicy(py_trees.behaviour.Behaviour):
    def __init__(self, context: SquatWorkflowContext):
        super().__init__(name="Start squat policy")
        self.context = context

    def update(self) -> py_trees.common.Status:
        self.context.begin_squat()
        if self.context.policy_is_ready():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class _EnterSquatCustom(py_trees.behaviour.Behaviour):
    def __init__(self, context: SquatWorkflowContext):
        super().__init__(name="Enter custom mode")
        self.context = context

    def update(self) -> py_trees.common.Status:
        self.context.enter_custom_mode()
        return py_trees.common.Status.SUCCESS


class _RunSquatUntilStanding(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        context: SquatWorkflowContext,
        custom: Any,
        stable_ticks: int,
    ):
        super().__init__(name="Squat and wait for measured standing")
        self.context = context
        self.custom = custom
        self.required_stable_ticks = stable_ticks
        self.stable_ticks = 0
        self.crouch_requested = False
        self.stand_requested = False

    def initialise(self) -> None:
        self.stable_ticks = 0
        self.crouch_requested = False
        self.stand_requested = False

    def update(self) -> py_trees.common.Status:
        if self.context.current_mode != self.custom:
            self.stable_ticks = 0
            return py_trees.common.Status.RUNNING
        if not self.crouch_requested:
            self.context.discard_crouch_request()
            self.context.request_crouch()
            self.crouch_requested = True
        if self.context.consume_crouch_request():
            self.context.request_stand()
            self.stand_requested = True

        complete = (
            self.stand_requested
            and self.context.squat_has_started()
            and self.context.standing_pose_complete()
            and self.context.robot_is_standing()
        )
        self.stable_ticks = self.stable_ticks + 1 if complete else 0
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
    """Retain the original on-demand workflow for the standalone squat task."""
    if standing_stable_ticks < 1:
        raise ValueError("standing_stable_ticks must be at least one")

    squat = py_trees.composites.Sequence(
        name="Walking squat cycle",
        memory=True,
        children=[
            _WaitForCrouch(context, walking_mode),
            _StartSquatPolicy(context),
            _EnterSquatCustom(context),
            _RunSquatUntilStanding(
                context,
                custom_mode,
                standing_stable_ticks,
            ),
            _ReturnToWalking(context),
        ],
    )
    root = py_trees.composites.Selector(
        name="Booster deploy workflow",
        memory=False,
        children=[
            _SquatInactiveMode(context, walking_mode, custom_mode),
            squat,
        ],
    )
    return py_trees.trees.BehaviourTree(root)
