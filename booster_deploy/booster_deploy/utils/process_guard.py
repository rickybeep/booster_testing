from __future__ import annotations

import os


class ProcessOwner:
    """Pin fork-unsafe resources to the process that created them.

    ROS 2's middleware and the Booster SDK client cannot be reused after
    ``fork()``. Deployment restarts the policy worker as a forked child for every
    crouch, so the child inherits live publisher, subscription, and client
    objects that it must never touch. Reusing them corrupts the middleware
    instead of raising, which is why the invariant is asserted explicitly rather
    than left to convention.
    """

    def __init__(self, name: str) -> None:
        self.name = name
        self.pid = os.getpid()

    def is_owner(self) -> bool:
        return os.getpid() == self.pid

    def assert_owner(self, action: str) -> None:
        if self.is_owner():
            return
        raise RuntimeError(
            f"{action} attempted in process {os.getpid()}, but {self.name} is "
            f"owned by process {self.pid}. ROS 2 communication and the SDK "
            "client cannot be reused after fork()."
        )
