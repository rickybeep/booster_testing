import socket

import pytest

from runtime import k1_motion_guard as guard_module
from runtime.k1_motion_guard import GuardServer, MotionGuard


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeSDK:
    def __init__(self, results=None, error=None):
        self.calls = []
        self.results = list(results or [])
        self.error = error

    def Move(self, vx, vy, vyaw):
        self.calls.append((vx, vy, vyaw))
        if self.error is not None:
            raise RuntimeError(self.error)
        return self.results.pop(0) if self.results else None


def make_guard(sdk=None):
    clock = FakeClock()
    sdk = sdk or FakeSDK()
    guard = MotionGuard(sdk.Move, clock=clock, sleep=clock.sleep, generation="test", status_path=None)
    return guard, sdk, clock


def acquire(guard):
    guard.startup()
    return guard.handle({"op": "acquire", "lease_ms": 400})


def move(guard, owner, seq, vx=0.2):
    return guard.handle({
        "op": "move", "generation": owner["generation"],
        "session": owner["session"], "seq": seq,
        "vx": vx, "vy": 0.0, "vyaw": 0.1,
    })


def test_expiry_sends_exactly_five_zeros_then_silence():
    guard, sdk, clock = make_guard()
    owner = acquire(guard)
    move(guard, owner, 1)
    clock.now += 0.401
    guard.tick()
    assert guard.status()["state"] == "EXPIRED"
    assert sdk.calls[-5:] == [(0.0, 0.0, 0.0)] * 5
    count = len(sdk.calls)
    clock.now += 10.0
    guard.tick()
    assert len(sdk.calls) == count


@pytest.mark.parametrize("result", [100, 400])
def test_only_known_nonwalking_zero_return_codes_are_tolerated(result):
    guard, _sdk, _clock = make_guard(FakeSDK(results=[result]))
    status = guard.startup()
    assert status["ready"] is True
    assert status["last_zero_outcome"] == f"rejected_safe_mode_rc_{result}"


def test_unknown_zero_return_code_degrades_instead_of_claiming_ready():
    guard, _sdk, _clock = make_guard(FakeSDK(results=[777]))
    with pytest.raises(RuntimeError, match="rc=777"):
        guard.startup()
    assert guard.status()["state"] == "DEGRADED"
    assert guard.status()["ready"] is False


def test_unknown_zero_exception_degrades_instead_of_claiming_ready():
    guard, _sdk, _clock = make_guard(FakeSDK(error="transport timeout"))
    with pytest.raises(RuntimeError, match="transport timeout"):
        guard.startup()
    assert guard.status()["ready"] is False


def test_known_raised_nonwalking_code_is_tolerated():
    guard, _sdk, _clock = make_guard(FakeSDK(error="API call failed, code = 400"))
    status = guard.startup()
    assert status["ready"] is True
    assert status["last_zero_outcome"] == "rejected_safe_mode_rc_400"


def test_nonwalking_startup_rejection_does_not_claim_zero_acknowledgement():
    guard, _sdk, _clock = make_guard(FakeSDK(results=[400]))
    status = guard.startup()
    assert status["ready"] is True
    assert status["ack_velocity"] is None


def test_acquire_requires_a_fresh_delivered_zero():
    guard, sdk, _clock = make_guard(FakeSDK(results=[400, 0]))
    guard.startup()
    owner = guard.handle({"op": "acquire", "lease_ms": 400})
    assert owner["state"] == "ACTIVE"
    assert owner["ack_velocity"] == [0.0, 0.0, 0.0]
    assert sdk.calls == [(0.0, 0.0, 0.0)] * 2


def test_acquire_rejection_never_creates_active_ownership():
    guard, _sdk, _clock = make_guard(FakeSDK(results=[400, 400]))
    guard.startup()
    with pytest.raises(RuntimeError, match="rc=400"):
        guard.handle({"op": "acquire", "lease_ms": 400})
    status = guard.status(include_session=True)
    assert status["state"] == "DEGRADED"
    assert status["ready"] is False
    assert status["session"] is None


def test_move_deduplication_requires_matching_acknowledgement():
    guard, sdk, _clock = make_guard()
    owner = acquire(guard)
    move(guard, owner, 1)
    calls = len(sdk.calls)
    guard._ack = None
    move(guard, owner, 2)
    assert len(sdk.calls) == calls + 1
    assert sdk.calls[-1] == (0.2, 0.0, 0.1)


def test_partial_client_frame_is_bounded_then_expiry_can_run(monkeypatch):
    guard, sdk, clock = make_guard()
    owner = acquire(guard)
    move(guard, owner, 1)
    clock.now += 0.401
    monkeypatch.setattr(guard_module, "validate_local_peer", lambda *_args: None)

    class PartialPeer:
        def __init__(self):
            self.timeout = None
            self.responses = []

        def settimeout(self, value):
            self.timeout = value

        def recv(self, _size):
            raise socket.timeout("partial frame")

        def sendall(self, value):
            self.responses.append(value)

    peer = PartialPeer()
    GuardServer(guard, socket_path="unused").serve_connection(peer)
    guard.tick()
    assert peer.timeout == guard_module.CLIENT_READ_TIMEOUT_S
    assert guard.status()["state"] == "EXPIRED"
    assert sdk.calls[-5:] == [(0.0, 0.0, 0.0)] * 5
