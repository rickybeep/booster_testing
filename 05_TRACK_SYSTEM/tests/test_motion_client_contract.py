from booster_motion import MotionController


def active_controller():
    motion = MotionController(hosts=["192.0.2.10"], panel_port=8080, robot_name="example")
    motion._clock = lambda: 50.0
    with motion._lock:
        motion._connected = True
        motion._armed = True
        motion._host = "192.0.2.10"
        motion._actual_mode = "walk"
        motion._mode_valid_mono = 50.0
        motion._yaw = 0.0
        motion._yaw_progress_mono = 50.0
        motion._guard_generation = "generation"
        motion._guard_session = "session"
        motion._guard_status = {"state": "ACTIVE", "ready": True, "lease_ms": 400}
        motion._lease_proof_mono = 50.0
    return motion


def status(*, seq=1, session="session", velocity=(0.1, 0.0, -0.2)):
    return {
        "protocol": 1,
        "state": "ACTIVE",
        "ready": True,
        "generation": "generation",
        "session": session,
        "lease_ms": 400,
        "seq": seq,
        "desired_velocity": list(velocity),
        "ack_velocity": list(velocity),
    }


def test_periodic_status_without_session_is_health_not_ownership_proof():
    motion = active_controller()
    periodic = status()
    periodic.pop("session")
    assert motion._observe_guard_status(periodic)
    assert motion._lease_proof_mono == 50.0


def test_move_proof_requires_matching_session_and_sequence():
    motion = active_controller()
    missing = status()
    missing.pop("session")
    assert not motion._observe_guard_status(missing, ownership_proof=True, expected_seq=1)
    assert motion.hard_fault

    motion = active_controller()
    assert not motion._observe_guard_status(status(seq=2), ownership_proof=True, expected_seq=1)
    assert motion.hard_fault


def test_nonzero_command_requires_top_level_and_velocity_ack():
    motion = active_controller()
    motion._move_post = lambda *_args, **_kwargs: {"rc": 0, "guard": status()}
    motion.command(0.1, 0.2)
    assert motion.last_command == (0.1, 0.2)
    assert not motion.hard_fault

    motion = active_controller()
    wrong = status(velocity=(0.0, 0.0, 0.0))
    motion._move_post = lambda *_args, **_kwargs: {"rc": 0, "guard": wrong}
    motion.command(0.1, 0.2)
    assert motion.hard_fault
    assert motion.last_command == (0.0, 0.0)


def test_nonzero_command_fails_closed_when_walk_feedback_is_stale():
    motion = active_controller()
    called = []
    motion._move_post = lambda *_args, **_kwargs: called.append(True)
    with motion._lock:
        motion._mode_valid_mono = 40.0
    motion.command(0.1, 0.0)
    assert not called
    assert motion.hard_fault
    assert "transport was unavailable" in motion.last_error


def test_hard_fault_requires_explicit_reconnect_before_rearm():
    motion = active_controller()
    motion._hard_guard_fault("ambiguous move acknowledgement")
    with motion._lock:
        motion._guard_status = {
            "generation": "generation",
            "state": "READY_IDLE",
            "ready": True,
        }
    called = []
    motion._guard_control_post = lambda *_args, **_kwargs: called.append(True)
    assert motion.arm() is False
    assert not called
    assert "fresh reconnect" in motion.last_error


def test_session_health_includes_feedback_and_lease_freshness():
    now = [50.0]
    motion = active_controller()
    motion._clock = lambda: now[0]
    assert motion.session_healthy
    now[0] = 50.401
    assert not motion.session_healthy


def test_head_requires_an_owned_healthy_walk_session():
    motion = active_controller()
    called = []
    motion._aux_session.post = lambda *_args, **_kwargs: called.append(True)
    with motion._lock:
        motion._armed = False
        motion._guard_session = None
    assert motion.head(0.0, 0.0) is False
    assert not called


class _Response:
    def __init__(self, payload, status_code=200):
        self.payload = payload
        self.status_code = status_code

    def raise_for_status(self):
        return None

    def json(self):
        return self.payload


def _nav_payload(timestamp, yaw=0.25):
    return {
        "ts": timestamp,
        "yaw": yaw,
        "gyro_z": 0.0,
        "odom": {"x": 1.0, "y": 2.0, "theta": 0.0},
        "rate_hz": 20.0,
    }


def test_sessions_disable_environment_proxies_and_send_bearer():
    token = "offline-client-token-0123456789-ABCDEF"
    motion = MotionController(["192.0.2.10"], auth_token=token)
    for session in (
        motion._move_session,
        motion._poll_session,
        motion._aux_session,
    ):
        assert session.trust_env is False
        assert session.headers["Authorization"] == f"Bearer {token}"


def test_host_probe_does_not_follow_or_accept_redirects():
    motion = MotionController(["192.0.2.10"])
    calls = []

    def get(*_args, **kwargs):
        calls.append(kwargs)
        return _Response(
            {
                "capabilities": {
                    "motion_guard": True,
                    "motion_guard_protocol": 1,
                }
            },
            status_code=302,
        )

    motion._poll_session.get = get
    assert motion._resolve_host() is None
    assert calls == [{"timeout": 1.0, "allow_redirects": False}]


def test_nav_requires_timestamp_progress_after_first_sample():
    motion = MotionController(["192.0.2.10"])
    now = [10.0]
    motion._clock = lambda: now[0]
    with motion._lock:
        motion._host = "192.0.2.10"
    payload = [_nav_payload(90.0), _nav_payload(90.0), _nav_payload(90.1)]
    motion._poll_session.get = lambda *_args, **_kwargs: _Response(payload.pop(0))
    motion._poll_nav()
    assert motion.heading_deg() is None
    now[0] = 10.1
    motion._poll_nav()
    assert motion.heading_deg() is None
    now[0] = 10.2
    motion._poll_nav()
    assert motion.heading_deg() is not None


def test_nav_timestamp_regression_does_not_refresh_heading():
    motion = MotionController(["192.0.2.10"])
    now = [10.0]
    motion._clock = lambda: now[0]
    with motion._lock:
        motion._host = "192.0.2.10"
    payload = [
        _nav_payload(99.8, 0.20),
        _nav_payload(99.9, 0.25),
        _nav_payload(99.8, 1.0),
    ]
    motion._poll_session.get = lambda *_args, **_kwargs: _Response(
        payload.pop(0)
    )
    motion._poll_nav()
    now[0] = 10.1
    motion._poll_nav()
    assert motion.heading_deg() is not None
    now[0] = 10.2
    motion._poll_nav()
    with motion._lock:
        assert motion._yaw == 0.25
        assert motion._yaw_progress_mono == 10.1
    assert "timestamp regressed" in motion.last_error


def test_successful_nav_cannot_erase_guard_hard_fault():
    motion = MotionController(["192.0.2.10"])
    motion._clock = lambda: 10.0
    with motion._lock:
        motion._host = "192.0.2.10"
    motion._hard_guard_fault("ambiguous release")
    motion._poll_session.get = lambda *_args, **_kwargs: _Response(
        _nav_payload(99.9)
    )
    motion._poll_nav()
    assert motion.hard_fault
    assert "ambiguous release" in motion.last_error
