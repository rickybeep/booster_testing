from email.message import Message
from io import BytesIO
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from runtime import k1_control_server as control
from runtime import k1_runtime_health as runtime_health
from runtime import k1_motion_zero_once as zero_helper
from runtime.k1_guard_protocol import require_sdk_module_path, require_sdk_success


TOKEN = "offline-test-token-0123456789-ABCDEF"


@pytest.fixture(autouse=True)
def restore_head_state():
    with control.head_lock:
        original = dict(control.head_state)
    yield
    with control.head_lock:
        control.head_state.clear()
        control.head_state.update(original)


class FakeGuardClient:
    def __init__(self):
        self.calls = []

    def status(self):
        self.calls.append(("status",))
        return {"protocol": 1, "ready": True, "state": "READY_IDLE"}

    def acquire(self, lease_ms):
        self.calls.append(("acquire", lease_ms))
        return {"ready": True, "state": "ACTIVE"}

    def renew(self, **request):
        self.calls.append(("renew", request))
        return {"ready": True, "state": "ACTIVE"}

    def release(self, **request):
        self.calls.append(("release", request))
        return {"ready": True, "state": "RELEASED"}

    def move(self, **request):
        self.calls.append(("move", request))
        return {
            "ready": True,
            "state": "ACTIVE",
            "ack_velocity": [request["vx"], request["vy"], request["vyaw"]],
        }


def make_handler(path, *, body=None, token=TOKEN, peer="198.51.100.20"):
    handler = control.Handler.__new__(control.Handler)
    handler.path = path
    handler.client_address = (peer, 12345)
    handler.close_connection = False
    handler.headers = Message()
    if token is not None:
        handler.headers.add_header("Authorization", f"Bearer {token}")
    handler._body = lambda: dict(body or {})
    responses = []

    def reply(value, status=200, extra_headers=None):
        responses.append((status, value, dict(extra_headers or {})))

    handler._json = reply
    return handler, responses


def test_control_token_file_requires_one_nontrivial_line(tmp_path):
    token_file = tmp_path / "control.token"
    token_file.write_text(TOKEN + "\n", encoding="ascii")
    assert control.load_control_token(token_file) == TOKEN

    token_file.write_text("too-short\n", encoding="ascii")
    with pytest.raises(RuntimeError, match="32-512"):
        control.load_control_token(token_file)

    token_file.write_text(TOKEN + "\nsecond-line\n", encoding="ascii")
    with pytest.raises(RuntimeError, match="one 32-512"):
        control.load_control_token(token_file)


def test_only_loopback_health_bypasses_bearer(monkeypatch):
    monkeypatch.setattr(control, "control_token", TOKEN)
    monkeypatch.setattr(control, "guard_status", lambda: {"ready": True})
    monkeypatch.setattr(control, "read_nav", lambda: {"ts": 99.0})
    monkeypatch.setattr(control.time, "time", lambda: 100.0)

    remote, responses = make_handler("/api/health", token=None)
    remote.do_GET()
    assert responses[0][0] == 401
    assert responses[0][2]["WWW-Authenticate"].startswith("Bearer ")
    assert remote.close_connection is True

    local, responses = make_handler(
        "/api/health", token=None, peer="127.0.0.1"
    )
    local.do_GET()
    assert responses == [
        (200, {"guard": {"ready": True}, "nav_source_age_s": 1.0}, {})
    ]

    local_status, responses = make_handler(
        "/api/status", token=None, peer="127.0.0.1"
    )
    local_status.do_GET()
    assert responses[0][0] == 401


def test_remote_request_accepts_only_exact_bearer(monkeypatch):
    monkeypatch.setattr(control, "control_token", TOKEN)
    for supplied, expected in (
        (None, False),
        ("wrong-token-value-that-is-long-enough", False),
        (TOKEN + " ", False),
        (TOKEN, True),
    ):
        handler, _responses = make_handler("/api/status", token=supplied)
        assert handler._authorized("/api/status", "GET") is expected


@pytest.mark.parametrize(
    "headers",
    (
        (("Transfer-Encoding", "chunked"),),
        (("Content-Length", "2"), ("Content-Length", "2")),
        (("Content-Length", "2, 2"),),
    ),
)
def test_request_body_rejects_ambiguous_http_framing(headers):
    handler, _responses = make_handler("/api/move")
    del handler._body
    handler.headers = Message()
    for name, value in headers:
        handler.headers.add_header(name, value)
    handler.rfile = BytesIO(b"{}")
    with pytest.raises(ValueError):
        handler._body()


def test_mode_reader_rejects_stale_and_future_samples(tmp_path, monkeypatch):
    mode_file = tmp_path / "mode.json"
    monkeypatch.setattr(control, "MODE_FILE", mode_file)
    monkeypatch.setattr(control, "MODE_NAMES", {7: "walk"})
    monkeypatch.setattr(control.time, "time", lambda: 100.0)

    mode_file.write_text(json.dumps({"mode": 7, "ts": 99.5}), encoding="utf-8")
    assert control.read_mode() == ("walk", 0.5)

    mode_file.write_text(json.dumps({"mode": 7, "ts": 98.0}), encoding="utf-8")
    assert control.read_mode() == ("unknown", 2.0)

    mode_file.write_text(json.dumps({"mode": 7, "ts": 100.001}), encoding="utf-8")
    assert control.read_mode() == ("unknown", None)


def test_head_baseline_requires_fresh_nonlost_physical_feedback(monkeypatch):
    monkeypatch.setattr(control.time, "time", lambda: 100.0)
    telemetry = {
        "ts": 99.8,
        "joints": [
            {"name": "HeadYaw", "q": -0.2, "lost": False},
            {"name": "HeadPitch", "q": 0.1, "lost": False},
        ],
    }
    monkeypatch.setattr(control, "read_telemetry", lambda: telemetry)
    assert control.read_head_baseline() == pytest.approx((0.1, -0.2, 99.8, 0.2))

    telemetry["joints"][0]["lost"] = True
    with pytest.raises(control.HeadBaselineError, match="marked lost"):
        control.read_head_baseline()

    telemetry["joints"][0]["lost"] = False
    telemetry["ts"] = 99.0
    with pytest.raises(control.HeadBaselineError, match="fresh head telemetry"):
        control.read_head_baseline()

    telemetry["ts"] = 100.01
    with pytest.raises(control.HeadBaselineError, match="fresh head telemetry"):
        control.read_head_baseline()

    telemetry["ts"] = 99.8
    telemetry["joints"][0]["q"] = control.HEAD_YAW_LIMIT + 0.1
    with pytest.raises(control.HeadBaselineError, match="physical envelope"):
        control.read_head_baseline()


@pytest.mark.parametrize(
    ("path", "body"),
    (
        ("/api/guard/acquire", {"lease_ms": 400}),
        ("/api/guard/renew", {"generation": "g", "session": "s", "seq": 1}),
        (
            "/api/move",
            {
                "generation": "g",
                "session": "s",
                "seq": 1,
                "vx": 0.1,
                "vy": 0.0,
                "vyaw": 0.0,
            },
        ),
        ("/api/head", {"pitch": 0.0, "yaw": 0.1, "speed": 0.5}),
    ),
)
def test_nonwalking_mode_blocks_lease_extension_and_actuation(
    path, body, monkeypatch
):
    fake = FakeGuardClient()
    monkeypatch.setattr(control, "control_token", TOKEN)
    monkeypatch.setattr(control, "guard_client", fake)
    monkeypatch.setattr(control, "read_mode", lambda: ("prep", 0.1))
    handler, responses = make_handler(path, body=body)
    handler.do_POST()
    assert responses[0][0] == 409
    assert "fresh WALK" in responses[0][1]["error"]
    assert fake.calls == []


def test_nonwalking_mode_still_allows_zero_and_release(monkeypatch):
    fake = FakeGuardClient()
    monkeypatch.setattr(control, "control_token", TOKEN)
    monkeypatch.setattr(control, "guard_client", fake)
    monkeypatch.setattr(control, "read_mode", lambda: ("unknown", None))

    zero, responses = make_handler(
        "/api/move",
        body={
            "generation": "g",
            "session": "s",
            "seq": 1,
            "vx": 0.0,
            "vy": 0.0,
            "vyaw": 0.0,
        },
    )
    zero.do_POST()
    assert responses[0][0] == 200
    assert fake.calls[0][0] == "move"

    release, responses = make_handler(
        "/api/guard/release",
        body={"generation": "g", "session": "s", "seq": 2},
    )
    release.do_POST()
    assert responses[0][0] == 200
    assert fake.calls[1][0] == "release"


def test_queued_head_target_is_discarded_when_walk_gate_closes(monkeypatch):
    class FakeHead:
        def __init__(self):
            self.calls = []

        def RotateHead(self, pitch, yaw):
            self.calls.append((pitch, yaw))
            return 0

    fake = FakeHead()
    monkeypatch.setattr(control, "head_client", fake)
    monkeypatch.setattr(
        control,
        "require_fresh_walk",
        lambda: (_ for _ in ()).throw(control.ModeGateError("not WALK")),
    )
    monkeypatch.setattr(
        control, "read_head_baseline", lambda: (0.0, 0.0, 99.9, 0.1)
    )
    with control.head_lock:
        control.head_state.update(
            target_pitch=0.2,
            target_yaw=0.3,
            current_pitch=0.0,
            current_yaw=0.0,
            speed=0.5,
            pending=True,
            last_sdk_call_accepted=False,
            consecutive_failures=0,
            _command_id=1,
            _feedback_ts=99.9,
            _last_feedback_ts_used=None,
            _retry_at=0.0,
            last_error="",
        )
    assert control.head_sender_step(10.0, 0.05) == "mode_blocked"
    assert fake.calls == []
    assert control.head_state["target_pitch"] == 0.0
    assert control.head_state["target_yaw"] == 0.0
    assert control.head_state["pending"] is False
    assert control.head_state["last_error"] == "not WALK"


def test_head_post_requires_feedback_and_reports_queue_only(monkeypatch):
    fake = FakeHeadClient(rc=0)
    monkeypatch.setattr(control, "control_token", TOKEN)
    monkeypatch.setattr(control, "head_client", fake)
    monkeypatch.setattr(control, "read_mode", lambda: ("walk", 0.1))
    monkeypatch.setattr(control.time, "time", lambda: 100.0)
    with control.head_lock:
        control.head_state.update(
            target_pitch=None,
            target_yaw=None,
            current_pitch=None,
            current_yaw=None,
            current_source="unknown",
            pending=False,
            last_sdk_call_accepted=False,
            consecutive_failures=0,
            _command_id=0,
            _feedback_ts=None,
            _last_feedback_ts_used=None,
            _retry_at=0.0,
            last_rc=None,
            last_error="",
        )

    monkeypatch.setattr(
        control,
        "read_head_baseline",
        lambda: (_ for _ in ()).throw(
            control.HeadBaselineError("fresh head telemetry is unavailable")
        ),
    )
    handler, responses = make_handler(
        "/api/head", body={"pitch": 0.2, "yaw": 0.3, "speed": 0.5}
    )
    handler.do_POST()
    assert responses[0][0] == 409
    assert control.head_state["pending"] is False
    assert fake.calls == []

    monkeypatch.setattr(
        control, "read_head_baseline", lambda: (0.0, 0.0, 99.8, 0.2)
    )
    with control.head_lock:
        control.head_state.update(
            last_sdk_call_accepted=True,
            consecutive_failures=4,
            _retry_at=999.0,
            last_rc=777,
            last_error="older command failed",
        )
    handler, responses = make_handler(
        "/api/head", body={"pitch": 0.2, "yaw": 0.3, "speed": 0.5}
    )
    handler.do_POST()
    assert responses == [
        (
            200,
            {
                "rc": 0,
                "accepted": True,
                "queued": True,
                "acceptance": "queued_only",
                "sdk_call_completed": False,
            },
            {},
        )
    ]
    status = control.head_status(now=0.0)
    assert status["current_pitch"] == 0.0
    assert status["current_yaw"] == 0.0
    assert status["current_source"] == "telemetry"
    assert status["current_age_s"] == pytest.approx(0.2)
    assert status["pending"] is True
    assert status["last_sdk_call_accepted"] is False
    assert status["consecutive_failures"] == 0
    assert status["retry_in_s"] == 0.0
    assert status["last_rc"] is None
    assert status["last_error"] == ""
    assert fake.calls == []

    assert control.head_sender_step(1.0, 0.05) == "sent"
    status = control.head_status(now=1.0)
    # SDK acceptance is not reported as measured pose. Current remains the
    # telemetry baseline and the queue waits for a newer feedback sample.
    assert status["current_pitch"] == 0.0
    assert status["current_yaw"] == 0.0
    assert status["pending"] is True
    assert status["last_sdk_call_accepted"] is True
    assert fake.calls == [(0.025, 0.025)]
    assert control.head_sender_step(1.05, 0.05) == "awaiting_feedback"
    assert fake.calls == [(0.025, 0.025)]


class FakeHeadClient:
    def __init__(self, rc):
        self.rc = rc
        self.calls = []

    def RotateHead(self, pitch, yaw):
        self.calls.append((pitch, yaw))
        return self.rc


def test_head_failures_back_off_then_abandon(monkeypatch):
    fake = FakeHeadClient(rc=777)
    monkeypatch.setattr(control, "head_client", fake)
    monkeypatch.setattr(control, "require_fresh_walk", lambda: ("walk", 0.1))
    monkeypatch.setattr(
        control, "read_head_baseline", lambda: (0.0, 0.0, 99.9, 0.1)
    )
    monkeypatch.setattr(control.time, "time", lambda: 100.0)
    with control.head_lock:
        control.head_state.update(
            target_pitch=0.2,
            target_yaw=0.0,
            current_pitch=0.0,
            current_yaw=0.0,
            speed=1.0,
            pending=True,
            last_sdk_call_accepted=False,
            consecutive_failures=0,
            _command_id=1,
            _feedback_ts=99.9,
            _last_feedback_ts_used=None,
            _retry_at=0.0,
            last_rc=None,
            last_error="",
        )

    assert control.head_sender_step(0.0, 0.05) == "failed"
    assert control.head_sender_step(0.1, 0.05) == "backoff"
    assert len(fake.calls) == 1
    assert control.head_sender_step(0.25, 0.05) == "failed"
    assert control.head_sender_step(0.75, 0.05) == "failed"
    assert control.head_sender_step(1.75, 0.05) == "failed"
    assert control.head_sender_step(3.75, 0.05) == "abandoned"
    assert len(fake.calls) == control.HEAD_MAX_FAILURES
    status = control.head_status(now=3.75)
    assert status["pending"] is False
    assert status["last_sdk_call_accepted"] is False
    assert status["consecutive_failures"] == control.HEAD_MAX_FAILURES
    assert status["retry_in_s"] == 4.0
    assert "command abandoned" in status["last_error"]


@pytest.mark.parametrize("outcome", [0, RuntimeError("transport failed")])
def test_superseded_inflight_head_result_cannot_mutate_new_command(
    monkeypatch, outcome
):
    class SupersedingHead:
        def __init__(self):
            self.calls = []

        def RotateHead(self, pitch, yaw):
            self.calls.append((pitch, yaw))
            # Model an authenticated POST arriving while the vendor call is in
            # flight. The old call may already have crossed the SDK boundary,
            # but its result must not be attributed to the newer command.
            with control.head_lock:
                control.head_state.update(
                    target_pitch=-0.1,
                    target_yaw=0.4,
                    pending=True,
                    last_sdk_call_accepted=False,
                    consecutive_failures=0,
                    _command_id=2,
                    _last_feedback_ts_used=None,
                    _retry_at=0.0,
                    last_rc=None,
                    last_error="",
                )
            if isinstance(outcome, Exception):
                raise outcome
            return outcome

    fake = SupersedingHead()
    monkeypatch.setattr(control, "head_client", fake)
    monkeypatch.setattr(control, "require_fresh_walk", lambda: ("walk", 0.1))
    monkeypatch.setattr(
        control, "read_head_baseline", lambda: (0.0, 0.0, 99.9, 0.1)
    )
    monkeypatch.setattr(control.time, "time", lambda: 100.0)
    with control.head_lock:
        control.head_state.update(
            target_pitch=0.2,
            target_yaw=0.0,
            current_pitch=0.0,
            current_yaw=0.0,
            speed=1.0,
            pending=True,
            last_sdk_call_accepted=False,
            consecutive_failures=0,
            _command_id=1,
            _feedback_ts=99.9,
            _last_feedback_ts_used=None,
            _retry_at=0.0,
            last_rc=None,
            last_error="",
        )

    assert control.head_sender_step(0.0, 0.05) == "superseded"
    assert fake.calls == [(0.05, 0.0)]
    with control.head_lock:
        assert control.head_state["_command_id"] == 2
        assert control.head_state["target_pitch"] == -0.1
        assert control.head_state["target_yaw"] == 0.4
        assert control.head_state["pending"] is True
        assert control.head_state["last_sdk_call_accepted"] is False
        assert control.head_state["consecutive_failures"] == 0
        assert control.head_state["_retry_at"] == 0.0
        assert control.head_state["last_rc"] is None
        assert control.head_state["last_error"] == ""


class FakeZeroClient:
    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.calls = []

    def Move(self, vx, vy, vyaw):
        self.calls.append((vx, vy, vyaw))
        outcome = self.outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return outcome


def test_shutdown_zero_helper_attempts_and_verifies_entire_burst():
    client = FakeZeroClient([None, 0, 100, RuntimeError("code = 400"), 0])
    sleeps = []
    assert zero_helper.deliver_zero_burst(client, sleep=sleeps.append) == 0
    assert client.calls == [(0.0, 0.0, 0.0)] * zero_helper.ZERO_ATTEMPTS
    assert sleeps == [zero_helper.ZERO_INTERVAL_S] * 4


def test_shutdown_zero_helper_reports_failure_but_finishes_burst():
    client = FakeZeroClient([0, RuntimeError("transport timeout"), 0, 777, 0])
    assert zero_helper.deliver_zero_burst(client, sleep=lambda _seconds: None) == 1
    assert client.calls == [(0.0, 0.0, 0.0)] * zero_helper.ZERO_ATTEMPTS


@pytest.mark.parametrize("result", [None, 0])
def test_sdk_initialization_accepts_only_documented_success_shapes(result):
    require_sdk_success(result, "offline fake init")


@pytest.mark.parametrize("result", [False, True, 1, -1, "0", object()])
def test_sdk_initialization_rejects_ambiguous_results(result):
    with pytest.raises(RuntimeError, match="unexpected result"):
        require_sdk_success(result, "offline fake init")


def test_sdk_module_must_resolve_under_explicit_expected_root(tmp_path, monkeypatch):
    sdk_root = tmp_path / "k1-sdk"
    sdk_root.mkdir()
    module_file = sdk_root / "booster_robotics_sdk_python.so"
    module_file.touch()
    monkeypatch.setenv("K1_SDK_EXPECTED_ROOT", str(sdk_root))
    assert require_sdk_module_path(SimpleNamespace(__file__=str(module_file))) == str(
        module_file.resolve()
    )

    outside = tmp_path / "old-global-sdk.so"
    outside.touch()
    with pytest.raises(RuntimeError, match="outside K1_SDK_EXPECTED_ROOT"):
        require_sdk_module_path(SimpleNamespace(__file__=str(outside)))

    monkeypatch.delenv("K1_SDK_EXPECTED_ROOT")
    with pytest.raises(RuntimeError, match="must be an absolute path"):
        require_sdk_module_path(SimpleNamespace(__file__=str(module_file)))


def test_runtime_health_reports_readiness_not_motion_authority(monkeypatch):
    class ReadyGuard:
        def status(self):
            return {"ready": True, "state": "READY_IDLE"}

    report = runtime_health.check_guard(ReadyGuard())
    assert report["guard_ready"] is True
    assert "motion_allowed" not in report

    monkeypatch.setattr(runtime_health, "check_guard", lambda: report)
    monkeypatch.setattr(
        runtime_health,
        "check_server",
        lambda: {"component": "control_server", "healthy": True, "status": {}},
    )
    combined = runtime_health.health_report()
    assert combined["guard_ready"] is True
    assert "motion_allowed" not in combined


def test_mode_qos_and_candidate_systemd_are_fail_safe():
    root = Path(__file__).resolve().parents[1] / "robot" / "runtime"
    watcher = (root / "k1_mode_watcher.py").read_text(encoding="utf-8")
    assert "ReliabilityPolicy.BEST_EFFORT" in watcher
    assert "HistoryPolicy.KEEP_LAST" in watcher
    assert "depth=1" in watcher

    target = (root / "systemd" / "k1-panel-v10.target").read_text(
        encoding="utf-8"
    )
    assert "AllowIsolate" not in target
    assert "WantedBy=multi-user.target" not in target

    for name in (
        "k1-control-server.service",
        "k1-mode-watcher.service",
        "k1-motion-guard.service",
        "k1-telemetry.service",
    ):
        service = (root / "systemd" / name).read_text(encoding="utf-8")
        assert "Restart=on-failure" in service
        assert "StartLimitIntervalSec=30s" in service
        assert "StartLimitBurst=3" in service
        assert "Restart=always" not in service
        assert "EnvironmentFile=/etc/booster-track-v10/runtime.env" in service
        assert "K1_PYTHON" in service

    control_service = (
        root / "systemd" / "k1-control-server.service"
    ).read_text(encoding="utf-8")
    assert "K1_TRACK_TOKEN_FILE=/etc/booster-track-v10/control.token" in control_service
    assert "K1_TRACK_RUNTIME_DIR=/run/booster-track-v10" in control_service

    for name in ("k1-mode-watcher.service", "k1-telemetry.service"):
        service = (root / "systemd" / name).read_text(encoding="utf-8")
        assert "Requires=k1-motion-guard.service" in service
        assert "After=k1-motion-guard.service" in service
        assert "K1_TRACK_RUNTIME_DIR=/run/booster-track-v10" in service

    manifest = json.loads(
        (root / "runtime_manifest.json").read_text(encoding="utf-8")
    )
    assert manifest["mode_status"] == "/run/booster-track-v10/mode.json"
    assert manifest["nav_status"] == "/run/booster-track-v10/nav.json"
    assert manifest["telemetry_status"] == "/run/booster-track-v10/telemetry.json"

    runtime_env = (root / "runtime.env.example").read_text(encoding="utf-8")
    for variable in (
        "K1_PYTHON=",
        "PYTHONPATH=",
        "K1_SDK_EXPECTED_ROOT=",
        "ROS_SETUP=",
        "BOOSTER_ROS_SETUP=",
    ):
        assert variable in runtime_env
