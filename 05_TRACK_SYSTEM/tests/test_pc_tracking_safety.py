from dataclasses import replace
from pathlib import Path
import sys
import threading
import types

import pytest
import booster_config as cfg
import track_uwb as uwb_module

from arena_runner import (
    LIVE_ACK,
    CommandSmoother,
    LiveFleet,
    LiveRobot,
    Observation,
    PatrolController,
    validate_live_gate,
)
from tracker_config import ConfigError, load_config, parse_config, read_auth_token
from track_uwb import UwbMeasurement, UwbTracker, parse_position_line


ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "config" / "tracker.example.json"
ANCHORS = [(0.0, 0.0), (0.0, 5.0), (5.0, 5.0), (5.0, 0.0)]


def example():
    config = load_config(EXAMPLE)
    config.apply_navigation_defaults()
    return config


def live_tracker(tmp_path, *, token="t" * 32, port="COM9"):
    token_file = tmp_path / "control-token.local.txt"
    token_file.write_text(token + "\n", encoding="utf-8")
    config = example()
    robot = replace(
        config.robots[0],
        enabled=True,
        panel_hosts=("10.23.45.67",),
        auth_token_file=token_file.name,
    )
    return replace(
        config,
        path=tmp_path / "tracker.local.json",
        live_motion_enabled=True,
        uwb_port=port,
        robots=(robot,),
    )


def test_position_parser_rejects_missing_role_instead_of_assigning_tag_zero():
    assert parse_position_line("LO=[1.0,2.0,0.0]", ANCHORS) is None
    parsed = parse_position_line("t7:0 LO=[1.0,2.0,0.0]", ANCHORS)
    assert parsed == (7, (1.0, 2.0), 0, 0.0, "lo")


def test_measurement_returns_position_and_quality_from_one_immutable_snapshot():
    tracker = UwbTracker("unused", 115200, ANCHORS, use_kf=False)
    with tracker.lock:
        tracker._update(7, (1.0, 2.0), 4, 0.125, "mc")
    measurement = tracker.measurement(7)
    assert measurement.position == (1.0, 2.0)
    assert measurement.n_anchors == 4
    assert measurement.residual_m == 0.125
    assert measurement.source == "mc"

    with tracker.lock:
        tracker._update(7, (1.1, 2.1), 3, 0.25, "mc")
    assert measurement.position == (1.0, 2.0)
    assert measurement.n_anchors == 4
    assert measurement.residual_m == 0.125


@pytest.mark.parametrize(
    "bad_quality",
    [
        {"n_anchors": 3, "residual": 0.1, "source": "mc"},
        {"n_anchors": 4, "residual": 0.75, "source": "mc"},
        {"n_anchors": 0, "residual": 0.0, "source": "lo"},
    ],
)
def test_rejected_quality_never_contaminates_position_filter(bad_quality):
    tracker = UwbTracker(
        "unused",
        115200,
        ANCHORS,
        use_kf=False,
        minimum_anchors=4,
        maximum_residual_m=0.5,
        allow_unverified_lo=False,
    )
    with tracker.lock:
        tracker._update(7, (100.0, 100.0), **bad_quality)
        tracker._update(7, (1.0, 2.0), 4, 0.1, "mc")
    accepted = tracker.measurement(7)
    assert accepted.position == (1.0, 2.0)
    assert accepted.n_anchors == 4
    assert accepted.residual_m == 0.1
    assert tracker.quality_rejects == 1


@pytest.mark.parametrize(
    "n_anchors,residual",
    [(-1, 0.1), (4, -0.1)],
)
def test_negative_quality_is_rejected_before_creating_filter_state(
    n_anchors,
    residual,
):
    tracker = UwbTracker(
        "unused",
        115200,
        ANCHORS,
        use_kf=False,
        minimum_anchors=4,
        maximum_residual_m=0.5,
        allow_unverified_lo=False,
    )
    with tracker.lock:
        tracker._update(7, (100.0, 100.0), n_anchors, residual, "mc")
    assert tracker.measurement(7).position is None
    assert 7 not in tracker.tags
    assert tracker.quality_rejects == 1


def test_live_fleet_passes_quality_policy_into_uwb_tracker(monkeypatch):
    captured = {}

    class FakeTracker:
        def __init__(self, *args, **kwargs):
            captured["args"] = args
            captured["kwargs"] = kwargs

    monkeypatch.setattr(uwb_module, "UwbTracker", FakeTracker)
    config = replace(example(), uwb_port="COM9")
    fleet = LiveFleet(config)
    assert fleet.uwb is not None
    assert captured["kwargs"] == {
        "dz": config.anchor_to_tag_height_m,
        "minimum_anchors": config.minimum_anchors,
        "maximum_residual_m": config.maximum_residual_m,
        "allow_unverified_lo": config.allow_unverified_lo,
    }


class MeasurementSource:
    def __init__(self, measurement):
        self.value = measurement
        self.calls = 0

    def measurement(self, _tag):
        self.calls += 1
        return self.value


def sample_robot(config):
    return types.SimpleNamespace(
        config=config.robots[0],
        motion=types.SimpleNamespace(heading_deg=lambda: 0.0),
    )


def sample_fleet(config, measurement):
    fleet = LiveFleet.__new__(LiveFleet)
    fleet.config = config
    fleet.uwb = MeasurementSource(measurement)
    return fleet


def measurement(*, timestamp=100.0, source="mc", anchors=4, residual=0.1):
    return UwbMeasurement(
        position=(2.0, 2.0),
        trail=(),
        timestamp=timestamp,
        n_anchors=anchors,
        residual_m=residual,
        source=source,
    )


def test_sample_uses_one_atomic_measurement_and_rejects_future_time():
    config = example()
    fleet = sample_fleet(config, measurement(timestamp=100.01))
    observation = fleet._sample(sample_robot(config), wall_now=100.0)
    assert not observation.trusted
    assert observation.reason == "future-dated fix"
    assert fleet.uwb.calls == 1


def test_device_solved_lo_requires_explicit_opt_in():
    config = example()
    fleet = sample_fleet(config, measurement(source="lo", anchors=0, residual=0.0))
    blocked = fleet._sample(sample_robot(config), wall_now=100.0)
    assert not blocked.trusted
    assert "not explicitly allowed" in blocked.reason

    allowed_config = replace(config, allow_unverified_lo=True)
    fleet = sample_fleet(allowed_config, measurement(source="lo", anchors=0, residual=0.0))
    accepted = fleet._sample(sample_robot(allowed_config), wall_now=100.0)
    assert accepted.trusted
    assert accepted.position == (2.0, 2.0)


def test_mc_measurement_enforces_anchor_count_and_residual():
    config = example()
    robot = sample_robot(config)
    too_few = sample_fleet(config, measurement(anchors=3))
    assert too_few._sample(robot, wall_now=100.0).reason == "only 3 anchors"
    noisy = sample_fleet(config, measurement(residual=0.75))
    assert "residual" in noisy._sample(robot, wall_now=100.0).reason


def test_patrol_holds_outside_safe_boundary_before_calling_navigator():
    config = example()
    robot = config.robots[0]
    patrol = PatrolController(robot, config, lambda: ())
    patrol.start()
    patrol.phase = "WALK"
    patrol.navigator.update = lambda *_args, **_kwargs: pytest.fail(
        "Navigator must not run outside the safe boundary"
    )
    x = config.arena[0] + config.safe_margin_m - 0.01
    intent = patrol.step(
        1.0,
        (x, 2.0),
        0.0,
        obstacle_feed_healthy=True,
    )
    assert (intent.forward_mps, intent.turn_rad_s, intent.phase) == (0.0, 0.0, "HOLD")
    assert intent.reason == "outside safe arena boundary"


def test_command_smoother_reverses_through_a_zero_tick():
    smoother = CommandSmoother(forward_slew_per_s=1.0, turn_slew_per_s=1.0)
    smoother.step(1.0, 1.0, 10.0)
    assert smoother.step(1.0, 1.0, 10.1) == pytest.approx((0.1, 0.1))
    assert smoother.step(-1.0, -1.0, 10.2) == (0.0, 0.0)
    assert smoother.step(-1.0, -1.0, 10.3) == pytest.approx((-0.1, -0.1))


class FakeMotion:
    def __init__(self):
        self.connected_count = 0
        self.stop_count = 0
        self.shutdown_count = 0

    def connect(self):
        self.connected_count += 1

    def stop(self):
        self.stop_count += 1

    def shutdown(self):
        self.shutdown_count += 1


class FakeUwb:
    def __init__(self, *, fail_start=False):
        self.running = True
        self.started = False
        self.fail_start = fail_start

    def start(self):
        if self.fail_start:
            raise RuntimeError("injected UWB startup failure")
        self.started = True

    def is_alive(self):
        return False


def cleanup_fleet(*, fail_start=False):
    fleet = LiveFleet.__new__(LiveFleet)
    fleet.config = types.SimpleNamespace(control_hz=20)
    fleet.uwb = FakeUwb(fail_start=fail_start)
    motion = FakeMotion()
    fleet.robots = [
        types.SimpleNamespace(
            config=types.SimpleNamespace(name="test"),
            motion=motion,
            readiness_error=lambda: "not ready",
        )
    ]
    fleet.stop_event = threading.Event()
    fleet._refresh_obstacles = lambda: False
    return fleet, motion


def test_live_fleet_cleanup_runs_after_cancelled_preflight():
    fleet, motion = cleanup_fleet()
    fleet.stop_event.set()
    assert fleet.run() == 0
    assert fleet.uwb.started
    assert fleet.uwb.running is False
    assert motion.connected_count == 1
    assert motion.stop_count == 1
    assert motion.shutdown_count == 1


def test_live_fleet_cleanup_runs_when_uwb_start_raises():
    fleet, motion = cleanup_fleet(fail_start=True)
    with pytest.raises(RuntimeError, match="UWB startup failure"):
        fleet.run()
    assert fleet.uwb.running is False
    assert motion.stop_count == 1
    assert motion.shutdown_count == 1


@pytest.mark.parametrize(
    "mutation,match",
    [
        (lambda value: value["uwb"].update(position_stale_s=0.91), "deadman"),
        (lambda value: value["arena"].update(safe_margin_m=2.55), "recovery hysteresis"),
        (
            lambda value: value["robots"][0].update(enabled=True, keepout_radius_m=1.6),
            "keepout_radius_m",
        ),
        (
            lambda value: value["robots"][0].update(enabled=True, nav_yaw_stale_s=1.3),
            "mode freshness",
        ),
        (lambda value: value["uwb"].update(allow_unverified_lo="yes"), "JSON boolean"),
    ],
)
def test_coupled_safety_configuration_fails_closed(mutation, match):
    import json

    value = json.loads(EXAMPLE.read_text(encoding="utf-8"))
    mutation(value)
    with pytest.raises(ConfigError, match=match):
        parse_config(value)


def test_wall_clearance_budget_accepts_exact_boundary_and_rejects_below():
    import json

    value = json.loads(EXAMPLE.read_text(encoding="utf-8"))
    value["robots"][0]["enabled"] = True
    boundary_keepout = (
        value["arena"]["safe_margin_m"]
        - value["navigation"]["max_forward_mps"] * cfg.DEADMAN_SECONDS
        - value["uwb"]["maximum_residual_m"]
    )
    value["robots"][0]["keepout_radius_m"] = boundary_keepout
    parse_config(value)

    value["robots"][0]["keepout_radius_m"] = boundary_keepout + 0.001
    with pytest.raises(ConfigError, match="one motion-deadman interval"):
        parse_config(value)


def test_live_gate_requires_explicit_uwb_port(tmp_path):
    tracker = live_tracker(tmp_path, port="AUTO")
    with pytest.raises(ConfigError, match="explicit uwb.port"):
        validate_live_gate(tracker, LIVE_ACK)


@pytest.mark.parametrize(
    "token",
    [
        "short",
        "x" * 31,
        "x" * 32 + " ",
        "x" * 31 + " y",
        "x" * 513,
        "x" * 32 + "\nsecond-line",
    ],
)
def test_live_gate_rejects_weak_or_whitespace_auth_tokens(tmp_path, token):
    tracker = live_tracker(tmp_path, token=token)
    with pytest.raises(ConfigError, match="32-512"):
        validate_live_gate(tracker, LIVE_ACK)


def test_live_gate_accepts_local_token_and_live_robot_passes_it_to_client(
    tmp_path,
    monkeypatch,
):
    tracker = live_tracker(tmp_path)
    validate_live_gate(tracker, LIVE_ACK)
    assert read_auth_token(tracker, tracker.robots[0]) == "t" * 32

    captured = {}

    class FakeController:
        def __init__(self, **kwargs):
            captured.update(kwargs)

        def set_imu_calibration(self, **_kwargs):
            pass

        def set_caps(self, *_args):
            pass

    monkeypatch.setitem(
        sys.modules,
        "booster_motion",
        types.SimpleNamespace(MotionController=FakeController),
    )
    LiveRobot(tracker.robots[0], tracker)
    assert captured["auth_token"] == "t" * 32
    assert captured["hosts"] == ("10.23.45.67",)


def test_live_gate_rejects_symbolic_link_auth_token(tmp_path, monkeypatch):
    target = tmp_path / "real-token.txt"
    target.write_text("t" * 32 + "\n", encoding="ascii")
    link = tmp_path / "linked-token.txt"
    try:
        link.symlink_to(target)
    except OSError:
        # Standard Windows accounts may not hold symlink privileges. Simulate
        # only that filesystem predicate so this fail-closed branch is still
        # exercised on every development host.
        link.write_text("t" * 32 + "\n", encoding="ascii")
        lexical_link = link.absolute()
        real_is_symlink = Path.is_symlink

        def is_symlink(path):
            return path == lexical_link or real_is_symlink(path)

        monkeypatch.setattr(Path, "is_symlink", is_symlink)
    tracker = live_tracker(tmp_path)
    tracker = replace(
        tracker,
        robots=(replace(tracker.robots[0], auth_token_file=link.name),),
    )
    with pytest.raises(ConfigError, match="symbolic link"):
        validate_live_gate(tracker, LIVE_ACK)


def test_live_gate_rechecks_relationships_after_dataclass_replacement(tmp_path):
    tracker = live_tracker(tmp_path)
    unsafe = replace(
        tracker,
        robots=(replace(tracker.robots[0], keepout_radius_m=1.6),),
    )
    with pytest.raises(ConfigError, match="keepout_radius_m"):
        validate_live_gate(unsafe, LIVE_ACK)


def test_readiness_accepts_a_healthy_owned_active_guard_session():
    robot = LiveRobot.__new__(LiveRobot)
    robot.motion = types.SimpleNamespace(
        connected=True,
        mode_fresh=True,
        actual_mode="walk",
        heading_deg=lambda: 0.0,
        guard_acquirable=False,
        session_healthy=True,
    )
    robot.observation = Observation((2.0, 2.0), 100.0, True)
    assert robot.readiness_error() == ""
