import json
import math
from pathlib import Path

from arena_runner import CommandSmoother, PatrolController, project_tag_position
from booster_nav import Navigator, peer_hold_active
from tracker_config import load_config
from track_uwb import UwbTracker


ROOT = Path(__file__).resolve().parents[1]


def example():
    config = load_config(ROOT / "config" / "tracker.example.json")
    config.apply_navigation_defaults()
    return config


def test_tag_offset_projection_uses_calibrated_heading():
    assert project_tag_position((1.0, 1.0), 0.0, 0.2, 0.1) == (1.2, 1.1)
    x, y = project_tag_position((1.0, 1.0), 90.0, 0.2, 0.1)
    assert math.isclose(x, 0.9, abs_tol=1e-9)
    assert math.isclose(y, 1.2, abs_tol=1e-9)


def test_increase_only_smoother_stops_immediately():
    smoother = CommandSmoother(forward_slew_per_s=1.0, turn_slew_per_s=1.0)
    smoother.step(1.0, 1.0, 10.0)
    forward, turn = smoother.step(1.0, 1.0, 10.1)
    assert math.isclose(forward, 0.1)
    assert math.isclose(turn, 0.1)
    assert smoother.step(0.0, 0.0, 10.2) == (0.0, 0.0)


def test_static_circle_changes_planned_path():
    config = example()
    obstacle = (2.7432, 2.7432, 0.6, {"type": "virtual", "label": "center"})
    nav = Navigator(peer_supplier=lambda: (obstacle,))
    nav.set_target((3.65, 3.65), source="manual")
    path = nav.plan_path((1.8, 1.8))
    assert path
    assert path[-1] == (3.65, 3.65)
    assert len(path) > 1


def test_hard_hold_has_hysteresis():
    close = ((0.5, 0.0, 1.5),)
    assert peer_hold_active((0.0, 0.0), close, currently_holding=False)
    still_near = ((1.0, 0.0, 1.5),)
    assert peer_hold_active((0.0, 0.0), still_near, currently_holding=True)
    clear = ((1.3, 0.0, 1.5),)
    assert not peer_hold_active((0.0, 0.0), clear, currently_holding=True)


def test_patrol_holds_on_unhealthy_obstacle_feed_and_dwells_at_waypoint():
    config = example()
    robot = config.robots[0]
    obstacles = []
    patrol = PatrolController(robot, config, lambda: tuple(obstacles))
    held = patrol.step(1.0, robot.waypoints[0], 0.0, obstacle_feed_healthy=False)
    assert (held.forward_mps, held.turn_rad_s, held.phase) == (0.0, 0.0, "HOLD")
    patrol.start()
    patrol.phase = "WALK"
    arrived = patrol.step(2.0, robot.waypoints[0], 0.0, obstacle_feed_healthy=True)
    assert arrived.phase == "DWELL"
    assert arrived.forward_mps == 0.0


def test_uwb_tracker_rejects_nonfinite_positions_before_trust():
    tracker = UwbTracker("unused", 115200, [(0, 0), (0, 1), (1, 0)], use_kf=False)
    tracker._update(1, (float("nan"), 1.0))
    tracker._update(2, (1.0, float("inf")))
    assert tracker.get(1) == (None, [], 0.0)
    assert tracker.get(2) == (None, [], 0.0)
