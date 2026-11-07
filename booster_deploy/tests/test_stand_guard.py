"""Native guard tests, independent of a robot or ROS runtime."""
import math
import json
from pathlib import Path
import pytest
core = pytest.importorskip("booster_policy_core")


def update(g, *, sit=False, terminal=True, upright=1., gyro=0., speed=0., error=0.):
    return g.update(sit, terminal, .02, upright, gyro, speed, error)


def test_push_up_tilt_does_not_trigger_terminal_guard():
    g = core.StandGuard()
    for _ in range(250):
        assert not update(g, terminal=False, upright=.05, gyro=2., speed=4., error=2.)
    assert not g.fault


def test_cat_terminal_tilt_latches_failure_not_walk():
    g = core.StandGuard()
    projection = math.cos(math.radians(80.0279))*math.cos(math.radians(66.5701))
    assert not update(g, upright=projection)
    assert g.fault
    for _ in range(100):
        assert not update(g)
    g.reset()
    assert not g.fault


def test_terminal_pose_hands_off_immediately_for_walk_balancing():
    g = core.StandGuard()
    assert update(g)
    assert g.stable_s == pytest.approx(.02)
    assert not g.fault


@pytest.mark.parametrize("moving", [{"gyro": 1.}, {"speed": 2.}, {"error": .8}, {"upright": .9}])
def test_terminal_motion_does_not_block_balancing(moving):
    g = core.StandGuard()
    assert update(g, **moving)
    assert not g.fault


def test_stand_request_reversal_cancels_completion():
    g = core.StandGuard()
    assert update(g)
    assert not update(g, sit=True)
    assert g.stable_s == 0


def test_no_elapsed_settle_fault_on_upright_terminal_pose():
    g = core.StandGuard()
    for _ in range(110):
        assert update(g, error=.8, speed=2.)
    assert not g.fault


@pytest.mark.parametrize("field", ["upright", "gyro", "speed", "error"])
def test_invalid_measurement_never_completes(field):
    g = core.StandGuard()
    assert not update(g, **{field: float("nan")})
    assert g.fault


def test_stale_frames_wait_for_fresh_state_without_settle_timeout():
    g = core.StandGuard()
    assert not g.update(False, True, .02, 1., 0., 0., 0., False)
    assert g.stable_s == 0
    for _ in range(110):
        assert not g.update(False, True, .02, 1., 0., 0., 0., False)
    assert not g.fault
    assert update(g)


@pytest.mark.parametrize('robot', ['cat', 'earl', 'tear'])
def test_recorded_terminal_pose_handoff(robot):
    data = json.loads((Path(__file__).parent/'fixtures/stand_terminal_nov05.json').read_text())
    case = next(c for c in data['cases'] if c['robot'] == robot)
    g = core.StandGuard()
    first_terminal = next(f for f in case['frames'] if f['terminal'])
    admitted = None
    for f in case['frames']:
        ok = update(g, sit=f['sit'], terminal=f['terminal'], upright=f['upright'],
                    gyro=f['gyro'], speed=f['speed'], error=f['error'])
        if ok:
            admitted = f
            break  # WALK takes over on the next tick; do not simulate SIT beyond that.
    if case['expected'] == 'handoff':
        assert admitted == first_terminal
        assert not g.fault
    else:
        assert admitted is None
        assert 'measured upright' in g.fault
