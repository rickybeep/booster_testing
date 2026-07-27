import math
import unittest
from pathlib import Path

import arena_patrol as patrol


HERE = Path(__file__).resolve().parent


class ArenaPatrolTests(unittest.TestCase):
    def setUp(self):
        self.cfg = patrol.ArenaConfig.load(HERE / "arena_config.json")

    @staticmethod
    def nav(now, yaw_deg=0.0):
        return patrol.NavSample(
            yaw_rad=math.radians(yaw_deg),
            source_ts=now + 1.0,
            progressed_at=now,
            rate_hz=20.0,
        )

    @staticmethod
    def status(now, mode="walk", move_ts=1.0):
        return patrol.PanelStatus(
            mode=mode,
            last_rc=0,
            last_move_ts=move_ts,
            last_move_error="",
            received_at=now,
        )

    def fix(self, now, position, *, anchors=4, residual=0.01):
        return patrol.UwbFix(position, now, anchors, residual)

    def test_shared_config_waypoints_are_inside_safe_interior(self):
        sx0, sy0, sx1, sy1 = self.cfg.safe_rect
        self.assertEqual(len(self.cfg.waypoints), 4)
        for x, y in self.cfg.waypoints:
            self.assertLessEqual(sx0, x)
            self.assertLessEqual(x, sx1)
            self.assertLessEqual(sy0, y)
            self.assertLessEqual(y, sy1)

    def test_device_solved_uwb_line(self):
        fix = patrol.parse_uwb_line(
            "t1:7 status LO=[1.250,2.500,0.700]", self.cfg, 9.0
        )
        self.assertIsNotNone(fix)
        assert fix is not None
        self.assertEqual(fix.position, (1.25, 2.5))
        self.assertEqual(fix.anchors_used, 0)
        self.assertEqual(fix.received_at, 9.0)

    def test_mc_ranges_trilaterate_back_to_known_point(self):
        target = (2.0, 3.0)
        slant_mm = []
        for anchor in self.cfg.anchors:
            horizontal = patrol.point_distance(target, anchor)
            slant = math.hypot(
                horizontal, self.cfg.anchor_to_tag_vertical_offset_m
            )
            slant_mm.append(int(round(slant * 1000.0)))
        fields = " ".join(f"{distance:08x}" for distance in slant_mm)
        fix = patrol.parse_uwb_line(f"mc 00000000 {fields}", self.cfg, 2.0)
        self.assertIsNotNone(fix)
        assert fix is not None
        self.assertAlmostEqual(fix.position[0], target[0], places=2)
        self.assertAlmostEqual(fix.position[1], target[1], places=2)
        self.assertEqual(fix.anchors_used, 4)
        self.assertLess(fix.residual_m, 0.01)

    def test_heading_calibration_uses_measured_travel_bearing(self):
        controller = patrol.ArenaPatrolController(
            self.cfg,
            heading_offset_deg=None,
            allow_calibration_motion=True,
        )
        start = controller.step(
            0.0,
            self.fix(0.0, (2.0, 2.0)),
            self.nav(0.0, yaw_deg=-30.0),
            self.status(0.0),
        )
        self.assertEqual(start.state, "CALIBRATE_FORWARD")
        self.assertGreater(start.motion.forward_m_s, 0.0)
        finish = controller.step(
            4.0,
            self.fix(4.0, (2.81, 2.0)),
            self.nav(4.0, yaw_deg=-30.0),
            self.status(4.0),
        )
        self.assertEqual(finish.state, "CALIBRATION_SETTLE")
        self.assertEqual(finish.motion, patrol.MotionCommand())
        self.assertAlmostEqual(controller.heading_offset_deg, 30.0, places=6)

    def test_no_heading_means_no_motion_without_explicit_calibration(self):
        controller = patrol.ArenaPatrolController(
            self.cfg,
            heading_offset_deg=None,
            allow_calibration_motion=False,
        )
        output = controller.step(
            0.0,
            self.fix(0.0, (2.0, 2.0)),
            self.nav(0.0),
            self.status(0.0),
        )
        self.assertEqual(output.state, "NEEDS_HEADING")
        self.assertFalse(output.motion.moving)

    def test_robot_must_already_report_walk(self):
        controller = patrol.ArenaPatrolController(
            self.cfg,
            heading_offset_deg=0.0,
            allow_calibration_motion=False,
        )
        output = controller.step(
            0.0,
            self.fix(0.0, (2.0, 2.0)),
            self.nav(0.0),
            self.status(0.0, mode="prep"),
        )
        self.assertEqual(output.state, "WAIT_INPUT")
        self.assertIn("WALK", output.reason)
        self.assertFalse(output.motion.moving)

    def test_stale_uwb_latches_stop_after_motion_started(self):
        controller = patrol.ArenaPatrolController(
            self.cfg,
            heading_offset_deg=0.0,
            allow_calibration_motion=False,
        )
        raw_tag = (2.20 - self.cfg.tag_forward_offset_m, 2.20)
        moving = controller.step(
            0.0,
            self.fix(0.0, raw_tag),
            self.nav(0.0),
            self.status(0.0),
        )
        self.assertTrue(moving.motion.moving)
        stopped = controller.step(
            1.0,
            self.fix(0.0, raw_tag),
            self.nav(0.0),
            self.status(1.0),
        )
        self.assertEqual(stopped.state, "FAULT_STOP")
        self.assertIn("stale", stopped.reason)
        self.assertFalse(stopped.motion.moving)

    def test_perimeter_stop_band_latches_zero(self):
        controller = patrol.ArenaPatrolController(
            self.cfg,
            heading_offset_deg=0.0,
            allow_calibration_motion=False,
        )
        output = controller.step(
            0.0,
            self.fix(0.0, (0.10, 2.70)),
            self.nav(0.0),
            self.status(0.0),
        )
        self.assertEqual(output.state, "FAULT_STOP")
        self.assertFalse(output.motion.moving)

    def test_warning_band_recovers_slowly_toward_center(self):
        controller = patrol.ArenaPatrolController(
            self.cfg,
            heading_offset_deg=0.0,
            allow_calibration_motion=False,
        )
        raw_tag = (1.0 - self.cfg.tag_forward_offset_m, 2.0)
        output = controller.step(
            0.0,
            self.fix(0.0, raw_tag),
            self.nav(0.0),
            self.status(0.0),
        )
        self.assertEqual(output.state, "RECOVER_CENTER")
        self.assertEqual(output.target, self.cfg.center)
        self.assertLessEqual(
            output.motion.forward_m_s, self.cfg.recovery_forward_m_s
        )

    def test_waypoint_arrival_stops_and_advances_head_sequence(self):
        controller = patrol.ArenaPatrolController(
            self.cfg,
            heading_offset_deg=0.0,
            allow_calibration_motion=False,
        )
        waypoint = self.cfg.waypoints[0]
        raw_tag = (waypoint[0] - self.cfg.tag_forward_offset_m, waypoint[1])
        arrived = controller.step(
            0.0,
            self.fix(0.0, raw_tag),
            self.nav(0.0),
            self.status(0.0),
        )
        self.assertEqual(arrived.state, "DWELL_LOOK")
        self.assertFalse(arrived.motion.moving)
        self.assertIsNotNone(arrived.head)
        second_pose = controller.step(
            1.30,
            self.fix(1.30, raw_tag),
            self.nav(1.30),
            self.status(1.30),
        )
        self.assertIsNotNone(second_pose.head)
        complete = controller.step(
            4.60,
            self.fix(4.60, raw_tag),
            self.nav(4.60),
            self.status(4.60),
        )
        self.assertEqual(complete.state, "TRAVEL")
        self.assertEqual(complete.target, self.cfg.waypoints[1])
        self.assertFalse(complete.motion.moving)
        self.assertEqual(complete.head, patrol.HeadCommand(0.0, 0.0, 0.5))

    def test_sdk_yaw_sign_is_applied_only_at_bridge_boundary(self):
        bridge = object.__new__(patrol.HttpPanelBridge)
        bridge.cfg = self.cfg
        payload = bridge.motion_payload(patrol.MotionCommand(0.1, 0.2))
        self.assertEqual(payload, {"vx": 0.1, "vy": 0.0, "vyaw": 0.2})
        flipped = patrol.ArenaConfig(
            **{**self.cfg.__dict__, "sdk_yaw_sign": -1}
        )
        bridge.cfg = flipped
        payload = bridge.motion_payload(patrol.MotionCommand(0.1, 0.2))
        self.assertEqual(payload["vyaw"], -0.2)

    def test_observe_bridge_never_posts_motion_or_head(self):
        bridge = object.__new__(patrol.HttpPanelBridge)
        bridge.cfg = self.cfg
        bridge.commands_enabled = False
        bridge.last_motion_payload = None
        bridge.last_head_payload = None
        bridge.send_motion(patrol.MotionCommand(0.1, 0.2))
        bridge.send_head(patrol.HeadCommand(0.0, 0.2))
        self.assertEqual(
            bridge.last_motion_payload, {"vx": 0.1, "vy": 0.0, "vyaw": 0.2}
        )
        self.assertEqual(bridge.last_head_payload["arc_dir"], 1)

    def test_delivery_gate_rejects_queue_ack_without_sdk_progress(self):
        gate = patrol.MotionDeliveryGate(timeout_s=1.25)
        moving = patrol.MotionCommand(0.1, 0.0)
        self.assertIsNone(gate.check(0.0, moving, self.status(0.0, move_ts=10.0)))
        error = gate.check(1.30, moving, self.status(1.30, move_ts=10.0))
        self.assertIn("no robot-side successful Move", error)

    def test_delivery_gate_accepts_and_then_requires_continued_progress(self):
        gate = patrol.MotionDeliveryGate(timeout_s=1.25)
        moving = patrol.MotionCommand(0.1, 0.0)
        self.assertIsNone(gate.check(0.0, moving, self.status(0.0, move_ts=10.0)))
        self.assertIsNone(gate.check(0.5, moving, self.status(0.5, move_ts=11.0)))
        self.assertIsNone(gate.check(1.0, moving, self.status(1.0, move_ts=12.0)))
        error = gate.check(2.30, moving, self.status(2.30, move_ts=12.0))
        self.assertIn("stopped advancing", error)

    def test_offline_simulation_calibrates_and_reaches_waypoints(self):
        self.assertEqual(patrol.run_simulation(self.cfg, 70.0, quiet=True), 0)

    def test_execute_mode_refuses_without_exact_confirmation(self):
        with self.assertRaises(SystemExit) as context:
            patrol.main(
                [
                    "--mode",
                    "execute",
                    "--robot-url",
                    "http://example.invalid:8080",
                    "--uwb-port",
                    "TEST_PORT",
                    "--heading-offset-deg",
                    "0",
                ]
            )
        self.assertIn("execute refused", str(context.exception))


if __name__ == "__main__":
    unittest.main()
