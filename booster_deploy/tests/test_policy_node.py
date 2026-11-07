"""End-to-end test of the C++ policy node driven through PolicyClient."""
from __future__ import annotations

import math
import os
import random
import threading
import time
import unittest

import pytest

pytest.importorskip("booster_policy_core")
rclpy = pytest.importorskip("rclpy")

from booster_interface.msg import LowCmd, LowState, MotorState  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402

from booster_deploy.policy_client import PolicyClient, PolicyStatus  # noqa: E402
from booster_deploy.policy_node import PolicyNodeProcess  # noqa: E402
from tasks.walk.walk import K1WalkControllerCfg  # noqa: E402


def wait_for(predicate, timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


class PolicyNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        # Isolate from other ROS graphs on the machine.
        os.environ["ROS_DOMAIN_ID"] = str(random.randint(100, 200))
        os.environ["ROS_LOCALHOST_ONLY"] = "1"

    def setUp(self) -> None:
        cfg = K1WalkControllerCfg()
        cfg.policy.command_timeout = 0.3
        self.default_pos = list(cfg.robot.default_joint_pos)
        self.roll = 0.0
        self.gyro = [0.0, 0.0, 0.0]
        self.leg_speed = 0.0
        self.low_cmds: list[LowCmd] = []

        self.node_process = PolicyNodeProcess(cfg)
        self.node_process.start()
        self.addCleanup(self.node_process.stop)

        rclpy.init()
        self.addCleanup(rclpy.try_shutdown)
        self.node = rclpy.create_node("policy_node_test")
        self.low_state_pub = self.node.create_publisher(
            LowState, "/low_state", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.low_cmd_sub = self.node.create_subscription(LowCmd, "/joint_ctrl", self.low_cmds.append, 10)
        self.node.create_timer(0.002, self.publish_low_state)
        self.client = PolicyClient(self.node, heartbeat_period=0.02)

        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        stop = threading.Event()
        thread = threading.Thread(
            target=lambda: [executor.spin_once(timeout_sec=0.05) for _ in iter(stop.is_set, True)],
            daemon=True,
        )
        thread.start()

        def shutdown() -> None:
            self.client.close()
            stop.set()
            thread.join(timeout=2.0)
            executor.shutdown()
            self.node.destroy_node()

        self.addCleanup(shutdown)

    def publish_low_state(self) -> None:
        msg = LowState()
        msg.imu_state.rpy = [self.roll, 0.0, 0.0]
        msg.imu_state.gyro = self.gyro
        msg.motor_state_serial = [MotorState(q=float(q), dq=self.leg_speed if i >= 10 else 0.0)
                                  for i, q in enumerate(self.default_pos)]
        self.low_state_pub.publish(msg)

    def test_diagnostic_listener_does_not_make_actuation_ready(self) -> None:
        from booster_policy.msg import PolicyTrace
        self.node.destroy_subscription(self.low_cmd_sub)
        traces = []
        self.node.create_subscription(
            PolicyTrace, "/booster_policy/trace", traces.append,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.assertTrue(wait_for(self.client.request_start, timeout=10.0))
        self.assertTrue(wait_for(lambda: len(traces) > 10, timeout=10.0))
        self.assertFalse(self.client.is_ready())
        trace = traces[-1]
        self.assertTrue(trace.command_published)
        self.assertGreater(trace.low_state_sequence, 0)
        self.assertEqual(len(trace.command.motor_cmd), 22)
        self.assertEqual(len(trace.measured.motor_state_serial), 22)
        self.assertAlmostEqual(trace.measured.motor_state_serial[0].q, self.default_pos[0])
        self.roll = 1.5
        self.assertTrue(wait_for(lambda: any(t.fault for t in traces)))
        failed = next(t for t in traces if t.fault)
        self.assertFalse(failed.command_published)
        self.assertTrue(failed.output_valid)  # inspectable output was withheld
        self.assertFalse(failed.standing_pose_complete)
        self.assertGreater(failed.source_steady_ns, 0)
        self.assertGreaterEqual(failed.low_state_age_s, 0)
        self.assertIn("orientation", failed.fault)
        self.assertAlmostEqual(failed.measured.imu_state.rpy[0], 1.5)

    def test_start_walk_squat_sit_stand_and_stop(self) -> None:
        self.client.start(timeout=10.0)
        self.assertTrue(self.client.is_ready())
        self.assertTrue(wait_for(lambda: len(self.low_cmds) > 5))
        cmd = self.low_cmds[-1]
        self.assertEqual(cmd.cmd_type, LowCmd.CMD_TYPE_SERIAL)
        self.assertEqual(len(cmd.motor_cmd), 22)
        self.assertEqual(cmd.motor_cmd[0].kp, 10.0)
        self.assertTrue(all(math.isfinite(motor.q) for motor in cmd.motor_cmd))

        self.client.set_head_target(0.3, -0.1)
        self.assertTrue(wait_for(lambda: abs(self.low_cmds[-1].motor_cmd[0].q - 0.3) < 1e-6))
        self.assertFalse(self.client.status.command_timed_out)

        self.client.squat()
        self.assertTrue(wait_for(lambda: self.client.squat_active and self.client.squat_started))
        self.assertTrue(self.client.status.squat_commanded)
        self.client.stand()
        self.assertTrue(wait_for(lambda: self.client.standing_pose_complete))
        self.assertTrue(wait_for(lambda: not self.client.squat_active))

        self.client.sit()
        self.assertTrue(wait_for(lambda: self.client.sit_active and self.client.squat_started))
        # Simulate the measured extended leg pose, not the WALK crouch.
        # Similar to CAT's recorded good terminal pose; no physical simulation.
        self.default_pos[10:] = [-.05, .13, .23, .22, -.09, -.09,
                                 -.07, -.14, -.14, .25, -.14, -.07]
        self.client.stand()
        self.assertTrue(wait_for(lambda: self.client.standing_pose_complete, timeout=30.0))
        self.assertTrue(wait_for(lambda: not self.client.sit_active))

        self.assertTrue(self.client.stop())
        count = len(self.low_cmds)
        time.sleep(0.2)
        self.assertLessEqual(len(self.low_cmds) - count, 1)
        self.assertIsNone(self.client.status)

        # A new start opens a fresh session.
        self.client.start(timeout=5.0)
        self.assertTrue(self.client.is_ready())

    def test_missing_commands_zero_velocity(self) -> None:
        self.client.start(timeout=10.0)
        self.node.destroy_timer(self.client._heartbeat)
        self.assertTrue(wait_for(lambda: self.client.status.command_timed_out, timeout=2.0))
        self.assertTrue(self.client.is_running())

    def test_moving_terminal_stand_keeps_publishing_and_resumes_walk(self) -> None:
        # Synthetic state, isolated ROS domain: reproduces the former EARL/TEAR
        # settle rejection without touching a real actuator or its ROS graph.
        from booster_policy.msg import PolicyTrace
        traces = []
        self.node.create_subscription(PolicyTrace, "/booster_policy/trace", traces.append,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.client.start(timeout=10.0)
        self.client.sit()
        self.assertTrue(wait_for(lambda: self.client.sit_active and self.client.squat_started))
        self.roll = math.radians(4)
        self.gyro = [0.0, 0.0, 1.0]
        self.leg_speed = 1.32
        self.client.stand()
        self.assertTrue(wait_for(lambda: self.client.standing_pose_complete, timeout=30.0))
        self.assertTrue(wait_for(lambda: not self.client.sit_active))
        count = len(self.low_cmds)
        time.sleep(2.5)  # beyond the removed terminal timeout
        self.assertGreater(len(self.low_cmds), count + 50)
        self.assertFalse(self.client.fault)
        self.assertTrue(traces)
        self.assertTrue(all(t.command_published and not t.fault for t in traces))

    def test_tilt_faults_and_stops_publishing(self) -> None:
        self.client.start(timeout=10.0)
        self.roll = 1.2
        self.assertTrue(wait_for(lambda: self.client.fault is not None))
        self.assertEqual(self.client.status.state, PolicyStatus.STATE_FAULT)
        self.assertTrue(self.client.status.posture_fault)
        self.assertGreater(self.client.status.instance_id, 0)
        self.assertLess(abs(time.monotonic()-self.client.status.source_steady_ns/1e9), .5)
        session = self.client.status.session
        # The client treats an existing session as already started. Exercise
        # the server directly to prove a fresh start cannot clear this fault.
        from booster_policy.srv import StartPolicy
        restart = self.client._start_client.call_async(StartPolicy.Request())
        self.assertTrue(wait_for(restart.done))
        self.assertFalse(restart.result().success)
        self.assertIn('latch', restart.result().message)
        self.assertEqual(self.client.status.session, session)
        self.assertEqual(self.client.status.state, PolicyStatus.STATE_FAULT)
        count = len(self.low_cmds)
        time.sleep(0.2)
        self.assertLessEqual(len(self.low_cmds) - count, 1)


if __name__ == "__main__":
    unittest.main()
