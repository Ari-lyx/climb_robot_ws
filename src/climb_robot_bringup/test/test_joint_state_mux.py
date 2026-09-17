"""Regression: asynchronous joint samples must keep their acquisition time."""
import runpy
import unittest
from pathlib import Path
from unittest.mock import Mock
from sensor_msgs.msg import JointState

Mux = runpy.run_path(str(Path(__file__).resolve().parents[2] /
                        'climb_robot_bringup/scripts/joint_state_mux.py'))['JointStateMux']


class JointStateForwarding(unittest.TestCase):
    def setUp(self):
        self.node = object.__new__(Mux)
        self.node.pub = Mock()
        self.node.last_stamp = {}
        self.node._logger = Mock()

    def sample(self, name):
        msg = JointState()
        msg.header.stamp.sec = 12
        msg.header.stamp.nanosec = 345678
        msg.name = [name]
        msg.position = [0.2]
        msg.velocity = [-0.1]
        return msg

    def test_arm_timestamp_velocity_and_effort_preserved(self):
        msg = self.sample('arm_shoulder_pan_joint')
        msg.effort = [1.3]
        self.node.update('arm', msg)
        forwarded = self.node.pub.publish.call_args.args[0]
        self.assertEqual(forwarded, msg)
        self.assertEqual(forwarded.header.stamp.nanosec, 345678)
        self.assertEqual(list(forwarded.velocity), [-0.1])

    def test_wheel_sample_does_not_republish_cached_arm(self):
        arm = self.sample('arm_shoulder_pan_joint')
        wheel = self.sample('front_left_joint')
        wheel.header.stamp.sec = 13
        self.node.update('arm', arm)
        self.node.update('wheel', wheel)
        self.assertEqual(self.node.pub.publish.call_count, 2)
        self.assertEqual(self.node.pub.publish.call_args.args[0], wheel)
        self.assertEqual(arm.header.stamp.sec, 12)

    def test_high_rate_source_is_bounded_and_clock_reset_recovers(self):
        for i in range(100):
            msg = self.sample('arm_shoulder_pan_joint')
            msg.header.stamp.nanosec = i * 1_000_000
            self.node.update('arm', msg)
        self.assertEqual(self.node.pub.publish.call_count, 5)
        msg.header.stamp.sec = 0
        self.node.update('arm', msg)
        self.assertEqual(self.node.pub.publish.call_count, 6)

    def test_conflicting_and_malformed_sources_rejected(self):
        self.node.update('wheel', self.sample('arm_shoulder_pan_joint'))
        self.node.update('arm', self.sample('front_left_joint'))
        self.node.update('wheel', self.sample('unrelated_joint'))
        msg = self.sample('arm_shoulder_pan_joint')
        msg.velocity = [0., 0.]
        self.node.update('arm', msg)
        self.node.pub.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
