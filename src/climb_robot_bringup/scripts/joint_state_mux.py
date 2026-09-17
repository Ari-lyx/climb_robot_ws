#!/usr/bin/env python3
"""Forward disjoint joint sources without changing measurement timestamps.

JointState may contain a subset of joints. robot_state_publisher and MoveIt
retain the other joints themselves; merging asynchronously sampled wheels and
arm under a newly generated timestamp would misrepresent their acquisition time.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMux(Node):
    def __init__(self):
        super().__init__('joint_state_mux')
        self.last_stamp = {}
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(JointState, '/wheel_joint_states',
                                 lambda msg: self.update('wheel', msg), 10)
        self.create_subscription(JointState, '/arm_joint_state_broadcaster/joint_states',
                                 lambda msg: self.update('arm', msg), 10)

    def update(self, source, msg):
        size = len(msg.name)
        if not size or len(msg.position) != size:
            return
        if any(len(values) not in (0, size) for values in (msg.velocity, msg.effort)):
            return
        if len(set(msg.name)) != size:
            return
        wheels = {'front_left_joint', 'front_right_joint',
                  'rear_left_joint', 'rear_right_joint'}
        if any((not name.startswith('arm_')) if source == 'arm'
               else name not in wheels for name in msg.name):
            self.get_logger().error('Rejected conflicting joint source: ' + source)
            return
        # The arm broadcaster can run at the 1 kHz control rate. Bound the
        # visualization/state-monitor feed to 50 Hz per source without restamping.
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        previous = self.last_stamp.get(source)
        if previous is not None and 0 <= stamp - previous < 20_000_000:
            return
        self.last_stamp[source] = stamp
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JointStateMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
