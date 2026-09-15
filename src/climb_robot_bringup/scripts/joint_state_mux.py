#!/usr/bin/env python3
"""One /joint_states publisher; disjoint wheel/arm sources, no invented positions."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
class JointStateMux(Node):
    def __init__(self):
        super().__init__('joint_state_mux');self.sources={}
        self.pub=self.create_publisher(JointState,'/joint_states',10)
        self.create_subscription(JointState,'/wheel_joint_states',lambda m:self.update('wheel',m),10)
        self.create_subscription(JointState,'/arm_joint_state_broadcaster/joint_states',lambda m:self.update('arm',m),10)
        self.create_timer(.02,self.publish)
    def update(self,source,msg):
        if len(msg.name)!=len(msg.position):return
        if any(name.startswith('arm_') != (source=='arm') for name in msg.name):
            self.get_logger().error('Rejected conflicting joint source: '+source);return
        self.sources[source]=msg
    def publish(self):
        msg=JointState();msg.header.stamp=self.get_clock().now().to_msg()
        now=self.get_clock().now().nanoseconds
        for source in self.sources.values():
            age=now-source.header.stamp.sec*10**9-source.header.stamp.nanosec
            if age<0 or age>500_000_000:continue
            msg.name.extend(source.name);msg.position.extend(source.position)
            # Gazebo's wheel driver does not publish effort; omit merged optional fields
            # instead of fabricating measured torque for the wheels.
        if msg.name:self.pub.publish(msg)
def main():
    rclpy.init();node=JointStateMux()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
