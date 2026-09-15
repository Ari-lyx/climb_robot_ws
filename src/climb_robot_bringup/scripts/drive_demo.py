#!/usr/bin/env python3
"""Publish a bounded-duration forward command; Ctrl-C also sends a stop."""
import argparse
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--speed',type=float,default=.2)
    parser.add_argument('--seconds',type=float,default=10)
    args=parser.parse_args()
    rclpy.init();node=Node('climb_drive_demo');pub=node.create_publisher(Twist,'/cmd_vel',10)
    start=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-start<args.seconds:
            command=Twist();command.linear.x=args.speed;pub.publish(command)
            rclpy.spin_once(node,timeout_sec=.05)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
