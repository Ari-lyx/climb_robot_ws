#!/usr/bin/env python3
"""Publish a bounded-duration forward command; Ctrl-C also sends a stop."""
# 发送一段限时的前进速度指令；Ctrl-C 时也会补发一次零速。
# 说明：本脚本只是“测试用命令源”，不是驱动本体——真正的四轮扭矩控制、
# 速度环、磁吸力都在 robot_control_driver 的 Gazebo 插件里。
import argparse
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def main():
    # --speed 前进速度 [m/s]；--seconds 持续时间 [s]（用真实时间计，不是仿真时间）
    parser=argparse.ArgumentParser()
    parser.add_argument('--speed',type=float,default=.2)
    parser.add_argument('--seconds',type=float,default=10)
    args=parser.parse_args()
    rclpy.init();node=Node('climb_drive_demo');pub=node.create_publisher(Twist,'/cmd_vel',10)
    start=time.monotonic()
    try:
        # 20 Hz 左右持续发布：插件侧的 command_timeout(0.5s) 依赖“持续收到消息”才算命令有效
        while rclpy.ok() and time.monotonic()-start<args.seconds:
            command=Twist();command.linear.x=args.speed;pub.publish(command)
            rclpy.spin_once(node,timeout_sec=.05)
    except KeyboardInterrupt:pass
    finally:
        # 退出前发一条零速，让驱动进入零速制动目标（而不是等待超时）
        if rclpy.ok():pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
