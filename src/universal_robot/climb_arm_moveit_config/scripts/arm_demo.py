#!/usr/bin/env python3
"""Use MoveIt's MoveGroup action to plan AND execute a small joint-space scan."""
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints,JointConstraint
JOINTS=['arm_'+j for j in ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']]
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--pan-offset',type=float,default=.25);args=parser.parse_args()
    rclpy.init();node=Node('arm_moveit_demo');positions={}
    node.create_subscription(JointState,'/joint_states',lambda m:positions.update(zip(m.name,m.position)),10)
    import time
    deadline=time.monotonic()+30
    while not all(j in positions for j in JOINTS) and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    if not all(j in positions for j in JOINTS):raise RuntimeError('Arm joint states unavailable')
    client=ActionClient(node,MoveGroup,'/move_action')
    if not client.wait_for_server(timeout_sec=30):raise RuntimeError('MoveIt not available; launch with moveit:=true')
    goal=MoveGroup.Goal();goal.request.group_name='arm';goal.request.allowed_planning_time=10.0;goal.request.num_planning_attempts=5
    goal.request.max_velocity_scaling_factor=.2;goal.request.max_acceleration_scaling_factor=.2
    goal.request.start_state.is_diff=True
    constraint=Constraints()
    for j in JOINTS:
        c=JointConstraint();c.joint_name=j;c.position=positions[j]+(args.pan_offset if j==JOINTS[0] else 0.0);c.tolerance_above=.01;c.tolerance_below=.01;c.weight=1.0;constraint.joint_constraints.append(c)
    goal.request.goal_constraints=[constraint];goal.planning_options.plan_only=False;goal.planning_options.planning_scene_diff.is_diff=True;goal.planning_options.planning_scene_diff.robot_state.is_diff=True
    future=client.send_goal_async(goal);rclpy.spin_until_future_complete(node,future,timeout_sec=30)
    handle=future.result()
    if not handle or not handle.accepted:raise RuntimeError('MoveIt rejected goal')
    result=handle.get_result_async();rclpy.spin_until_future_complete(node,result,timeout_sec=90)
    if not result.done():handle.cancel_goal_async();raise RuntimeError('MoveIt execution timeout')
    code=result.result().result.error_code.val;print('MoveIt plan/execute result:',code)
    node.destroy_node();rclpy.shutdown()
    if code!=1:raise SystemExit(1)
if __name__=='__main__':main()
