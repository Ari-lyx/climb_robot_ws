#!/usr/bin/env python3
"""Isolated simulation regression. Source install/setup.bash before running.

PROBE_CONFIG/PROBE_RVIZ override test inputs, PROBE_SOAK controls wall seconds.
Results and launch output go to PROBE_OUT (default /tmp/climb_stability_check).
This is a controller regression in simulation, not a collision-checked task.
"""
import os,time,subprocess,signal,json
from pathlib import Path
os.environ['ROS_DOMAIN_ID'] = '191'
os.environ['GAZEBO_MASTER_URI'] = 'http://127.0.0.1:11461'
os.environ['ROS_LOG_DIR'] = os.environ.get('PROBE_OUT', '/tmp/climb_stability_check') + '/ros_logs'
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState,PointCloud2
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.duration import Duration
from collections import deque
from rclpy.qos import qos_profile_sensor_data
# Refuse to attach to an already running test server.
import socket
with socket.socket() as port_check:
    port_check.bind(('127.0.0.1', 11461))
out=Path(os.environ.get('PROBE_OUT','/tmp/climb_stability_check'));out.mkdir(exist_ok=True)
log=open(out/'launch.log','w');proc=subprocess.Popen(['ros2','launch','climb_robot_bringup','simulation.launch.py','gui:=false','world:=hemisphere'] + ([f"config:={os.environ['PROBE_CONFIG']}"] if 'PROBE_CONFIG' in os.environ else []) + ([f"rviz_config:={os.environ['PROBE_RVIZ']}"] if 'PROBE_RVIZ' in os.environ else []),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();n=Node('stability_probe',parameter_overrides=[Parameter('use_sim_time',value=True)]);q={}; samples=deque(maxlen=2000);cloud=[0]; results=[]; resources=[]; pending=deque(maxlen=100); tf_stats={"ok":0,"expired":0}; last_resource=0.0; moveit_result=None; raw_samples=deque(maxlen=200)
n.create_subscription(JointState,'/arm_joint_state_broadcaster/joint_states',lambda m:q.update(zip(m.name,m.position)),10)
n.create_subscription(JointState,'/arm/raw_joint_states',lambda m:raw_samples.append(list(m.velocity)),qos_profile_sensor_data)
def state(m):
 samples.append({'t':time.time(),'sim_time':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'q':list(m.feedback.positions),'v':list(m.feedback.velocities),'e':list(m.error.positions)})
n.create_subscription(JointTrajectoryControllerState,'/arm_controller/controller_state',state,10)
buffer=Buffer(cache_time=Duration(seconds=10)); listener=TransformListener(buffer,n)
def on_cloud(msg):
 cloud[0]+=1
 pending.append((time.monotonic(),msg.header))
n.create_subscription(PointCloud2,'/d435i/depth/color/points',on_cloud,qos_profile_sensor_data)
def health():
 global last_resource
 now=time.monotonic()
 while pending:
  arrived,header=pending[0]
  if buffer.can_transform('world',header.frame_id,Time.from_msg(header.stamp)):
   tf_stats['ok']+=1;pending.popleft()
  elif now-arrived>1.:
   tf_stats['expired']+=1;pending.popleft()
  else:break
 if now-last_resource>5:
  last_resource=now
  import re
  pids=re.findall(r'process started with pid \[([0-9]+)\]',(out/'launch.log').read_text())
  for pid in pids:
   try:
    status=Path('/proc',pid,'status').read_text()
    name=re.search(r'^Name:\s+(.*)',status,re.M).group(1)
    if name in ('rviz2','gzserver','move_group'):
     rss=int(re.search(r'^VmRSS:\s+(\d+)',status,re.M).group(1))
     resources.append({'wall':time.time(),'pid':int(pid),'name':name,'rss_kib':rss})
   except (FileNotFoundError,AttributeError):pass

def spin(sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.03)
  health()
  if proc.poll() is not None:raise RuntimeError('launch exited')
try:
 deadline=time.monotonic()+100
 while len(q)<6 and time.monotonic()<deadline:spin(.1)
 spin(12)
 client=ActionClient(n,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory');assert client.wait_for_server(timeout_sec=10)
 names=['arm_'+s+'_joint' for s in ['shoulder_pan','shoulder_lift','elbow','wrist_1','wrist_2','wrist_3']]
 origin=[q[j] for j in names]
 for offset in [[.25,0,0,0,0,0],[0,-.2,.15,.2,0,0],[0,0,0,0,0,0]]:
  goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=names
  p=JointTrajectoryPoint();p.positions=[a+b for a,b in zip(origin,offset)];p.velocities=[0.]*6;p.time_from_start.sec=8;goal.trajectory.points=[p]
  f=client.send_goal_async(goal);rclpy.spin_until_future_complete(n,f,timeout_sec=10);h=f.result();assert h and h.accepted
  f=h.get_result_async();deadline=time.monotonic()+120
  while not f.done() and time.monotonic()<deadline:spin(.05)
  results.append({'offset':offset,'code':f.result().result.error_code if f.done() else None,'result':str(f.result().result) if f.done() else 'timeout'})
  spin(4)
 spin(float(os.environ.get('PROBE_SOAK','60')))
 if os.environ.get('PROBE_MOVEIT') == '1':
  with open(out/'moveit-demo.log','w') as demo_log:
   demo=subprocess.Popen(['ros2','run','climb_arm_moveit_config','arm_demo.py'],stdout=demo_log,stderr=subprocess.STDOUT,start_new_session=True)
   try:
    deadline=time.monotonic()+120
    while demo.poll() is None and time.monotonic()<deadline:spin(.05)
    moveit_result=demo.poll()
   finally:
    if demo.poll() is None:
     os.killpg(demo.pid,signal.SIGTERM)
     try:demo.wait(timeout=5)
     except subprocess.TimeoutExpired:os.killpg(demo.pid,signal.SIGKILL);demo.wait()
  spin(3)
except Exception as e:results.append({'exception':str(e)})
finally:
 (out/'result.json').write_text(json.dumps({'goals':results,'clouds':cloud[0],'samples':list(samples),'tf':tf_stats,'resources':resources,'raw_velocities':list(raw_samples),'moveit_exit_code':moveit_result},indent=2))
 print(json.dumps({'goals':results,'clouds':cloud[0],'samples':len(samples)}),flush=True)
 n.destroy_node();rclpy.shutdown()
 if proc.poll() is None:proc.send_signal(signal.SIGINT)
 try:proc.wait(timeout=15)
 except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 log.close()
raise SystemExit(0 if len(results)==3 and all(r.get("code")==0 for r in results) and cloud[0]>0 and tf_stats["expired"]==0 and (os.environ.get("PROBE_MOVEIT") != "1" or moveit_result == 0) else 1)
