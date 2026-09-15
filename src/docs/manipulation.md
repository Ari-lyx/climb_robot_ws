# UR3e + D435i：安装、状态与 MoveIt 控制

## 启动与移动

```bash
cd /home/lyx/Downloads/climb_robot_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch climb_robot_bringup simulation.launch.py
```

默认加载真实尺寸的 UR3e、末端 D435i、Gazebo、MoveIt move_group 和 RViz。
在 RViz 的 MotionPlanning 中选择 `arm` 组，拖动末端目标，点击 Plan，再 Execute；
也可以选命名目标 `ready` 或 `inspect`。路径通过碰撞检查后发送给 Gazebo 中的机械臂。

脚本方式同样经过 MoveIt 规划和执行，默认将当前肩部绕竖轴角度增加 0.25 rad：

```bash
ros2 run climb_arm_moveit_config arm_demo.py --pan-offset 0.25
```

机械臂操作期间保持底盘停止。首版是“移动到位后操作机械臂”，没有车臂协同规划。
MoveIt 的规划参考系是 `base_link`；环境节点根据 `/odom` 更新球壳和地面在该参考系中的位置。
球壳与地面对机械臂参与碰撞检查，仅允许四轮与这些表面的正常接触。
已知的视觉焊缝不作为障碍物，RGB/深度相机仍能看到视觉焊缝。

```bash
# 无窗口：深度相机的渲染仍需要图形环境/GPU，gui=false 仅关闭客户端窗口
ros2 launch climb_robot_bringup simulation.launch.py gui:=false rviz:=false
# 调整、调试时可单独关闭组件
ros2 launch climb_robot_bringup simulation.launch.py moveit:=false rviz:=false
ros2 launch climb_robot_bringup simulation.launch.py camera:=false
ros2 launch climb_robot_bringup simulation.launch.py arm:=false moveit:=false rviz:=false
```

## 如何明确调整 joint 相对位置

所有安装参数在 `climb_robot_bringup/config/simulation.yaml` 的 `mounts` 中：

```yaml
mounts:
  arm:    {xyz: [0.04, 0.0, 0.09],  rpy: [0.0, 0.0, 0.0]}
  lidar:  {xyz: [-0.15, 0.0, 0.075], rpy: [0.0, 0.0, 0.0]}
  camera: {xyz: [0.0, 0.0, 0.045], rpy: [0.0, -1.57079632679, 0.0]}
```

| 安装点/关节 | 相对父坐标系 | 修改入口 |
|---|---|---|
| UR3e 基座固定关节 | `base_link` | `mounts.arm.xyz/rpy` |
| VLP-16 安装关节 | `base_link` | `mounts.lidar.xyz/rpy` |
| D435i 底部安装螺孔固定关节 | `arm_tool0` | `mounts.camera.xyz/rpy` |
| 四轮转动关节 | `base_link` | 前后 `x=±wheelbase/2`，左右 `y=±track_width/2`，高度 `z=axle_z` |
| UR3e 内部六轴的固定几何变换 | 各自上一节连杆 | 官方 `ur_description/config/ur3e/default_kinematics.yaml`，可用 `arm.kinematics_file` 指定自己的副本 |
| 六轴初始角度 | 各关节转轴 | `arm.initial_positions`，顺序见下文；这是关节角，不是安装 origin |

`xyz` 单位为米，`rpy` 单位为弧度，旋转采用 URDF 的 roll/pitch/yaw 约定。
`origin` 定义父链接到子关节零位坐标系的固定变换；`axis` 定义转轴；运行时角度 `q` 是另一项变换。
例如把 `mounts.lidar.xyz[0]` 从 -0.15 改为 -0.18，会将雷达向车尾移动 3 cm。
相机本体以 +x 朝前，光学帧以 +z 看向场景；默认安装旋转让相机 +x 对齐 tool0 +z。
修改后重新构建/启动；不要只改安装目录里的副本。

机械臂关节有统一 `arm_` 前缀，顺序为：

1. `arm_shoulder_pan_joint`
2. `arm_shoulder_lift_joint`
3. `arm_elbow_joint`
4. `arm_wrist_1_joint`
5. `arm_wrist_2_joint`
6. `arm_wrist_3_joint`

UR3e 内部几何使用官方参数；修改连杆长度时还必须同步碰撞/视觉/惯量，不能仅修改 origin 后仍宣称是真实 UR3e。
启动时的完整 URDF、初始角度 YAML 保存在 `/tmp/climb_robot_*`，便于逐个检查所有 `<joint><origin .../>`。

## joint_states 会不会和磁轮冲突？

不会：执行控制和状态上报分开，名称和控制接口也分开。

```text
/cmd_vel → magnetic_drive → 四轮关节扭矩
                       └→ /wheel_joint_states ───────────────┐
MoveIt → /arm_controller/follow_joint_trajectory              │
         → JointTrajectoryController → 六轴 effort 接口       │
         → arm_joint_state_broadcaster                       │
           → /arm_joint_state_broadcaster/joint_states ───────┤
                                                            ↓
                                                   joint_state_mux
                                                            ↓
                                                /joint_states（唯一发布者）
                                                            ↓
                                              robot_state_publisher + MoveIt
```

MoveIt 订阅实际状态，不靠发布假 joint_states 驱动 Gazebo。
同一话题由两个发布者发送互不重叠的关节子集本身不一定冲突，但本工程进一步采用单一汇总出口，方便检查完整十关节状态。
汇总节点拒绝错误来源的关节名，丢弃过时数据，不伪造缺失角度。
底盘四轮没有进入 ros2_control，机械臂控制器也没有声明四轮接口。
不要在仿真旁再启动 `joint_state_publisher_gui` 或另一套假硬件。

```bash
ros2 topic info /joint_states --verbose
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 action list | grep follow_joint_trajectory
```

## 末端相机

复用 `third_party/realsense2_description` 与 `realsense_gazebo_plugin`，
没有启动要求 USB 实物的 `realsense2_camera` 驱动。

| 输出 | 话题/坐标系 |
|---|---|
| RGB | `/d435i/camera/color/image_raw`，`d435i_color_optical_frame` |
| 深度 | `/d435i/camera/depth/image_rect_raw`，16UC1，毫米单位，`d435i_depth_optical_frame` |
| 红外 | `/d435i/camera/infra1/image_rect_raw`、`infra2/image_rect_raw` |
| 点云 | `/d435i/depth/color/points` |
| 相机内参 | 各 image 话题同目录下的 `camera_info` |
| IMU | `/camera/imu`，`d435i_gyro_optical_frame` |

默认 640×480、10 Hz，修改 `camera.width/height/fps` 可控制渲染开销。
深度不是工厂级 RealSense 噪声/畸变仿真，IMU 也为简化模型。
颜色/深度未做真实标定配准；本版本不把深度点云自动加入 MoveIt OctoMap，避免自体点云误占据。

## 动力学变化与边界

- UR3e 保留官方尺寸和惯量，无缩放。原有底盘加装后总质量明显增加。
- 每轮磁吸上限改为 250 N，车轮电机上限改为 18 N·m，均在统一 YAML 可调整；旧版 90 N / 8 N·m 的报告不代表加装后的承载能力。
- ODE 采用 `solver_type: world` 直接求解器与微小 `constraint_cfm`，改善多连杆/多接触的求解稳定性。
- 机械臂使用 effort 轨迹控制，位置反馈进入 PID，施加关节扭矩；没有逐帧直接设置连杆位姿。
- 相机使用 75 g 均质外壳惯量近似，替换了所提供描述中不满足惯量三角不等式的占位值。
- 挂载关节是刚性固定连接；没有柔性支架、线缆拖曳、真实负载辨识或联合车臂稳定性优化。
