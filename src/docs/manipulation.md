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
在 RViz 的 MotionPlanning 中选择 `arm` 组。默认只显示实际机器人，避免启动时的目标叠影。
需要拖动末端时，在 Displays → MotionPlanning → Planning Request 中勾选
`Query Goal State`，再选择工具栏 `Interact`，拖动目标并点击 Plan、Execute；
也可以选命名目标 `ready` 或 `inspect`。路径通过碰撞检查后发送给 Gazebo 中的机械臂。

脚本方式同样经过 MoveIt 规划和执行，默认将当前肩部绕竖轴角度增加 0.25 rad：

```bash
ros2 run climb_arm_moveit_config arm_demo.py --pan-offset 0.25
```

机械臂操作期间保持底盘停止。首版是“移动到位后操作机械臂”，没有车臂协同规划。
MoveIt 的规划参考系是 `world`；浮动虚拟关节 `world_joint` 从现有
`world → base_link` TF 跟踪底盘位姿，但不属于 `arm` 组，也不驱动底盘。
环境节点只安装一次世界坐标系中的静态球壳和地面，不再每 0.2 秒移动整片罐壁。
球壳与地面对机械臂参与碰撞检查，仅允许四轮与这些表面的正常接触。
已知的视觉焊缝不作为障碍物，RGB/深度相机仍能看到视觉焊缝。

## RViz 性能与规划碰撞模型

- `arm` 规划组始终只有机械臂六轴。完整机器人描述仍保留车体、雷达和末端相机，
  让机械臂规划检查这些碰撞；把这些 link 删掉并不能正确代替性能优化。
- Gazebo 使用原有高精度球壳接触网格，MoveIt 单独使用 `tank_planning.stl`：
  默认半球由 73,728 面降为 4,608 面。规划壳体内面向内保守近似，外面扩张包住外球面。
  半径 3 m 时，内部可用间隙的保守缩减上界为半球约 14.5 mm、完整球约 25.7 mm；
  近壁精细作业应结合这个余量评估，不能当作和 Gazebo 完全相同的碰撞几何。
- RViz 默认关闭 `Scene Geometry → Show Scene Geometry`，只隐藏罐体绘制，
  不删除规划器中的碰撞对象。需要检查障碍物时可重新勾选。
- `Move Camera` 用来旋转/平移视角；`Interact` 用来拖动启用的机械臂目标。
- 改动后构建 `gazebo_models`、`climb_robot_bringup`、`climb_arm_moveit_config`，
  再重启 launch。不要用 RViz 保存的旧配置覆盖新的默认配置。

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
  camera_mount: {xyz: [0.035, 0.05, 0.03], rpy: [0.0, 3.14159265359, 1.57079632679]}
  camera: {xyz: [0.02, 0.0, -0.061], rpy: [3.14159265359, 0.0, 0.0]}
  imu: {xyz: [0.0, 0.0, 0.0], rpy: [0.0, 0.0, 0.0]}
```

| 安装点/关节 | 相对父坐标系 | 修改入口 |
|---|---|---|
| UR3e 基座固定关节 | `base_link` | `mounts.arm.xyz/rpy` |
| VLP-16 安装关节 | `base_link` | `mounts.lidar.xyz/rpy` |
| D435i 支架固定关节 | `arm_flange` | `mounts.camera_mount.xyz/rpy` |
| D435i 底部安装螺孔固定关节 | `arm_flange` | `mounts.camera.xyz/rpy` |
| 壳体中心 IMU 固定关节 | `base_link` | `mounts.imu.xyz/rpy` |
| 四轮转动关节 | `base_link` | 前后 `x=±wheelbase/2`，左右 `y=±track_width/2`，高度 `z=axle_z` |
| UR3e 内部六轴的固定几何变换 | 各自上一节连杆 | 官方 `ur_description/config/ur3e/default_kinematics.yaml`，可用 `arm.kinematics_file` 指定自己的副本 |
| 六轴初始角度 | 各关节转轴 | `arm.initial_positions`，顺序见下文；这是关节角，不是安装 origin |

`xyz` 单位为米，`rpy` 单位为弧度，旋转采用 URDF 的 roll/pitch/yaw 约定。
`origin` 定义父链接到子关节零位坐标系的固定变换；`axis` 定义转轴；运行时角度 `q` 是另一项变换。
例如把 `mounts.lidar.xyz[0]` 从 -0.15 改为 -0.18，会将雷达向车尾移动 3 cm。
相机本体以 +x 朝前，光学帧以 +z 看向场景；当前安装沿用提供的 MiR 模板，
相机与支架均以 `arm_flange` 为安装参考，不再使用原来悬空的 tool0 偏移。
`gazebo_models/D435i_mounted.STL` 的单位为毫米，视觉和碰撞均用 0.001 缩放。
支架暂按 50 g、包围盒近似惯量建模，真实质量和质心需要实测后校准。
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
转发节点拒绝错误来源的关节名，保留原始时间戳与速度，每个来源最多 50 Hz。
每条消息可只含轮子或机械臂子集；下游按关节名更新，不把异步测量拼成同一时刻。
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
| 车体 IMU | `/imu/data`，`imu_link`，100 Hz |

默认 640×480、10 Hz，修改 `camera.width/height/fps` 可控制渲染开销。
深度不是工厂级 RealSense 噪声/畸变仿真，IMU 也为简化模型。

车体 `imu_link` 默认在 base_link 的 `(0, 0, 0)`，位于 12 cm 高壳体内部中心，
轴向与车体一致。绿色小盒表示电路板位置，正常外部视角会被壳体挡住；
通过 RViz TF 显示可检查它的位置。电路板质量已包含在 body_mass 中，不再重复加质量，
也不增加与车体重叠的内部碰撞体。车体 IMU 不依赖机械臂或相机开关。

`/imu/data` 与 D435i 的 `/camera/imu` 独立；姿态使用世界参考，
加速度与角速度在各自传感器坐标系中表达。车体 IMU 噪声标准差暂设为
0.00017 rad/s 和 0.01 m/s²，不代表尚未选型的真实传感器指标。
可用 `ros2 topic echo /imu/data --once` 检查，静止水平时 z 轴加速度应接近 +9.81 m/s²。
颜色/深度未做真实标定配准；本版本不把深度点云自动加入 MoveIt OctoMap，避免自体点云误占据。

## 动力学变化与边界

- UR3e 保留官方尺寸和惯量，无缩放。原有底盘加装后总质量明显增加。
- 每轮磁吸上限改为 250 N，车轮电机上限改为 18 N·m，均在统一 YAML 可调整；旧版 90 N / 8 N·m 的报告不代表加装后的承载能力。
- ODE 采用 `solver_type: quick`、300 次迭代、1 ms 步长和 `constraint_cfm: 1e-6`。
- 机械臂使用 effort 轨迹控制，位置反馈进入 PID，施加关节扭矩；没有逐帧直接设置连杆位姿。
- 相机使用 75 g 均质外壳惯量近似，替换了所提供描述中不满足惯量三角不等式的占位值。
- 挂载关节是刚性固定连接；没有柔性支架、线缆拖曳、真实负载辨识或联合车臂稳定性优化。


## 稳定运行配置（2026-09-16）

默认加载 `climb_arm_moveit_config/config/stable.rviz`：Fixed Frame 为 world，
深度点云默认关闭，开启后使用 Points，不积累历史点；需要拖动机械臂目标时再开启 Query Goal State。
用户原有安装目录里的 `moveit.rviz` 已备份并保留；使用个人配置时显式指定：

```bash
ros2 launch climb_robot_bringup simulation.launch.py rviz_config:=/absolute/path/personal.rviz
```

MoveIt 和 RViz 等 arm_controller 成功激活后启动。Humble 插件卸载存在关闭崩溃，
launch 为这两个进程单独设置 LD_PRELOAD，让相关插件库保持加载到进程退出；不修改系统库。

机械臂硬件适配层仍转交原 gazebo_ros2_control 的 effort 命令，速度反馈改为实际关节角度
相对仿真时间的差分；原始 ODE 速度保留在 `/arm/raw_joint_states`，用于比较，不能再转发到 `/joint_states`。
位置、惯量、PID 和停止容差保持不变。该处理修正的是仿真反馈不一致，不等同于解决 ODE 的全部动力学误差。
详细原因、对比实验与测试范围见 [稳定性修复报告](stability-fix-2026-09-16.md)。
