# 磁吸球罐爬壁机器人仿真

ROS 2 Humble / Gazebo Classic 11。四轮滑移差速底盘、顶部 VLP-16、真实重力与轮胎接触、每轮独立近场磁吸力。

## 构建与运行

在工作区根目录执行（`src` 是本目录）：

```bash
source /opt/ros/humble/setup.bash
cd /home/lyx/Downloads/climb_robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch climb_robot_bringup simulation.launch.py
```

默认打开下半球，机器人从球底出发。另一个终端 source 相同环境后：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 或运行 10 秒的前进演示（按实际时间计时）
ros2 run climb_robot_bringup drive_demo.py --speed 0.2 --seconds 10
# 或持续向前；Ctrl-C 后 0.5 秒命令超时，速度环制动
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

不同场景（每次先退出上一仿真）：

```bash
# 从较陡的内壁开始，方便立即观察上爬
ros2 launch climb_robot_bringup simulation.launch.py spawn_angle_deg:=55
# 完整球壳的竖直内壁
ros2 launch climb_robot_bringup simulation.launch.py world:=sphere spawn_angle_deg:=90
# 完整球壳顶端倒挂
ros2 launch climb_robot_bringup simulation.launch.py world:=sphere spawn_angle_deg:=180
# 普通地面；球壳不存在，所以所有磁吸力为零
ros2 launch climb_robot_bringup simulation.launch.py world:=ground
# 无 GUI
ros2 launch climb_robot_bringup simulation.launch.py gui:=false
```

完整球壳会遮挡机器人，可在 Gazebo GUI 中将球罐设为透明，或者将相机移到球内；显示设置不改变碰撞。下半球为真实开口，驶出边缘会失去附着。

## 参数和接口

修改 `climb_robot_bringup/config/simulation.yaml` 后重新启动，或用 `config:=/absolute/path/custom.yaml` 指定完整配置。

| 参数组 | 主要参数 |
|---|---|
| 车体 | `body_length/width/height/mass` |
| 轮子 | `wheel_radius/width/mass`、`wheelbase`、`track_width`、`axle_z`、`friction` |
| 电机 | `motor_torque`、`velocity_gain`、`velocity_integral_gain`、速度/加速度/超时 |
| 磁吸 | `magnet_force`（每轮接触吸力上限 N）、`magnet_decay`、`magnet_range`（m） |
| 雷达 | `lidar_enabled`、`lidar_samples`、`lidar_hz` |
| 球罐 | 内半径、壁厚、球心高度、纬向/经向网格数 |
| 物理 | 仿真步长、实时更新频率、ODE 迭代次数 |

| 接口 | 含义 |
|---|---|
| `/cmd_vel` | Twist，车体前向速度与绕自身 z 的转向速度 |
| `/odom` | 完整三维 Gazebo 真值，父坐标系 `world`，子坐标系 `base_link` |
| `/tf`、`/tf_static` | world → base_link → 轮子、雷达 |
| `/joint_states` | 四轮角度和角速度 |
| `/velodyne_points` | PointCloud2，扫描坐标系 `lidar` |
| `/magnet/diagnostics` | 每轮 `gap_m`、`force_N`、开关状态；无有效壁面时 gap 为 nan |
| `/magnet/enable` | SetBool，全轮磁吸开关 |

```bash
ros2 service call /magnet/enable std_srvs/srv/SetBool '{data: false}'
ros2 service call /magnet/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /magnet/diagnostics
```

开关是仿真实验接口，永久磁铁实物并不能直接用软件关闭。重新启用只在有效捕获距离内恢复吸力，不会把远处机器人拉回球壁。停车时电机继续制动，磁吸只提供法向作用力，沿壁面的重力由轮胎摩擦及电机扭矩平衡。

## 验证与工程边界

```bash
colcon test --packages-select robot_control_driver
colcon test-result --verbose
python3 src/validation/test_geometry.py
python3 src/validation/run_physics.py ground
python3 src/validation/run_physics.py climb
python3 src/validation/run_physics.py wall
python3 src/validation/run_physics.py ceiling
python3 src/validation/run_physics.py delete
python3 src/validation/run_physics.py traverse
```

物理测试会自行启动无 GUI 仿真、检查位移/附着/释放/点云/TF，然后关闭该次仿真。结果默认写入 `/tmp/climb_validation`，可用 `--output` 修改。测试使用 ROS domain 187 和 Gazebo 端口 11455，请按顺序运行。

- 详细约束：`docs/requirements.md`。
- 实测结果：`validation/REPORT.md`。
- 第三方来源：`third_party/README.md`。
- 每次启动生成的 STL、SDF、URDF 放在 `/tmp/climb_robot_*`，方便检查；仿真退出后可删除该次目录。

这是刚体运动和附着能力模型，磁力是有界距离衰减近似。曲面差速包含滑移，不保证输入角速度与实际角速度一致；提高磁力会增加转弯阻力。尚未加入 SLAM、路径规划、柔性轮胎、悬架或磁场有限元。

## 启动排错

- RViz 视角操作：工具栏选择 `Move Camera`，左键拖动旋转、中键拖动平移、滚轮缩放；拖动机械臂目标使用 `Interact`。`moveit.rviz` 必须显式配置 `Tools`，仅配置 `Views/Orbit` 不够。
- 默认 ODE 求解器使用 `quick`。`world` 直接求解器在本模型的轮胎/球壳接触中复现了 `LCP internal error, s <= 0`；`solver_iterations` 只影响 `quick`。修改配置后需重新构建 `climb_robot_bringup`、`climb_arm_moveit_config` 并重启 launch，已运行的 gzserver 不会自动读取新配置。
- 用对应 `GAZEBO_MASTER_URI` 下的 `gz stats -p` 检查实时倍率；相机的 10 Hz 是仿真时间频率，0.2 倍实时速度时墙钟输出只有约 2 Hz。这与 RViz 的界面渲染 FPS 是两个指标。关闭 gzclient 不会自动停止 gzserver，请在启动 launch 的终端按 Ctrl+C。
- `ros2 run` 的可执行程序名是 `drive_demo.py`（包含 `.py`）。它通过 CMake 的 `install(PROGRAMS ...)` 安装，无需在 package.xml 注册 executable。
- 使用 `--symlink-install` 时源码脚本也必须有执行权限。本工程已将 `scripts/drive_demo.py` 设为可执行；复制工程时请保留权限。
- `ros2 pkg executables climb_robot_bringup` 应显示 `climb_robot_bringup drive_demo.py`。若没有，确认 source 的是本工作区 `install/setup.bash`。
- 本启动器直接启动 Gazebo 并使用独立模型目录，避免系统其他包的模型路径导出把普通 ROS 包当作模型扫描。球罐使用本地生成网格，雷达使用已安装的第三方资源，不需要下载 TurtleBot3 模型。
- 雷达 DAE 网格由统一启动器解析为绝对 `file://` 路径；如果依赖文件缺失，会在启动前报告具体路径。模型数据库使用本地空清单，无需在线下载。
- 可运行 `python3 src/validation/run_physics.py ground --gui` 进行带 Gazebo 客户端的验证（会打开测试窗口并在完成后结束测试进程）。

## 球罐焊缝

`gazebo_models/gazebo_models/welds.py` 独立生成半椭圆焊缝网格与程序化银灰鱼鳞纹理。
罐壁默认棕褐色；半球显示下部两道环缝、两组错开的纵缝，完整球壳显示四道环缝和三组纵缝；两极保留完整封头。

集中配置在 `climb_robot_bringup/config/simulation.yaml`：

- `tank.color_rgba`：罐壁 RGBA 颜色。
- `welds.enabled`：焊缝视觉开关。
- `welds.width / height`：半椭圆截面全宽/凸起高度，默认 45 mm / 6 mm。
- `welds.ring_angles_deg`：从球底测量的环缝极角，默认 `[25, 70, 110, 155]`。
- `welds.meridians_per_band / band_offsets_deg`：各相邻环缝之间的纵缝条数和错缝偏移。
- `welds.texture_pitch`：鱼鳞纹的纵向重复间距，默认 12 mm。
- `welds.segment_length / cross_section_segments`：网格精度。
- `welds.surface_offset`：防止焊缝被球壳离散三角面遮盖的微小内移量。

生成资源 `welds.dae`、`weld_scales.png` 随本次世界保存在 `/tmp/climb_robot_*`。
焊缝只包含 visual，没有 collision：不会增加越障阻力，也不会改变当前 CPU ray 雷达的点云。
鱼鳞纹是程序化近似，不是参考照片的直接贴图；当前不建模焊缝热影响区或微观粗糙几何。

实际 Gazebo 渲染图：`validation/results/welds/overview.png` 和 `closeup.png`。
可用 `python3 src/validation/render_welds.py` 重新渲染默认半球的总览和近景（需要本机图形环境、Pillow）；输出在 `/tmp/climb_weld_preview`。
