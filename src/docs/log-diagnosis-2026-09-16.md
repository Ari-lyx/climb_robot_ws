# 2026-09-16 启动、执行与 RViz 日志诊断

本轮只分析日志和配置，未启动仿真、修改控制器或覆盖用户保存的 RViz 配置。

## 证据范围

- `/home/lyx/.ros/log/2026-09-16-08-36-04-216203-lyx-Nitro-AN515-57-6951/launch.log`
- 同级日志根目录内 `move_group_6964_1789518965827.log`、`rviz2_6968_1789518965873.log`、`gzserver_6970_1789518966326.log`。
- 源码与 install 中的 RViz 配置、ros2_controllers.yaml、simulation.launch.py。
- 未取得卡死时线程栈、内存/GPU 时间序列、TF 或控制器状态录包，因此不能确认 RViz 卡死根因，也不能判定内存泄漏。

## 时间线（本地时间）

| 时间 | 事件 |
|---|---|
| 08:36:04 | launch 启动 |
| 08:36:10–08:36:33 | 交互标记反复初始化；当时模型尚在加载 |
| 08:39:57、08:40:07 | world 到 base_link 的 TF 向未来外推，分别相差 17ms、6ms，交互标记重置 |
| 08:40:19、08:40:27 | 两次规划在约 5 秒预算内未找到解 |
| 08:40:41 | 后续规划成功 |
| 08:40:43 | 控制器接受执行目标 |
| 08:42:21 | 控制器因终点速度超限中止 |
| 08:46:23 | 深度光学坐标系消息因 TF MessageFilter 队列满被丢弃，日志仅记录一次 |
| 12:01:09 | 用户 Ctrl-C；gzserver 正常退出 |
| 12:01:10–11 | move_group 退出码 -11，RViz -6，发生于关闭阶段 |

总运行约 3 小时 25 分钟。执行失败发生在启动约 6 分钟后，丢消息发生在约 10 分钟后；不能从这两条日志推出“两小时后才发生故障”。关闭阶段崩溃也不能直接解释之前的界面卡死。

## 已明确的执行失败

`GOAL_TOLERANCE_VIOLATED`，超过 `goal_time=5.0` 后仍未满足终点条件：

- joint 1（按控制器关节列表为 shoulder_lift）：速度误差 -0.164194 rad/s。
- joint 3（wrist_1）：速度误差 0.110623 rad/s。
- 停止速度阈值为 0.08 rad/s。

这是 effort 控制器未满足停止条件，RViz 的 failed or timeout 是上层通用提示。应记录 `/arm_controller/state` 的目标、实际、误差，并观察姿态、底盘运动和接触状态，判断是否振荡、负载影响或接触扰动，再调 PID/阻尼或仿真参数。不能仅放宽阈值、加长超时来宣布修复。前面两次规划失败与这次执行失败是不同问题；日志未记录足够的目标/碰撞信息，无法确定那两次未找到解的原因。

## RViz 与 TF

已确认 TF 时序错误；尚未确认与深度消息队列满、后续卡死具有同一个根因。

启动读取 install 中的 `climb_arm_moveit_config/config/moveit.rviz`，并非 src 中的简化配置。install 文件是独立文件，修改时间早于此次启动：

- Fixed Frame 为 base_link；Query Goal State 为 true（src 为 false）。
- 添加了雷达点云、Camera、深度点云三个显示项。
- 深度点云 Style 为 Boxes，Decay Time 为 0；这些传感器显示项保存为禁用，无法由此判断运行中何时打开。
- 没有发现配置设置长期点云累积的证据。

建议按顺序复现：

1. 保留用户配置备份，使用轻量 RViz 基线，仅模型、Grid、MotionPlanning。Fixed Frame 和 Orbit Target Frame 设 world；这是利用已有真值 TF，不依赖 SLAM。world 可以避免交互标记从规划 world 到移动 base 的这条转换，但不能代替修复传感器 TF 时序。
2. 验证稳定后依次开启雷达、深度显示，记录 FPS、RViz RSS、CPU、GPU 使用率、Gazebo 实时率。深度用 Points/小像素点，Decay Time=0，必要时只给显示流降采样/限频；不要先扩大队列。
3. 在消息原始时间戳检查完整 TF 链，确认 /clock 连续、use_sim_time、joint_states 新鲜度、TF 发布延迟及重复发布者。当前 joint_state_mux 以当前时刻重打最新缓存状态时间戳，存在时间近似，需用数据判断影响，不能仅靠调缓存大小解决。
4. 若仍卡死，关闭前保存线程栈与资源数据，确认是渲染、TF 回调还是插件锁等待。日志中没有足够证据判定泄漏。
5. 临时恢复可以只重启 RViz，继续使用已有仿真与 move_group；应通过相同 launch 参数加载机器人和 MoveIt 配置，避免误启动第二套 gzserver。

## 其他日志说明

- `/recognize_objects`：可选物体识别 action 未部署。不是 arm 轨迹控制服务；本次后续规划成功和控制器接受目标证明它未阻断该流程。
- IMU visual 无 collision：内部 PCB 可视化有意不重复添加车体内部碰撞体。警告不等于 IMU 数据异常；若清理警告，可去掉内部可视几何。不要为消日志随意添加影响碰撞规划的模型。
- KDL 根链接惯量：KDL 不支持根 link 惯量；不表示 Gazebo 丢弃该惯量。本次机械臂运动学已经加载并规划成功。后续如加无惯量 dummy root，须一起审查 SRDF virtual joint、TF 唯一父节点和 Gazebo 插件，不能直接删除底盘物理惯量。
- Gazebo Event connection：一个事件连接很快析构的生命周期警告；不是 Gazebo 网络连接失败。检查到本仓库磁吸驱动和 RealSense 的主要 Connect 调用都有保存指针，仅凭这一行不能定位插件或断言无害。
- `No 3D sensor plugin(s) defined for octomap updates`：MoveIt 没有配置实时 Octomap 传感器更新，不影响当前 CAD 碰撞环境；RViz 显示点云不等于 MoveIt 自动使用点云避障。
- `world` 不存在：启动时 move_group 早于机器人生成，首次 TF 查询失败。需要与运行期间持续 TF 故障区分。
- namespace reload、Connected、Service response：初始化信息；本次共有 343 次初始化请求和响应，绝大多数在启动期，并非两小时持续刷屏。
- SIGINT 后 -11 / -6：分别为 SIGSEGV / SIGABRT，是真实的关闭异常，需崩溃栈单独定位。

## 为什么出现 gzserver

`simulation.launch.py` 显式执行 `gzserver ... --verbose`，并设置 `output='screen'`。gzserver 负责物理、传感器和仿真插件，gzclient 是图形客户端。Gazebo Classic 的图形启动也会使用服务器组件；之前是否隐藏输出，需要对照旧 launch 才能确定。

`127.0.0.1:11345` 是这次 Gazebo master 的本机地址；打印局域网地址不是远程仿真的证据。本次 PID 6970 由同一 launch 启动，日志确认其在 Ctrl-C 后正常退出，没有证据说明这次 gzserver 残留。

## 上游依据

- MoveIt Humble 可选物体识别客户端：[motion_planning_frame.cpp](https://github.com/moveit/moveit2/blob/humble/moveit_ros/visualization/motion_planning_rviz_plugin/src/motion_planning_frame.cpp)。
- Gazebo 事件连接析构警告：[Event.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/common/Event.cc)。
- TF MessageFilter 行为：[ROS 2 源码文档](https://docs.ros2.org/foxy/api/tf2_ros/message__filter_8h_source.html)；队列满是一种消息丢弃原因，不能单独证明渲染过载或内存泄漏。
