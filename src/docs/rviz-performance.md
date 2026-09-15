# RViz 性能修复与验证（2026-09-15）

## 原因与修改

原环境节点每 0.2 秒把整个 73,728 面罐体碰撞网格的位置重新表达为 base_link 坐标。
MoveIt Humble 的 PlanningSceneDisplay 在场景更新时设置几何重绘标志；仅关闭显示开关
只会隐藏几何节点，不能阻止所有几何处理。机器人描述包含底盘并不是主要瓶颈：
原 SRDF 的 arm 组本来就只规划六轴。

现在使用 world 规划坐标系，浮动虚拟关节通过 TF 跟踪底盘，仍只有 arm 组被控制。
环境只安装一次静态地面与球壳；规划球壳使用 4,608 面轻量网格，Gazebo 接触网格不变。
内面保守收缩自由空间，外面保守扩张，3 m 半径的最大内部余量损失界为半球
14.5 mm / 完整球 25.7 mm。默认隐藏障碍物绘制、关闭查询目标叠影及轨迹拖尾。

另将 quick 迭代次数从 100 增加到 300。100 次下有执行结束速度超出 0.08 rad/s
容差的复现；尝试增加 PID 阻尼未改善，已撤回，最终 PID 和执行容差均未放宽。
300 次下本轮验收成功，但不能据此保证所有姿态和长时间驻车控制稳定。

## 本机实测

- 修改前独立仿真 `/tmp/rviz_perf_before/launch.log`：载入罐体后出现数十秒不刷新。
  初版探针累计了两个 OpenGL drawable 的交换次数，初始约 62 swaps/s **不是 62 FPS**；
  阻塞期约 0.07 swaps/s。不能把这一日志当作精确逐窗口 FPS 基准。
- 修改后探针按 drawable 分开统计，约 31.2 FPS；RViz 状态栏显示 31 fps。
  `/tmp/rviz_perf_after/launch.log`、`/tmp/climb_arm_converged_hemisphere_0/launch.log`。
- 实际点击 Move Camera 并拖动后，截图确认视角改变，状态栏仍显示 31 fps。
  `/tmp/rviz_mouse_before.png`、`/tmp/rviz_mouse_after.png`。
- GetPlanningScene 中根关节为 world_joint，世界平移与同时读取的 /odom 相符；
  ground、tank 均存在，tank 为 4,608 面。
- 最终半球验收：MoveIt SUCCEEDED，目标 pan 变化 0.25 rad，实测 0.240575 rad；
  四轮吸力均为 250 N；机械臂控制器 active；彩色/深度/双红外图像存在；
  /joint_states 只有一个发布者。结果在 `/tmp/climb_arm_converged_hemisphere_0/result.json`。
- 新增规划网格闭合性、内外保守边界测试通过；已有几何测试 2/2 通过。
- 三个相关包构建安装成功；测试结束后进程检查没有 gzserver、rviz2、move_group 残留。

## 范围和剩余问题

测试为默认半球、无 gzclient 窗口、有 D435i 渲染；其他场景/不同显示负载未作 FPS 保证。
未消除底盘缓慢漂移，未进行完整贴壁/倒挂长时间机械臂验收。
RViz 在测试关停阶段仍有 -11 退出，运行期间没有据此判定为崩溃；该退出问题未修复。
启动后需等机器人和规划场景加载完成再规划。

操作方式见 [机械臂使用说明](manipulation.md)。

## 参考

- [MoveIt Humble PlanningSceneDisplay 源码](https://github.com/moveit/moveit2/blob/humble/moveit_ros/visualization/planning_scene_rviz_plugin/src/planning_scene_display.cpp)
- [用户提供的 UR-MiR 仓库](https://github.com/Spartan-Velanjeri/UR-MiR-mobile-manipulator)：参考其仿真和机械臂规划分工；本项目仍保留与车体、相机、环境的碰撞检查。
