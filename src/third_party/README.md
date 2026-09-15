# 第三方传感器与来源

本包的 `urdf/lidar.xacro` 是 VLP-16 的薄接入层，直接调用系统安装的
`velodyne_description/urdf/VLP-16.urdf.xacro`，并复用
`libgazebo_ros_velodyne_laser.so`。发布 `/velodyne_points`（PointCloud2），
扫描坐标系为 `lidar`；TF 保留上游 `lidar_base_link → lidar` 固定变换。

- 上游：Dataspeed [velodyne_simulator](https://index.ros.org/p/velodyne_simulator/)
- 源码：[DataspeedInc/velodyne_simulator](https://bitbucket.org/DataspeedInc/velodyne_simulator.git)
- 实际版本：工作区 `validation/installed-packages.tsv`，采用已安装的 Humble Debian 二进制包。
- 许可证：`velodyne-description-copyright` 和 `velodyne-gazebo-plugins-copyright`。
- 激光默认 CPU ray，16 线、每圈 360 采样、5 Hz，可在统一 YAML 中调节采样数与频率。

现有 `gazebo-ros2-control-copyright` 仅作为原目录遗留来源记录保留。
本版本的驱动是自行实现的 Gazebo 关节扭矩插件，不加载 gazebo_ros2_control。
球罐网格为本工程程序生成，不使用外部模型资源。

统一启动器会在展开上游宏后，将网格的 `package://` URI 解析为本机安装目录下的绝对 `file://` URI；这样 Gazebo 转换 SDF 后无需通过模型数据库查找雷达网格。没有修改系统安装的第三方文件。
