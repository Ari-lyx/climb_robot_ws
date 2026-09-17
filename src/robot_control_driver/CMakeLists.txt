cmake_minimum_required(VERSION 3.16)
project(robot_control_driver)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
# Gazebo 11's system JsonCpp still requests old CMake policies.
# 中文说明：Gazebo 11 依赖的系统 JsonCpp 仍在使用旧的 CMake policy，
# 在 CMake 4.x 下需要显式放宽最低版本，否则配置阶段直接报错。
if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0")
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()
# Gazebo Classic 本体 + ROS 2 桥接
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
# 插件用到的 ROS 2 接口包
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)   # Twist  (/cmd_vel)
find_package(nav_msgs REQUIRED)        # Odometry (/odom)
find_package(sensor_msgs REQUIRED)     # JointState (/joint_states)
find_package(diagnostic_msgs REQUIRED) # DiagnosticArray (/magnet/diagnostics)
find_package(std_srvs REQUIRED)        # SetBool (/magnet/enable)
find_package(tf2_ros REQUIRED)         # world -> base_link 变换
# 编译成共享库：Gazebo 通过 libmagnetic_drive.so 这一文件名动态加载插件
# （URDF 里 <plugin filename="libmagnetic_drive.so"> 必须与此对应）。
add_library(magnetic_drive SHARED src/magnetic_drive.cpp)
target_include_directories(magnetic_drive PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(magnetic_drive PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(magnetic_drive ${GAZEBO_LIBRARIES})
ament_target_dependencies(magnetic_drive gazebo_ros rclcpp geometry_msgs nav_msgs sensor_msgs diagnostic_msgs std_srvs tf2_ros)
target_compile_options(magnetic_drive PRIVATE -Wall -Wextra)
# 安装到本包 lib/ 下，launch 会把该目录加入 GAZEBO_PLUGIN_PATH
install(TARGETS magnetic_drive LIBRARY DESTINATION lib)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 纯函数单元测试，不启动 Gazebo；需要 src 目录以便 #include "magnet_math.hpp"
  ament_add_gtest(test_magnet test/test_magnet.cpp)
  target_include_directories(test_magnet PRIVATE src)
  ament_add_gtest(test_encoder_rate test/test_encoder_rate.cpp)
  target_include_directories(test_encoder_rate PRIVATE src)
endif()
find_package(gazebo_ros2_control REQUIRED)
find_package(pluginlib REQUIRED)
add_library(encoder_gazebo_system SHARED src/encoder_gazebo_system.cpp)
target_include_directories(encoder_gazebo_system PRIVATE ${GAZEBO_INCLUDE_DIRS})
ament_target_dependencies(encoder_gazebo_system gazebo_ros2_control pluginlib rclcpp sensor_msgs)
pluginlib_export_plugin_description_file(gazebo_ros2_control encoder_gazebo_system.xml)
install(TARGETS encoder_gazebo_system LIBRARY DESTINATION lib)
ament_package()
