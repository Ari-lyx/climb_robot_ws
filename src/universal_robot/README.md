# Universal Robots sources and local integration

Official repositories cloned on 2026-09-15, Humble branch:

| Repository | Pinned commit | Usage |
|---|---|---|
| [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) | `18e6f603b3ebc2ec479fecb62d6be544b15755e9` | Build `ur_description`; original UR3e meshes, joint geometry, limits and inertias |
| [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) | `4830f3d7425819712146743fd8572b24cff8888f` | Preserve official ROS 2 driver and MoveIt reference sources; repository has local `COLCON_IGNORE` |

The hardware driver is not needed for Gazebo and is not started. The local
`climb_arm_moveit_config` package connects MoveIt to the combined climbing robot
through `gazebo_ros2_control` and an effort JointTrajectoryController. No fake
hardware and no second robot_state_publisher are launched. The upstream model
is not scaled or edited; preserve all upstream LICENSE files, including asset
licenses. Third-party code is not covered by this workspace's root license.

The camera's 75 g case inertia is an approximate uniform-box model, not factory
calibration. The arm's mass is taken from this pinned description (about 10.8 kg),
which can differ from vendor total shipping/cable mass specifications.
