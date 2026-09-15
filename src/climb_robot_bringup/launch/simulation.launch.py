"""Generate matching geometry, physics parameters and initial pose in one place."""
import math
import tempfile
from pathlib import Path
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from gazebo_models.world import generate_world, spawn_pose
import os


def setup(context):
    cfg_path=LaunchConfiguration('config').perform(context)
    config=yaml.safe_load(Path(cfg_path).read_text())
    robot=config['robot']
    for key,value in robot.items():
        if isinstance(value,bool):continue
        if not math.isfinite(float(value)) or (key!='axle_z' and float(value)<=0):
            raise ValueError(f'Invalid robot parameter: {key}={value}')
    if not isinstance(robot['lidar_enabled'], bool):
        raise ValueError('lidar_enabled must be a YAML boolean')
    if int(robot['lidar_samples'])!=robot['lidar_samples']:
        raise ValueError('lidar_samples must be an integer')
    if robot['track_width'] <= robot['body_width']+robot['wheel_width']:
        raise ValueError('track_width must clear the body and wheel half-widths')
    if robot['wheelbase']<=2*robot['wheel_radius']:
        raise ValueError('wheelbase must exceed wheel diameter')
    if robot['wheel_radius']-robot['axle_z']<=robot['body_height']/2:
        raise ValueError('body bottom must clear the contact plane')
    for key,value in config['physics'].items():
        if not math.isfinite(float(value)) or float(value)<=0:raise ValueError(f'Invalid physics parameter: {key}')
    mode=LaunchConfiguration('world').perform(context)
    pose=spawn_pose(config,mode,LaunchConfiguration('spawn_angle_deg').perform(context))
    # Persist artifacts for inspection; each launch gets an isolated directory.
    output=tempfile.mkdtemp(prefix='climb_robot_')
    world_path=generate_world(output,config,mode)
    description=xacro.process_file(str(Path(get_package_share_directory('robot_description'))/'urdf/climb_robot.urdf.xacro'),
        mappings={k:str(v).lower() if isinstance(v,bool) else str(v) for k,v in robot.items()}).toxml()
    Path(output,'robot.urdf').write_text(description)
    plugin_dir=str(Path(get_package_prefix('robot_control_driver'))/'lib')
    return [
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH',plugin_dir+os.pathsep+os.environ.get('GAZEBO_PLUGIN_PATH','')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('gazebo_ros'))/'launch/gazebo.launch.py')),
            launch_arguments={'world':world_path,'gui':LaunchConfiguration('gui'),'verbose':'true'}.items()),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':description,'use_sim_time':True}],output='screen'),
        Node(package='gazebo_ros',executable='spawn_entity.py',arguments=['-entity','climb_robot','-file',str(Path(output,'robot.urdf')),
            '-x',str(pose[0]),'-y',str(pose[1]),'-z',str(pose[2]),'-R',str(pose[3]),'-P',str(pose[4]),'-Y',str(pose[5]),'-timeout','120'],output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(Path(get_package_share_directory('climb_robot_bringup'))/'config/simulation.yaml')),
        DeclareLaunchArgument('world',default_value='hemisphere',choices=['hemisphere','sphere','ground']),
        DeclareLaunchArgument('gui',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('spawn_angle_deg',default_value='0'),
        OpaqueFunction(function=setup)])
