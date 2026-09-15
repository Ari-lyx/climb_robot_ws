"""统一生成几何、物理参数与出生位姿的仿真启动文件。

为什么这个文件长得不像“标准”的 ROS 2 launch（`ld = LaunchDescription([...]); return ld`）？
1. 常规写法在“构造 launch 描述”阶段（即 `generate_launch_description()` 被调用时）就要
   把所有值确定下来；
2. 但本文件必须先读到运行时才存在的 YAML 路径，再现场生成球罐网格、世界 SDF、URDF，
   并计算出生位姿，把这些结果注入节点参数和命令行；
3. 这一切必须延迟到 `ros2 launch` 解析完命令行参数之后才能做，所以用 `OpaqueFunction`
   把 `setup(context)` 注册进去。launch 系统在展开描述时会用当前 context 回调该函数，
   并把 `setup` 返回的动作列表**就地插入**到 LaunchDescription 中；
4. 因此 `LaunchConfiguration(...).perform(context)` 只能写在 `setup` 里——在
   `generate_launch_description()` 中调用时还没有 context，取不到用户传入的值。

一句话：`generate_launch_description()` 只声明“有哪些参数、要回调谁”，真正的世界构建
放在 `setup(context)` 里延迟执行。
"""
import math
import tempfile
from pathlib import Path
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from gazebo_models.world import generate_world, spawn_pose, planning_shell_mesh
import os
import runpy


def setup(context):
    """在 launch 展开阶段执行：读配置 -> 校验 -> 生成世界与 URDF -> 返回动作列表。"""
    # ---- 1) 读取集中配置文件：YAML 是几何/物理/磁吸参数的唯一真值来源 ----
    cfg_path=LaunchConfiguration('config').perform(context)
    config=yaml.safe_load(Path(cfg_path).read_text())
    robot=config['robot']
    arm_enabled=config.get('arm',{}).get('enabled',True) and LaunchConfiguration('arm').perform(context)=='true'
    camera_enabled=config.get('camera',{}).get('enabled',True) and LaunchConfiguration('camera').perform(context)=='true'
    moveit_enabled=arm_enabled and LaunchConfiguration('moveit').perform(context)=='true'
    # ---- 2) 参数校验：长度/质量等必须是有限正数；axle_z 允许为负（轮轴低于车体中心）----
    for key,value in robot.items():
        if isinstance(value,bool):continue
        if not math.isfinite(float(value)) or (key!='axle_z' and float(value)<=0):
            raise ValueError(f'Invalid robot parameter: {key}={value}')
    # 布尔值必须是真正的 YAML bool，而不是字符串 "true"，否则 xacro 实参类型不明确。
    if not isinstance(robot['lidar_enabled'], bool):
        raise ValueError('lidar_enabled must be a YAML boolean')
    if int(robot['lidar_samples'])!=robot['lidar_samples']:
        raise ValueError('lidar_samples must be an integer')
    # ---- 3) 几何自洽性：轮子不能陷进车体、轴距要放得下两个轮、车底不能穿透接触面 ----
    if robot['track_width'] <= robot['body_width']+robot['wheel_width']:
        raise ValueError('track_width must clear the body and wheel half-widths')
    if robot['wheelbase']<=2*robot['wheel_radius']:
        raise ValueError('wheelbase must exceed wheel diameter')
    if robot['wheel_radius']-robot['axle_z']<=robot['body_height']/2:
        raise ValueError('body bottom must clear the contact plane')
    for key,value in config['physics'].items():
        if key=='solver_type':
            if value not in ('world','quick'):raise ValueError('solver_type must be world or quick')
            continue
        if not math.isfinite(float(value)) or float(value)<=0:raise ValueError(f'Invalid physics parameter: {key}')
    # ---- 4) 世界模式 + 出生位姿：位姿由解析几何算出，保证初始不与球壳穿插 ----
    mode=LaunchConfiguration('world').perform(context)
    pose=spawn_pose(config,mode,LaunchConfiguration('spawn_angle_deg').perform(context))
    # ---- 5) 世界由 Python 现场生成（没有 .world 静态文件）----
    # Persist artifacts for inspection; each launch gets an isolated directory.
    output=tempfile.mkdtemp(prefix='climb_robot_')
    world_path=generate_world(output,config,mode)
    # ---- 6) YAML 的值直接作为 xacro 的 arg 映射，展开成完整 URDF ----
    # 布尔统一转成小写字符串，xacro 侧再用 $(arg xxx) 取用。
    mappings={k:str(v).lower() if isinstance(v,bool) else str(v) for k,v in robot.items()}
    for mount,values in config['mounts'].items():
        for field in ('xyz','rpy'):
            if len(values[field])!=3 or not all(math.isfinite(float(x)) for x in values[field]):raise ValueError(f'Invalid {mount} {field}')
            mappings[f'{mount}_{field}']=' '.join(str(x) for x in values[field])
    if config['arm'].get('kinematics_file'):
        mappings['arm_kinematics_file']=str(Path(config['arm']['kinematics_file']).resolve(strict=True))
    arm_initial=config['arm']['initial_positions']
    if len(arm_initial)!=6 or not all(math.isfinite(float(x)) for x in arm_initial):raise ValueError('Need six finite arm initial_positions')
    initial_path=Path(output,'arm_initial.yaml')
    initial_path.write_text(yaml.safe_dump(dict(zip(['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'],arm_initial))))
    mappings.update(arm_enabled=str(arm_enabled).lower(),camera_enabled=str(camera_enabled).lower(),arm_initial_file=str(initial_path))
    for key in ('width','height','fps'):mappings['camera_'+key]=str(config['camera'][key])
    robot_xml=xacro.process_file(str(Path(get_package_share_directory('robot_description'))/'urdf/climb_robot.urdf.xacro'),
        mappings=mappings)
    # URDF-to-SDF turns package:// into model://, which Gazebo cannot resolve
    # with an isolated model directory. Resolve installed mesh assets explicitly.
    # 中文说明：URDF→SDF 会把 package:// 改写成 model://，而 Gazebo 在隔离的模型目录下
    # 解析不了 model://，所以这里把网格显式解析成绝对 file:// URI。
    for mesh in robot_xml.getElementsByTagName('mesh'):
        uri=mesh.getAttribute('filename')
        if uri.startswith('package://'):
            package,relative=uri[len('package://'):].split('/',1)
            asset=Path(get_package_share_directory(package),relative).resolve()
            if not asset.is_file():
                raise FileNotFoundError(f'Missing robot mesh: {asset}')
            mesh.setAttribute('filename',asset.as_uri())
    # gazebo_ros2_control forwards URDF via a ROS parameter CLI argument. XML
    # comments containing ': ' can be misread as YAML; they are not model data.
    def strip_comments(node):
        for child in list(node.childNodes):
            if child.nodeType==child.COMMENT_NODE:node.removeChild(child)
            else:strip_comments(child)
    strip_comments(robot_xml)
    description=robot_xml.toxml()
    Path(output,'robot.urdf').write_text(description)
    # The stock gazebo_ros launch collects every installed package's model exports.
    # Some exports point at the entire ROS share directory, producing false
    # 'Missing model.config' errors for ordinary ROS packages in the Insert panel.
    # 中文说明：官方 gazebo_ros launch 会收集所有已安装包的 model 导出，有些导出指向
    # 整个 ROS share 目录，导致 Insert 面板里普通 ROS 包报假的 'Missing model.config'。
    # 这里只给 Gazebo 一个空模型目录，绕开这些干扰。
    model_dir=Path(output,'models');model_dir.mkdir()
    # Gazebo 11 interprets an empty database URI as '/', not as offline mode.
    # 中文说明：Gazebo 11 会把空的数据库 URI 当成 '/'（联网查找），所以必须给一个
    # 本地空清单，才是真正的离线模式。
    database_dir=Path(output,'model_database');database_dir.mkdir()
    (database_dir/'database.config').write_text('<database><models/></database>')
    # 插件搜索路径：本包自研插件 + gazebo_ros 官方插件 + 雷达点云插件，
    # 并保留当前环境已有的 GAZEBO_PLUGIN_PATH；dict.fromkeys 用于去重且保持顺序。
    plugin_dirs=[str(Path(get_package_prefix(package))/'lib') for package in
                 ('robot_control_driver','gazebo_ros','velodyne_gazebo_plugins','realsense_gazebo_plugin','gazebo_ros2_control')]
    plugin_dirs.append(os.environ.get('GAZEBO_PLUGIN_PATH',''))
    gazebo_env={
        'GAZEBO_MODEL_PATH':str(model_dir),
        'GAZEBO_PLUGIN_PATH':os.pathsep.join(dict.fromkeys(p for p in plugin_dirs if p)),
        'GAZEBO_MODEL_DATABASE_URI':database_dir.as_uri(),
    }
    # ---- 7) 返回要启动的动作：gzserver / gzclient / robot_state_publisher / spawn_entity ----
    # 这些动作会被 OpaqueFunction 就地插入到 LaunchDescription 中。
    extras=[Node(package='climb_robot_bringup',executable='joint_state_mux.py',parameters=[{'use_sim_time':True}],output='screen')]
    if arm_enabled:
        for controller in ('arm_joint_state_broadcaster','arm_controller'):
            extras.append(Node(package='controller_manager',executable='spawner',arguments=[controller,'--controller-manager','/controller_manager','--controller-manager-timeout','120'],output='screen'))
    if moveit_enabled:
        # MoveIt does not need the dense Gazebo contact mesh. Inscribed inner
        # facets conservatively reduce free space; expand the outer surface so
        # its coarse facets still enclose the physical outer sphere.
        planning_mesh=''
        if mode!='ground':
            planning_mesh=str(Path(output,'tank_planning.stl'))
            radius=float(config['tank']['radius'])
            planning_shell_mesh(planning_mesh,radius,float(config['tank']['thickness']),mode=='hemisphere')
        share=Path(get_package_share_directory('climb_arm_moveit_config'))
        moveit_config=runpy.run_path(str(share/'config/build_config.py'))['make_moveit'](description,arm_initial)
        extras.append(Node(package='moveit_ros_move_group',executable='move_group',parameters=[moveit_config],output='screen'))
        extras.append(Node(package='climb_robot_bringup',executable='planning_environment.py',parameters=[{'use_sim_time':True,'world_mesh':planning_mesh, 'radius':float(config['tank']['radius']),'center_z':float(config['tank']['center_z'])}],output='screen'))
        extras.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'config/moveit.rviz')],parameters=[moveit_config],condition=IfCondition(LaunchConfiguration('rviz')),output='screen'))
    return extras+[
        # gzserver 直接加载现场生成的 world 文件；加载 ROS 初始化与实体工厂两个系统插件。
        # on_exit=Shutdown：用户关掉 Gazebo 后，整个 launch（含各节点）一起退出。
        ExecuteProcess(cmd=['gzserver',world_path,'--verbose',
                            '-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so'],
                       additional_env=gazebo_env,output='screen',on_exit=Shutdown()),
        # GUI 客户端可选（gui:=false 用于无头验证）。
        ExecuteProcess(cmd=['gzclient','--verbose','--gui-client-plugin=libgazebo_ros_eol_gui.so'],
                       additional_env=gazebo_env,output='screen',
                       condition=IfCondition(LaunchConfiguration('gui'))),
        # 把同一份 URDF 字符串喂给 robot_state_publisher，保证 TF 与 Gazebo 实体一致。
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':description,'use_sim_time':True}],output='screen'),
        # 用 spawn_entity.py 把机器人插到解析计算出的出生位姿（x y z R P Y）。
        Node(package='gazebo_ros',executable='spawn_entity.py',arguments=['-entity','climb_robot','-file',str(Path(output,'robot.urdf')),
            '-x',str(pose[0]),'-y',str(pose[1]),'-z',str(pose[2]),'-R',str(pose[3]),'-P',str(pose[4]),'-Y',str(pose[5]),'-timeout','120'],output='screen')]


def generate_launch_description():
    """只声明 launch 参数并挂上 OpaqueFunction；真正的构建在 setup(context) 里完成。"""
    return LaunchDescription([
        # config：集中配置文件路径（默认指向本包安装后的 simulation.yaml）。
        DeclareLaunchArgument('config',default_value=str(Path(get_package_share_directory('climb_robot_bringup'))/'config/simulation.yaml')),
        # world：场景模式。hemisphere=下半球开口，sphere=完整球壳，ground=普通地面。
        DeclareLaunchArgument('world',default_value='hemisphere',choices=['hemisphere','sphere','ground']),
        DeclareLaunchArgument('arm',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('camera',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('moveit',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('rviz',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('gui',default_value='true',choices=['true','false']),
        # spawn_angle_deg：出生角，从球底沿 +x 量起，0=球底，90=竖直内壁，180=顶端。
        DeclareLaunchArgument('spawn_angle_deg',default_value='0'),
        # 延迟回调：此刻才拥有 context，可以 perform() 取出上面的参数值。
        OpaqueFunction(function=setup)])
