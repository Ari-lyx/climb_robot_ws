"""Build MoveIt settings from the SAME generated robot description as Gazebo."""
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml
JOINTS=['arm_'+j for j in ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']]
def make_moveit(description,initial):
    urdf=ET.fromstring(description)
    srdf=ET.Element('robot',name=urdf.attrib['name'])
    # Track the mobile base through the existing world -> base_link TF. This
    # joint is outside the arm group and is never commanded by MoveIt.
    ET.SubElement(srdf,'virtual_joint',name='world_joint',type='floating',parent_frame='world',child_link='base_link')
    group=ET.SubElement(srdf,'group',name='arm')
    ET.SubElement(group,'chain',base_link='arm_base_link',tip_link='arm_tool0')
    for name,q in [('ready',initial),('inspect',[initial[0]+.3,*initial[1:]])]:
        state=ET.SubElement(srdf,'group_state',name=name,group='arm')
        for joint,value in zip(JOINTS,q):ET.SubElement(state,'joint',name=joint,value=str(value))
    pairs=set()
    # Adjacent links and all links rigidly attached to one another cannot collide.
    components={l.attrib['name']:{l.attrib['name']} for l in urdf.findall('link')}
    for j in urdf.findall('joint'):
        p=j.find('parent').attrib['link'];c=j.find('child').attrib['link'];pairs.add(tuple(sorted((p,c))))
        if j.attrib['type']=='fixed':
            union=components[p]|components[c]
            for l in union:components[l]=union
        elif not j.attrib['name'].startswith('arm_'):
            ET.SubElement(srdf,'passive_joint',name=j.attrib['name'])
    for comp in components.values():
        for a in comp:
            for b in comp:
                if a!=b:pairs.add(tuple(sorted((a,b))))
    for a,b in [('arm_wrist_1_link','arm_wrist_3_link'),('arm_tool0','arm_wrist_1_link'),('arm_tool0','arm_wrist_2_link')]:pairs.add(tuple(sorted((a,b))))
    # Camera mount sits against wrist_3, separated only by rigid tool frames.
    for a,b in sorted(pairs):ET.SubElement(srdf,'disable_collisions',link1=a,link2=b,reason='Adjacent')
    limits={j:{'has_velocity_limits':True,'max_velocity':.5,'has_acceleration_limits':True,'max_acceleration':.5} for j in JOINTS}
    config={
        'robot_description':description,'robot_description_semantic':ET.tostring(srdf,encoding='unicode'),
        'robot_description_kinematics':{'arm':{'kinematics_solver':'kdl_kinematics_plugin/KDLKinematicsPlugin','kinematics_solver_search_resolution':.005,'kinematics_solver_timeout':.1}},
        'robot_description_planning':{'joint_limits':limits},
        'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl',
        'ompl':{'planning_plugin':'ompl_interface/OMPLPlanner','request_adapters':'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints','start_state_max_bounds_error':.1,'planner_configs':{'RRTConnectkConfigDefault':{'type':'geometric::RRTConnect','range':0.0}},'arm':{'planner_configs':['RRTConnectkConfigDefault']}},
        'moveit_controller_manager':'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager':{'controller_names':['arm_controller'],'arm_controller':{'type':'FollowJointTrajectory','action_ns':'follow_joint_trajectory','default':True,'joints':JOINTS}},
        'trajectory_execution.allowed_execution_duration_scaling':2.0,'trajectory_execution.allowed_goal_duration_margin':5.0,
        'trajectory_execution.allowed_start_tolerance':.03,
        'publish_robot_description_semantic':True,'publish_planning_scene':True,'publish_geometry_updates':True,'publish_state_updates':True,'publish_transforms_updates':True,
        'use_sim_time':True,
    }
    return config
