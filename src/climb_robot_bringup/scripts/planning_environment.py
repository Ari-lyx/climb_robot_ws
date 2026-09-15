#!/usr/bin/env python3
"""Install static tank/ground collision geometry once, in world coordinates.

The base is not a MoveIt planning DOF: move the arm only while parked. Real wheel
contacts are allowed explicitly; tank and ground remain obstacles for the arm.
"""
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose,Point
from shape_msgs.msg import Mesh,MeshTriangle,SolidPrimitive
from moveit_msgs.msg import PlanningScene,CollisionObject,AllowedCollisionEntry,PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene,ApplyPlanningScene

class Environment(Node):
    def __init__(self):
        super().__init__('planning_environment')
        self.declare_parameter('world_mesh','');self.declare_parameter('radius',3.0);self.declare_parameter('center_z',3.15)
        self.stage='waiting';self.future=None
        self.get=self.create_client(GetPlanningScene,'/get_planning_scene');self.apply=self.create_client(ApplyPlanningScene,'/apply_planning_scene')
        self.mesh=None;path=self.get_parameter('world_mesh').value
        if path:
            self.mesh=Mesh();indices={};face=[];scale=self.get_parameter('radius').value
            for line in Path(path).read_text().splitlines():
                if line.lstrip().startswith('vertex '):
                    xyz=tuple(float(x)*scale for x in line.split()[1:])
                    if xyz not in indices:indices[xyz]=len(self.mesh.vertices);self.mesh.vertices.append(Point(x=xyz[0],y=xyz[1],z=xyz[2]))
                    face.append(indices[xyz])
                    if len(face)==3:self.mesh.triangles.append(MeshTriangle(vertex_indices=face));face=[]
        self.create_timer(.2,self.tick)
    def world_pose(self,z):
        pose=Pose();pose.position.z=float(z);pose.orientation.w=1.0
        return pose
    def objects(self):
        result=[]
        if self.mesh:
            tank=CollisionObject();tank.header.frame_id='world';tank.id='tank';tank.mesh_poses=[self.world_pose(self.get_parameter('center_z').value)]
            tank.operation=CollisionObject.ADD;tank.meshes=[self.mesh]
            result.append(tank)
        ground=CollisionObject();ground.header.frame_id='world';ground.id='ground';ground.primitive_poses=[self.world_pose(-.05)]
        ground.operation=CollisionObject.ADD
        ground.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[40.,40.,.1])]
        result.append(ground)
        return result
    def tick(self):
        if self.stage=='ready':return
        if self.stage=='waiting':
            if not self.get.service_is_ready() or not self.apply.service_is_ready():return
            req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            self.future=self.get.call_async(req);self.stage='get';return
        if self.stage=='get':
            if not self.future.done():return
            matrix=self.future.result().scene.allowed_collision_matrix
            wheels=['front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel']
            for name in ['tank','ground',*wheels]:
                if name not in matrix.entry_names:
                    matrix.entry_names.append(name)
                    for row in matrix.entry_values:row.enabled.append(False)
                    matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
            for obstacle in ('tank','ground'):
                i=matrix.entry_names.index(obstacle)
                for wheel in wheels:
                    j=matrix.entry_names.index(wheel);matrix.entry_values[i].enabled[j]=True;matrix.entry_values[j].enabled[i]=True
            scene=PlanningScene();scene.is_diff=True;scene.robot_state.is_diff=True;scene.allowed_collision_matrix=matrix;scene.world.collision_objects=self.objects()
            self.future=self.apply.call_async(ApplyPlanningScene.Request(scene=scene));self.stage='apply';return
        if self.stage=='apply':
            if not self.future.done():return
            if not self.future.result().success:self.stage='waiting';return
            self.stage='ready';self.get_logger().info('Static tank and ground installed in world frame')

def main():
    rclpy.init();node=Environment()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
