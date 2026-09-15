"""程序化生成“闭合、绕向一致”的球壳网格，并写出完整的 Gazebo 世界文件。

这个包没有 .world 静态文件，也没有在线模型库：世界（球罐 + 地面 + 光照 + 物理参数）
在每次 launch 时由这里的 Python 代码现算现写。这样做的好处是：
  * 球罐半径/壁厚/网格细分/球心高度只写在 YAML 里，几何随之重建，不会出现
    “SDF 里的值和 URDF 里的值不一致”；
  * 碰撞网格是真实的带厚度壳体（内表面 + 外表面 + 开口封边），不是无限薄面，
    机器人不会从边缘或背面穿模；
  * 不依赖网络资源，离线可运行。

约定：球心在原点、开口朝下（z 轴向上），曲面参数 theta 从底极点量起，
`hemisphere=True` 时只画 theta∈[0, π/2]，即下半球。
"""
import math
from pathlib import Path
from xml.sax.saxutils import escape


def shell_mesh(path, thickness_ratio, hemisphere=True, latitudes=96, longitudes=192):
    """生成球壳 STL。半径按 1 归一化，实际半径通过 SDF 的 <scale> 缩放。

    参数:
        path:            STL 输出路径
        thickness_ratio: 壁厚 / 半径（相对值，缩放后自动保持不变）
        hemisphere:      True=下半球（开口），False=完整球壳
        latitudes:       纬度（theta）细分，控制沿极角方向的精度
        longitudes:      经度（phi）细分，控制圆周方向的精度
    返回:
        (vertices, faces) 便于测试或后续使用
    """
    # theta is measured from the bottom pole. The opening is z=0.
    extent = math.pi / 2 if hemisphere else math.pi
    vertices, faces = [], []
    def surface(radius, reverse):
        """构造一层球面（半径 radius）。

        reverse=True 表示该层是内表面，需要翻转三角形绕向，这样内外表面法线
        都指向实体外侧（外表面朝外、内表面朝球心），Gazebo/ODE 才能正确判碰撞。
        """
        # 底极点：单独一个顶点，避免在极点处退化。
        bottom = len(vertices)
        vertices.append((0., 0., -radius))
        rings = []
        last = latitudes + 1 if hemisphere else latitudes
        # 逐纬圈生成顶点。注意：用参数化坐标而不是笛卡尔近似，保证严格闭合。
        for i in range(1, last):
            theta = extent * i / latitudes
            ring = []
            for j in range(longitudes):
                phi = 2 * math.pi * j / longitudes
                ring.append(len(vertices))
                vertices.append((radius*math.sin(theta)*math.cos(phi), radius*math.sin(theta)*math.sin(phi), -radius*math.cos(theta)))
            rings.append(ring)
        def tri(a,b,c):
            """写三角形；reverse 时交换 b/c 实现绕向翻转。"""
            faces.append((a,c,b) if reverse else (a,b,c))
        # 底部极点扇形
        for j in range(longitudes):
            k=(j+1)%longitudes
            tri(bottom,rings[0][k],rings[0][j])
        # 相邻纬圈之间的四边形，拆成两个三角形（k = j+1 取模形成环向闭合并接回起点）
        for lower,upper in zip(rings,rings[1:]):
            for j in range(longitudes):
                k=(j+1)%longitudes
                tri(lower[j],lower[k],upper[k]);tri(lower[j],upper[k],upper[j])
        # 完整球壳才需要顶极点扇形
        if not hemisphere:
            top=len(vertices);vertices.append((0.,0.,radius))
            for j in range(longitudes):
                tri(rings[-1][j],rings[-1][(j+1)%longitudes],top)
        return rings[-1]
    # 先内表面（半径 1，绕向翻转），再外表面（半径 1 + 厚度比）。
    inner=surface(1.0, True)
    outer=surface(1.0+thickness_ratio, False)
    # 半球是开口壳体，必须把开口环（z=0 那一圈）封起来，
    # 否则机器人会从“零厚度”的边缘缝隙穿进去；这里连接内外两层，形成实体壁厚。
    if hemisphere:
        for j in range(longitudes):
            k=(j+1)%longitudes
            faces.extend([(inner[j],outer[j],outer[k]),(inner[j],outer[k],inner[k])])
    # ASCII STL is directly supported by Gazebo/Assimp; normals from winding.
    # 中文说明：手写 ASCII STL（而不是用 meshio/trimesh 依赖），法线由顶点绕向推出。
    with Path(path).open('w') as out:
        out.write('solid tank_shell\n')
        for ids in faces:
            a,b,c=[vertices[i] for i in ids]
            # 叉积求面法线并归一化，保证与绕向一致。
            u=[b[i]-a[i] for i in range(3)];v=[c[i]-a[i] for i in range(3)]
            n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
            length=math.sqrt(sum(x*x for x in n));n=[x/length for x in n]
            out.write('facet normal '+' '.join(map(str,n))+'\nouter loop\n')
            for vertex in (a,b,c):out.write('vertex '+' '.join(f'{x:.10g}' for x in vertex)+'\n')
            out.write('endloop\nendfacet\n')
        out.write('endsolid tank_shell\n')
    return vertices,faces


def generate_world(directory, config, mode):
    """生成世界文件与世界资源，返回 .world 的绝对路径。

    mode: 'hemisphere' 下半球开口 / 'sphere' 完整球壳 / 'ground' 纯地面
    """
    if mode not in ('hemisphere','sphere','ground'):
        raise ValueError('world must be hemisphere, sphere or ground')
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    tank=config['tank'];physics=config['physics']
    # 全部转成 float：YAML 里可能写成整数，SDF 里需要一致的小数形式。
    r=float(tank['radius']);thickness=float(tank['thickness']);center=float(tank['center_z'])
    # 物理自洽性检查：半径/壁厚为正，且壳体外侧不能穿到地面以下。
    if not all(math.isfinite(v) for v in (r,thickness,center)) or r<=0 or thickness<=0 or center<r+thickness:
        raise ValueError('tank radius/thickness must be positive and shell must clear the ground')
    lat=int(tank['latitude_segments']);lon=int(tank['longitude_segments'])
    # 细分过少会让圆柱轮“卡进”三角面，这里给下限保护。
    if lat<16 or lon<32:raise ValueError('tank mesh needs >=16 latitude and >=32 longitude segments')
    shell=''
    if mode!='ground':
        mesh=directory/'tank.stl'
        # 只生成一次几何，两种球壳共用一个文件，靠 hemisphere 开关控制是否封顶。
        shell_mesh(mesh,thickness/r,mode=='hemisphere',lat,lon)
        # 归一化网格 + <scale> = 真实半径，因此壁厚比在缩放后仍保持一致。
        geometry=f'<geometry><mesh><uri>{escape(mesh.as_uri())}</uri><scale>{r} {r} {r}</scale></mesh></geometry>'
        # 静态 (static) 球罐：作为刚性环境，其反力由环境承担，符合 docs/requirements.md 的约定。
        # 碰撞名 hemisphere_shell / sphere_shell 是磁吸插件识别“壁面类型”的约定名。
        shell=f'''<model name="tank"><static>true</static><pose>0 0 {center} 0 0 0</pose><link name="shell">
          <collision name="{mode}_shell">{geometry}<surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction><contact><ode><kp>200000</kp><kd>200</kd><max_vel>0.05</max_vel><min_depth>0.0005</min_depth></ode></contact></surface></collision>
          <visual name="shell_visual">{geometry}<material><ambient>0.45 0.52 0.6 1</ambient><diffuse>0.55 0.62 0.7 1</diffuse><specular>0.2 0.2 0.2 1</specular></material></visual>
        </link></model>'''
    output=directory/'climb.world'
    # 物理参数同样来自 YAML：步长/实时倍率/求解迭代直接影响爬壁稳定性。
    output.write_text(f'''<?xml version="1.0"?><sdf version="1.6"><world name="climb_world">
      <gravity>0 0 -9.81</gravity>
      <physics name="ode" type="ode"><max_step_size>{physics['step_size']}</max_step_size><real_time_update_rate>{physics['update_rate']}</real_time_update_rate><ode><solver><type>quick</type><iters>{physics['solver_iterations']}</iters><sor>1.0</sor></solver><constraints><cfm>0</cfm><erp>0.2</erp><contact_max_correcting_vel>0.1</contact_max_correcting_vel><contact_surface_layer>0.0005</contact_surface_layer></constraints></ode></physics>
      <scene><ambient>0.65 0.65 0.65 1</ambient><background>0.8 0.86 0.92 1</background><shadows>false</shadows></scene>
      <light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.3 0.2 -1</direction><cast_shadows>false</cast_shadows></light>
      <model name="ground_plane"><static>true</static><link name="ground"><collision name="ground"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision><visual name="ground"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry><material><ambient>0.3 0.33 0.35 1</ambient><diffuse>0.3 0.33 0.35 1</diffuse></material></visual></link></model>
      {shell}
      <gui fullscreen="0"><camera name="overview"><pose>7 -9 9 0 0.55 2.2</pose></camera></gui>
    </world></sdf>''')
    return str(output)


def spawn_pose(config, mode, angle_deg):
    """计算出生位姿：保证四轮（含轮胎轴向长度）都不与球壳穿插。

    angle_deg 从球底沿 +x 量起：0=球底，90=竖直内壁，180=顶端。
    返回 (x, y, z, roll, pitch, yaw)，直接喂给 spawn_entity.py 的 -x/-y/-z/-R/-P/-Y。
    """
    robot=config['robot'];tank=config['tank']
    r=float(tank['radius']);wr=float(robot['wheel_radius']);ww=float(robot['wheel_width'])
    axle=float(robot['axle_z']);x=float(robot['wheelbase'])/2;y=float(robot['track_width'])/2
    # 地面模式：只要轮胎底部略微离地（+2 mm 让接触求解器自然落稳）。
    if mode=='ground':return (0.,0.,wr-axle+0.002,0.,0.,0.)
    # Solve exact farthest radial point on the cylinder at the bottom.
    # Four symmetric wheels share the same clearance before rigid rotation.
    # 中文说明：对每个圆柱轮求“球面上最靠外的点”，取最紧的间隙，
    # 再由间隙反推车体中心到球心的距离 base_radial（车体径向外移 axle 偏移）。
    radial=math.sqrt((math.sqrt((r-0.002)**2-(y+ww/2)**2)-wr)**2-x*x)
    base_radial=radial+axle
    angle=math.radians(float(angle_deg))
    # 半球开口附近不能出生：留出车体边缘余量 asin((x+wr)/r)，否则会悬在开口外。
    edge_margin=math.asin((x+wr)/r)
    max_angle=math.pi/2-edge_margin if mode=='hemisphere' else math.pi
    if not 0<=angle<=max_angle:
        raise ValueError(f'spawn angle must be between 0 and {math.degrees(max_angle):.1f} degrees for {mode}')
    # 把“沿球面角位置”换算成世界坐标，并让车体绕 y 轴俯仰 -angle：
    # 使车体 z 轴指向球心方向，轮胎法向贴着内壁，重力自然把车压向壁面。
    return (base_radial*math.sin(angle),0.,float(tank['center_z'])-base_radial*math.cos(angle),0.,-angle,0.)
