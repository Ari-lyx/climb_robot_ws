"""Generate closed, consistently wound shell meshes without network assets."""
import math
from pathlib import Path
from xml.sax.saxutils import escape


def shell_mesh(path, thickness_ratio, hemisphere=True, latitudes=96, longitudes=192):
    # theta is measured from the bottom pole. The opening is z=0.
    extent = math.pi / 2 if hemisphere else math.pi
    vertices, faces = [], []
    def surface(radius, reverse):
        bottom = len(vertices)
        vertices.append((0., 0., -radius))
        rings = []
        last = latitudes + 1 if hemisphere else latitudes
        for i in range(1, last):
            theta = extent * i / latitudes
            ring = []
            for j in range(longitudes):
                phi = 2 * math.pi * j / longitudes
                ring.append(len(vertices))
                vertices.append((radius*math.sin(theta)*math.cos(phi), radius*math.sin(theta)*math.sin(phi), -radius*math.cos(theta)))
            rings.append(ring)
        def tri(a,b,c):
            faces.append((a,c,b) if reverse else (a,b,c))
        for j in range(longitudes):
            k=(j+1)%longitudes
            tri(bottom,rings[0][k],rings[0][j])
        for lower,upper in zip(rings,rings[1:]):
            for j in range(longitudes):
                k=(j+1)%longitudes
                tri(lower[j],lower[k],upper[k]);tri(lower[j],upper[k],upper[j])
        if not hemisphere:
            top=len(vertices);vertices.append((0.,0.,radius))
            for j in range(longitudes):
                tri(rings[-1][j],rings[-1][(j+1)%longitudes],top)
        return rings[-1]
    inner=surface(1.0, True)
    outer=surface(1.0+thickness_ratio, False)
    if hemisphere:
        for j in range(longitudes):
            k=(j+1)%longitudes
            faces.extend([(inner[j],outer[j],outer[k]),(inner[j],outer[k],inner[k])])
    # ASCII STL is directly supported by Gazebo/Assimp; normals from winding.
    with Path(path).open('w') as out:
        out.write('solid tank_shell\n')
        for ids in faces:
            a,b,c=[vertices[i] for i in ids]
            u=[b[i]-a[i] for i in range(3)];v=[c[i]-a[i] for i in range(3)]
            n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
            length=math.sqrt(sum(x*x for x in n));n=[x/length for x in n]
            out.write('facet normal '+' '.join(map(str,n))+'\nouter loop\n')
            for vertex in (a,b,c):out.write('vertex '+' '.join(f'{x:.10g}' for x in vertex)+'\n')
            out.write('endloop\nendfacet\n')
        out.write('endsolid tank_shell\n')
    return vertices,faces


def generate_world(directory, config, mode):
    if mode not in ('hemisphere','sphere','ground'):
        raise ValueError('world must be hemisphere, sphere or ground')
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    tank=config['tank'];physics=config['physics']
    r=float(tank['radius']);thickness=float(tank['thickness']);center=float(tank['center_z'])
    if not all(math.isfinite(v) for v in (r,thickness,center)) or r<=0 or thickness<=0 or center<r+thickness:
        raise ValueError('tank radius/thickness must be positive and shell must clear the ground')
    lat=int(tank['latitude_segments']);lon=int(tank['longitude_segments'])
    if lat<16 or lon<32:raise ValueError('tank mesh needs >=16 latitude and >=32 longitude segments')
    shell=''
    if mode!='ground':
        mesh=directory/'tank.stl'
        shell_mesh(mesh,thickness/r,mode=='hemisphere',lat,lon)
        geometry=f'<geometry><mesh><uri>{escape(mesh.as_uri())}</uri><scale>{r} {r} {r}</scale></mesh></geometry>'
        shell=f'''<model name="tank"><static>true</static><pose>0 0 {center} 0 0 0</pose><link name="shell">
          <collision name="{mode}_shell">{geometry}<surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction><contact><ode><kp>200000</kp><kd>200</kd><max_vel>0.05</max_vel><min_depth>0.0005</min_depth></ode></contact></surface></collision>
          <visual name="shell_visual">{geometry}<material><ambient>0.45 0.52 0.6 1</ambient><diffuse>0.55 0.62 0.7 1</diffuse><specular>0.2 0.2 0.2 1</specular></material></visual>
        </link></model>'''
    output=directory/'climb.world'
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
    """Clear all finite cylinders, including their axial extent, at spawn."""
    robot=config['robot'];tank=config['tank']
    r=float(tank['radius']);wr=float(robot['wheel_radius']);ww=float(robot['wheel_width'])
    axle=float(robot['axle_z']);x=float(robot['wheelbase'])/2;y=float(robot['track_width'])/2
    if mode=='ground':return (0.,0.,wr-axle+0.002,0.,0.,0.)
    # Solve exact farthest radial point on the cylinder at the bottom.
    # Four symmetric wheels share the same clearance before rigid rotation.
    radial=math.sqrt((math.sqrt((r-0.002)**2-(y+ww/2)**2)-wr)**2-x*x)
    base_radial=radial+axle
    angle=math.radians(float(angle_deg))
    edge_margin=math.asin((x+wr)/r)
    max_angle=math.pi/2-edge_margin if mode=='hemisphere' else math.pi
    if not 0<=angle<=max_angle:
        raise ValueError(f'spawn angle must be between 0 and {math.degrees(max_angle):.1f} degrees for {mode}')
    return (base_radial*math.sin(angle),0.,float(tank['center_z'])-base_radial*math.cos(angle),0.,-angle,0.)
