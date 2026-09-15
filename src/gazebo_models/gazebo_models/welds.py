"""球罐内壁焊缝：半椭圆视觉网格、UV 与程序化鱼鳞纹理；没有 collision。

极角 theta 从球底量起，与 world.py 一致。环缝分隔板带；纵缝仅出现在
相邻环缝之间，相邻板带可设置不同条数和角度偏移；两极保留完整封头板。
仅用 Python 标准库生成 COLLADA + PNG，不依赖在线资源或额外图像库。
"""
import math
from pathlib import Path
import struct
import zlib
from xml.sax.saxutils import escape


def _dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def _unit(v):
    length=math.sqrt(_dot(v,v))
    return tuple(x/length for x in v)


def write_weld_texture(path, width=256, height=256):
    """沿 u 周期平铺的一枚鱼鳞：弧形亮边、暗沟和细金属拉丝。

    PNG 的横向是焊缝行进方向，纵向是半椭圆截面；图像两端周期连续。
    """
    rows=[]
    for j in range(height):
        row=bytearray([0])  # PNG filter: none
        v=2*j/(height-1)-1
        for i in range(width):
            u=i/width
            phase=(u-0.36*(1-v*v)) % 1
            def ridge(center, sigma):
                d=(phase-center+0.5)%1-0.5
                return math.exp(-0.5*(d/sigma)**2)
            metal=154+27*(1-v*v)+40*ridge(.10,.065)-46*ridge(.24,.06)
            metal+=5*math.sin(2*math.pi*(u*17+v*36))+3*math.cos(2*math.pi*(u*31-v*73))
            for tint in (.91,.98,1.02):row.append(int(max(0,min(255,metal*tint))))
        rows.append(bytes(row))
    def chunk(kind, data):
        return struct.pack('!I',len(data))+kind+data+struct.pack('!I',zlib.crc32(kind+data)&0xffffffff)
    Path(path).write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',width,height,8,2,0,0,0))+
                          chunk(b'IDAT',zlib.compress(b''.join(rows)))+chunk(b'IEND',b''))


def weld_paths(mode, options):
    """返回 (名称, 类型, 固定角, 起角, 终角)，角度均为弧度。"""
    rings=[float(x) for x in options.get('ring_angles_deg',[25,70,110,155])]
    counts=options.get('meridians_per_band',[10,12,10])
    shifts=options.get('band_offsets_deg',[0,15,0])
    if len(rings)<2 or any(not math.isfinite(x) or not 0<x<180 for x in rings) or rings!=sorted(set(rings)):
        raise ValueError('weld ring_angles_deg must be strictly increasing in (0,180)')
    if len(counts)!=len(rings)-1 or len(shifts)!=len(counts):
        raise ValueError('weld meridians_per_band and band_offsets_deg need one entry per inter-ring band')
    if any(int(x)!=x or x<1 for x in counts) or any(not math.isfinite(float(x)) for x in shifts):
        raise ValueError('weld counts must be positive integers; offsets must be finite')
    limit=math.pi/2 if mode=='hemisphere' else math.pi
    angles=[math.radians(x) for x in rings]
    paths=[]
    for i,theta in enumerate(angles):
        if theta<limit-1e-8:paths.append((f'ring_{i}','ring',theta,0,2*math.pi))
    for i,(lo,hi) in enumerate(zip(angles,angles[1:])):
        hi=min(hi,limit)
        if lo>=hi:continue
        for j in range(int(counts[i])):
            phi=2*math.pi*j/counts[i]+math.radians(float(shifts[i]))
            paths.append((f'band_{i}_seam_{j}','meridian',phi,lo,hi))
    return paths


def build_weld_mesh(radius, mode, options):
    """返回顶点、平滑法线、UV、三角面和各条焊缝元数据，便于离线检查。"""
    width=float(options.get('width',.045));height=float(options.get('height',.006))
    offset=float(options.get('surface_offset',.0015));step=float(options.get('segment_length',.025))
    pitch=float(options.get('texture_pitch',.012));sides=options.get('cross_section_segments',12)
    if not all(math.isfinite(x) and x>0 for x in (radius,width,height,offset,step,pitch)):
        raise ValueError('weld geometry parameters must be finite and positive')
    if width>=radius*.1 or height+offset>=radius*.05 or int(sides)!=sides or sides<4:
        raise ValueError('weld is too large for tank or cross_section_segments is invalid')
    sides=int(sides);vertices=[];normals=[];uv=[];faces=[];metadata=[]
    for name,kind,fixed,start,end in weld_paths(mode,options):
        length=radius*(end-start)*(math.sin(fixed) if kind=='ring' else 1)
        segments=max(8,math.ceil(length/step))
        # Integer texture cycles keep the circumferential closure seamless.
        repeats=max(1,round(length/pitch)) if kind=='ring' else length/pitch
        first=len(vertices)
        for i in range(segments+1):
            a=start+(end-start)*i/segments
            theta,phi=(fixed,a) if kind=='ring' else (a,fixed)
            n=(math.sin(theta)*math.cos(phi),math.sin(theta)*math.sin(phi),-math.cos(theta))
            tangent=(-math.sin(phi),math.cos(phi),0) if kind=='ring' else (math.cos(theta)*math.cos(phi),math.cos(theta)*math.sin(phi),math.sin(theta))
            across=_cross(n,tangent)
            for j in range(sides+1):
                alpha=math.pi*j/sides
                side=width*.5*math.cos(alpha);h=height*math.sin(alpha)
                beta=side/radius
                radial=tuple(math.cos(beta)*n[k]+math.sin(beta)*across[k] for k in range(3))
                lateral=tuple(-math.sin(beta)*n[k]+math.cos(beta)*across[k] for k in range(3))
                vertices.append(tuple((radius-offset-h)*x for x in radial))
                normals.append(_unit(tuple(-radial[k]*math.sin(alpha)/height+lateral[k]*math.cos(alpha)/(width*.5) for k in range(3))))
                uv.append((i/segments*repeats,j/sides))
        for i in range(segments):
            for j in range(sides):
                a=first+i*(sides+1)+j;b=a+sides+1
                for face in ((a,b,b+1),(a,b+1,a+1)):
                    p,q,r=[vertices[k] for k in face]
                    normal=_cross(tuple(q[k]-p[k] for k in range(3)),tuple(r[k]-p[k] for k in range(3)))
                    reference=tuple(sum(normals[v][k] for v in face) for k in range(3))
                    faces.append(face if _dot(normal,reference)>0 else (face[0],face[2],face[1]))
        metadata.append({'name':name,'kind':kind,'length':length,'first_vertex':first,'vertices':(segments+1)*(sides+1)})
    return vertices,normals,uv,faces,metadata


def write_collada(path, texture_path, mesh):
    vertices,normals,uv,faces,_=mesh
    def source(name, values, params):
        data=' '.join(f'{x:.8g}' for row in values for x in row)
        return f'''<source id="{name}"><float_array id="{name}-array" count="{len(values)*len(params)}">{data}</float_array><technique_common><accessor source="#{name}-array" count="{len(values)}" stride="{len(params)}">{''.join(f'<param name="{p}" type="float"/>' for p in params)}</accessor></technique_common></source>'''
    indices=' '.join(f'{v} {v} {v}' for face in faces for v in face)
    Path(path).write_text(f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><created>2026-09-15T00:00:00Z</created><modified>2026-09-15T00:00:00Z</modified><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_images><image id="weld-image"><init_from>{escape(Path(texture_path).resolve().as_uri())}</init_from></image></library_images>
<library_effects><effect id="weld-effect"><profile_COMMON>
<newparam sid="weld-surface"><surface type="2D"><init_from>weld-image</init_from></surface></newparam>
<newparam sid="weld-sampler"><sampler2D><source>weld-surface</source><wrap_s>WRAP</wrap_s><wrap_t>CLAMP</wrap_t></sampler2D></newparam>
<technique sid="common"><phong><ambient><color>0.42 0.46 0.5 1</color></ambient><diffuse><texture texture="weld-sampler" texcoord="UVSET0"/></diffuse><specular><color>0.45 0.48 0.5 1</color></specular><shininess><float>35</float></shininess></phong></technique>
</profile_COMMON></effect></library_effects>
<library_materials><material id="weld-material" name="SilverWeld"><instance_effect url="#weld-effect"/></material></library_materials>
<library_geometries><geometry id="weld-geometry"><mesh>
{source('positions',vertices,('X','Y','Z'))}{source('normals',normals,('X','Y','Z'))}{source('uv',uv,('S','T'))}
<vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
<triangles count="{len(faces)}" material="weld-symbol"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="NORMAL" source="#normals" offset="1"/><input semantic="TEXCOORD" source="#uv" offset="2" set="0"/><p>{indices}</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="Scene"><node id="Welds"><instance_geometry url="#weld-geometry"><bind_material><technique_common><instance_material symbol="weld-symbol" target="#weld-material"><bind_vertex_input semantic="UVSET0" input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#Scene"/></scene></COLLADA>''')


def generate_weld_visual(directory, radius, mode, options):
    """返回可插入 tank/shell link 的 SDF visual 片段，位置相对球心。"""
    if mode=='ground' or not options.get('enabled',True):return ''
    directory=Path(directory)
    texture=directory/'weld_scales.png';dae=directory/'welds.dae'
    write_weld_texture(texture)
    write_collada(dae,texture,build_weld_mesh(radius,mode,options))
    return f'''<visual name="inner_welds"><cast_shadows>false</cast_shadows><geometry><mesh><uri>{escape(dae.resolve().as_uri())}</uri></mesh></geometry></visual>'''
