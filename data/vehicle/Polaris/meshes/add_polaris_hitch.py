# Add a hitch receiver with ball mount to the Polaris chassis mesh (Polaris_chassis.obj and .mtl).
#
# Usage: blender -b --python add_polaris_hitch.py [-- <out_dir>]
#
# Generates a cross bar between the rear frame uprights, a 2" receiver, a ball mount, and a ball centered at the
# "Rear Connector Location" in Polaris_Chassis.json, and appends them to Polaris_chassis.obj as object 'Hitch'. A hitch
# appended by an earlier run is replaced, so the script can be rerun. Missing materials are added to Polaris_chassis.mtl.
# Tested with Blender 5.1.
import bpy, bmesh, sys, os, re, math, tempfile
from mathutils import Vector, Matrix

# Output directory: the directory of this script, unless given after '--'
argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
out_dir = argv[0] if argv else os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------------------------------------------------
# Hitch ball center (Polaris chassis frame)
P_BALL = Vector((-3.25, 0.0, 0.445))

MARK = '# Hitch receiver and ball mount'

MATS = {
    'Frame_Suspension': (0.80, 0.80, 0.80),
    'Chassis': (0.698, 0.616, 0.412),
    'Interior': (0.458, 0.445, 0.329),
    'Lights': (0.814, 0.140, 0.140),
    'Black': (0.04, 0.04, 0.04),
    'Steel': (0.55, 0.56, 0.58),
}


def material(name):
    m = bpy.data.materials.get(name)
    if m is None:
        m = bpy.data.materials.new(name)
        m.diffuse_color = (*MATS[name], 1.0)
        m.use_nodes = True
        bsdf = m.node_tree.nodes.get('Principled BSDF')
        bsdf.inputs['Base Color'].default_value = (*MATS[name], 1.0)
        bsdf.inputs['Roughness'].default_value = 0.6
    return m


def finish(obj, mat, bevel=0.0, smooth=False):
    obj.data.materials.append(material(mat))
    if bevel > 0:
        mod = obj.modifiers.new('bevel', 'BEVEL')
        mod.width = bevel
        mod.segments = 2
        mod.limit_method = 'ANGLE'
    if smooth:
        for p in obj.data.polygons:
            p.use_smooth = True
    return obj


def new_obj(name, bm):
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    obj = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def box(name, mn, mx, mat, bevel=0.003):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    c = (Vector(mn) + Vector(mx)) / 2
    s = Vector(mx) - Vector(mn)
    bmesh.ops.scale(bm, vec=s, verts=bm.verts)
    bmesh.ops.translate(bm, vec=c, verts=bm.verts)
    return finish(new_obj(name, bm), mat, bevel)


def bar(name, p0, p1, w, h, mat, bevel=0.003):
    """Rectangular bar from p0 to p1 (w: horizontal width, h: height of the cross section)."""
    p0, p1 = Vector(p0), Vector(p1)
    d = p1 - p0
    L = d.length
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.scale(bm, vec=(L, w, h), verts=bm.verts)
    x = d.normalized()
    y = Vector((0, 0, 1)).cross(x).normalized()
    z = x.cross(y)
    R = Matrix((x, y, z)).transposed().to_4x4()
    bmesh.ops.transform(bm, matrix=Matrix.Translation((p0 + p1) / 2) @ R, verts=bm.verts)
    return finish(new_obj(name, bm), mat, bevel)


def cyl(name, p0, p1, r, mat, segs=32, cap=True):
    p0, p1 = Vector(p0), Vector(p1)
    d = p1 - p0
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=cap, segments=segs, radius1=r, radius2=r, depth=d.length)
    R = d.normalized().to_track_quat('Z', 'Y').to_matrix().to_4x4()
    bmesh.ops.transform(bm, matrix=Matrix.Translation((p0 + p1) / 2) @ R, verts=bm.verts)
    o = new_obj(name, bm)
    return finish(o, mat, smooth=True)


def sphere(name, c, r, mat, scale=(1, 1, 1)):
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=32, v_segments=16, radius=r)
    bmesh.ops.scale(bm, vec=scale, verts=bm.verts)
    bmesh.ops.translate(bm, vec=Vector(c), verts=bm.verts)
    return finish(new_obj(name, bm), mat, smooth=True)



def export(objs, path):
    bpy.ops.object.select_all(action='DESELECT')
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    bpy.ops.object.convert(target='MESH')  # apply modifiers
    bpy.ops.object.join()
    bpy.ops.wm.obj_export(filepath=path, export_selected_objects=True, up_axis='Z', forward_axis='Y',
                          export_materials=True, export_uv=False, export_normals=True, apply_modifiers=True,
                          path_mode='STRIP')
    joined = bpy.context.view_layer.objects.active
    bpy.data.objects.remove(joined)


def normalize_mtl(path):
    """Use the shading parameters of the Polaris meshes."""
    t = open(path).read()
    t = re.sub(r'^Ns .*$', 'Ns 96.078443', t, flags=re.M)
    t = re.sub(r'^Ka .*$', 'Ka 0.100000 0.100000 0.100000', t, flags=re.M)
    t = re.sub(r'^Ks .*$', 'Ks 0.450000 0.450000 0.450000', t, flags=re.M)
    t = re.sub(r'^Ni .*$', 'Ni 1.000000', t, flags=re.M)
    open(path, 'w', newline='\n').write(t)


def append_to_chassis(hitch_obj, hitch_mtl, chassis_obj, chassis_mtl):
    """Append the hitch to the chassis OBJ (offsetting indices) and add its materials to the chassis MTL."""
    # The chassis OBJ uses CRLF line endings, the MTL uses LF
    txt = open(chassis_obj, newline='').read()
    if MARK in txt:
        txt = txt[:txt.index(MARK)]
    nv = sum(1 for l in txt.splitlines() if l.startswith('v '))
    nn = sum(1 for l in txt.splitlines() if l.startswith('vn '))
    out = [MARK, 'o Hitch']
    for l in open(hitch_obj).read().splitlines():
        if l.startswith(('v ', 'vn ', 'usemtl', 's ')):
            out.append(l)
        elif l.startswith('f '):
            idx = []
            for t in l.split()[1:]:
                v, _, n = t.split('/')
                idx.append(f'{int(v) + nv}//{int(n) + nn}')
            out.append('f ' + ' '.join(idx))
    if not txt.endswith('\r\n'):
        txt += '\r\n'
    open(chassis_obj, 'w', newline='').write(txt + '\r\n'.join(out) + '\r\n')

    mtl = open(chassis_mtl).read()
    for b in re.split(r'\n(?=newmtl )', open(hitch_mtl).read()):
        m = re.match(r'newmtl (\S+)', b)
        if m and ('newmtl ' + m.group(1)) not in mtl:
            mtl = mtl.rstrip('\n') + '\n\n' + '\n'.join(b.strip().splitlines()) + '\n'
    count = len(re.findall(r'^newmtl ', mtl, flags=re.M))
    mtl = re.sub(r'^# Material Count: \d+', f'# Material Count: {count}', mtl, flags=re.M)
    open(chassis_mtl, 'w', newline='\n').write(mtl)


bpy.ops.wm.read_factory_settings(use_empty=True)

# ---------------------------------------------------------------------------------------------------------------------
# Hitch geometry (Polaris chassis frame)
H = []
zrcv = 0.475
H.append(bar('crossbar', (-3.03, -0.27, zrcv), (-3.03, 0.27, zrcv), 0.05, 0.05, 'Frame_Suspension'))
H.append(box('receiver', (-3.17, -0.032, zrcv - 0.032), (-2.95, 0.032, zrcv + 0.032), 'Black', bevel=0.004))
H.append(box('receiver_collar', (-3.17, -0.038, zrcv - 0.038), (-3.15, 0.038, zrcv + 0.038), 'Black', bevel=0.004))
H.append(box('shank', (-3.24, -0.025, zrcv - 0.025), (-3.10, 0.025, zrcv + 0.025), 'Steel', bevel=0.003))
H.append(cyl('pin', (-3.13, -0.05, zrcv), (-3.13, 0.05, zrcv), 0.008, 'Steel', segs=12))
H.append(box('drop', (-3.26, -0.035, P_BALL.z - 0.045), (-3.23, 0.035, zrcv + 0.025), 'Steel', bevel=0.004))
H.append(box('platform', (-3.30, -0.035, P_BALL.z - 0.045), (-3.23, 0.035, P_BALL.z - 0.025), 'Steel', bevel=0.004))
H.append(cyl('ball_neck', (P_BALL.x, 0, P_BALL.z - 0.025), (P_BALL.x, 0, P_BALL.z), 0.016, 'Steel', segs=24))
H.append(sphere('ball', P_BALL, 0.025, 'Steel'))

with tempfile.TemporaryDirectory() as tmp:
    hitch_obj = os.path.join(tmp, 'polaris_hitch.obj')
    export(H, hitch_obj)
    hitch_mtl = os.path.join(tmp, 'polaris_hitch.mtl')
    normalize_mtl(hitch_mtl)
    append_to_chassis(hitch_obj, hitch_mtl, os.path.join(out_dir, 'Polaris_chassis.obj'), os.path.join(out_dir, 'Polaris_chassis.mtl'))
