# Generate the chassis mesh of the UTV utility trailer (utv_trailer_chassis.obj and .mtl).
#
# Usage: blender -b --python make_utv_trailer_chassis.py [-- <out_dir>]
#
# The trailer chassis reference frame (Z up, X forward) is at the axle center. The coupler socket is centered on the
# hitch ball, at the "Front Connector Location" in UTV_TrailerChassis.json. Materials use the names and shading of the
# Polaris meshes. Tested with Blender 5.1.
import bpy, bmesh, sys, os, re, math
from mathutils import Vector, Matrix

# Output directory: the directory of this script, unless given after '--'
argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
out_dir = argv[0] if argv else os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------------------------------------------------
# Dimensions
TRACK = 1.232        # same as the Polaris (spindles at y = +-0.616)
RAIL_Y = 0.42        # rail centerline
RAIL_W, RAIL_H = 0.05, 0.10
RAIL_Z0 = 0.12       # rail bottom (above axle center)
X_REAR, X_FRONT = -1.0, 1.0
DECK_T = 0.012
HITCH = Vector((2.0, 0.0, 0.05))   # ball center (front connector location)
FENDER_R = 0.40
FENDER_W = 0.28

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


def fender(name, yc, mat):
    """Curved fender shell over the wheel at lateral position yc, with a lip on the outer edge."""
    bm = bmesh.new()
    a0, a1, n = math.radians(10), math.radians(170), 40
    t = 0.004
    rows = []
    for i in range(n + 1):
        a = a0 + (a1 - a0) * i / n
        ring = []
        for r in (FENDER_R, FENDER_R + t):
            for y in (yc - FENDER_W / 2, yc + FENDER_W / 2):
                ring.append(bm.verts.new((r * math.cos(a), y, r * math.sin(a))))
        rows.append(ring)  # [in-, in+, out-, out+]
    for i in range(n):
        a, b = rows[i], rows[i + 1]
        bm.faces.new((a[2], a[3], b[3], b[2]))  # outer
        bm.faces.new((a[0], b[0], b[1], a[1]))  # inner
        bm.faces.new((a[0], a[2], b[2], b[0]))  # side -
        bm.faces.new((a[1], b[1], b[3], a[3]))  # side +
    for ring, flip in ((rows[0], False), (rows[-1], True)):
        f = (ring[0], ring[1], ring[3], ring[2])
        bm.faces.new(f[::-1] if flip else f)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    o = new_obj(name, bm)
    return finish(o, mat, smooth=True)


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


bpy.ops.wm.read_factory_settings(use_empty=True)

# ---------------------------------------------------------------------------------------------------------------------
# Trailer chassis
T = []
zr0, zr1 = RAIL_Z0, RAIL_Z0 + RAIL_H
zc = (zr0 + zr1) / 2
for s in (-1, 1):
    T.append(box('rail', (X_REAR, s * RAIL_Y - RAIL_W / 2, zr0), (X_FRONT, s * RAIL_Y + RAIL_W / 2, zr1), 'Frame_Suspension'))
for x in (-0.975, -0.5, 0.0, 0.5, 0.975):
    T.append(box('xmember', (x - 0.025, -RAIL_Y + RAIL_W / 2, zr0 + 0.01), (x + 0.025, RAIL_Y - RAIL_W / 2, zr1 - 0.01), 'Frame_Suspension'))
# deck
T.append(box('deck', (X_REAR - 0.01, -0.46, zr1), (X_FRONT + 0.01, 0.46, zr1 + DECK_T), 'Interior', bevel=0.004))
# low side rails around deck edge
for s in (-1, 1):
    T.append(box('deck_edge', (X_REAR - 0.01, s * 0.46 - 0.015, zr1 - 0.04), (X_FRONT + 0.01, s * 0.46 + 0.015, zr1 + DECK_T + 0.01), 'Frame_Suspension'))
# tongue (A-frame); the arms end inside the coupler body
tz0 = zc
tz1 = HITCH.z + 0.075
for s in (-1, 1):
    T.append(bar('tongue', (X_FRONT - 0.05, s * (RAIL_Y - 0.03), tz0), (HITCH.x - 0.20, s * 0.03, tz1), 0.05, 0.08, 'Frame_Suspension'))
# coupler: body + socket hood over the ball
T.append(box('coupler', (HITCH.x - 0.30, -0.06, HITCH.z + 0.025), (HITCH.x - 0.02, 0.06, HITCH.z + 0.125), 'Steel', bevel=0.006))
T.append(sphere('socket', (HITCH.x, 0, HITCH.z + 0.025), 0.05, 'Steel', scale=(1.0, 0.95, 0.9)))
T.append(box('latch', (HITCH.x - 0.17, -0.012, HITCH.z + 0.125), (HITCH.x - 0.04, 0.012, HITCH.z + 0.15), 'Black', bevel=0.004))
# jack (raised) with caster foot
jx, jy = 1.35, 0.20
T.append(box('jack_mount', (jx - 0.05, 0.03, tz0 - 0.03), (jx + 0.05, jy, tz0 + 0.03), 'Frame_Suspension'))
T.append(cyl('jack_outer', (jx, jy, -0.02), (jx, jy, 0.46), 0.03, 'Black'))
T.append(cyl('jack_inner', (jx, jy, -0.16), (jx, jy, -0.02), 0.022, 'Steel'))
T.append(box('jack_fork', (jx - 0.04, jy - 0.035, -0.18), (jx + 0.04, jy + 0.035, -0.155), 'Steel'))
T.append(cyl('jack_wheel', (jx, jy - 0.025, -0.215), (jx, jy + 0.025, -0.215), 0.055, 'Black'))
T.append(cyl('jack_crank', (jx, jy, 0.46), (jx + 0.10, jy, 0.46), 0.008, 'Steel', segs=12))
T.append(cyl('jack_knob', (jx + 0.10, jy, 0.46), (jx + 0.10, jy, 0.52), 0.012, 'Black', segs=12))
# fenders with inner support panels
for s in (-1, 1):
    yc = s * TRACK / 2
    T.append(fender('fender', yc, 'Chassis'))
    yi = yc - s * FENDER_W / 2
    T.append(box('fender_panel', (-0.30, min(yi, yi - s * 0.004), zr1), (0.30, max(yi, yi - s * 0.004), FENDER_R - 0.02), 'Chassis', bevel=0.0))
    T.append(box('fender_brace', (-0.02, min(s * (RAIL_Y + RAIL_W / 2), yi), zr0 + 0.02), (0.02, max(s * (RAIL_Y + RAIL_W / 2), yi), zr0 + 0.06), 'Frame_Suspension'))
# rear light bar, tail lights, plate
T.append(box('lightbar', (X_REAR - 0.04, -0.42, zr0 + 0.02), (X_REAR - 0.01, 0.42, zr0 + 0.07), 'Frame_Suspension'))
for s in (-1, 1):
    T.append(box('taillight', (X_REAR - 0.06, s * 0.36 - 0.06, zr0 + 0.005), (X_REAR - 0.04, s * 0.36 + 0.06, zr0 + 0.085), 'Lights', bevel=0.005))
T.append(box('plate', (X_REAR - 0.05, -0.15, zr0 - 0.14), (X_REAR - 0.045, 0.15, zr0 - 0.0), 'Black', bevel=0.0))
T.append(box('plate_bracket', (X_REAR - 0.045, -0.02, zr0 - 0.05), (X_REAR - 0.01, 0.02, zr0 + 0.02), 'Frame_Suspension'))

obj_file = os.path.join(out_dir, 'utv_trailer_chassis.obj')
export(T, obj_file)
normalize_mtl(os.path.splitext(obj_file)[0] + '.mtl')
