# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2026 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
#
# Render the output of demo_FSI-SPH_TankTrailer (vehicle mode, run with --blender) in Blender with Cycles.
#
# Requires the fluid surfaces created by make_fluid_surfaces.py (surfaces/fluid_NNNNN.ply in the run directory).
#
# Usage:
#   blender -b -P render_tank_trailer.py -- <run_dir> [--camera chase|side] [--first 0] [--last -1] [--step 1]
#                                            [--frames F1 F2 ...] [--samples 128] [--res 1920 1080]
#                                            [--out-dir DIR] [--no-cues] [--save-blend]
#
# The scene (vehicle, trailer, tank, terrain) is loaded with the Chrono::Blender add-on (chrono_import.py from
# src/importer_blender), which reloads the object states at each frame. A frame-change handler appended after the
# add-on's own handler loads the fluid surface of the current frame and places it in the tank, using the offset of the
# frame translating with the tank (fluid_offset.txt). Unless --no-cues is given, lane markings and traffic cones along
# the double lane change of the demo are added to the scene. Frames are rendered to <out_dir>/<camera>_NNNNN.png
# (default <out_dir>: <run_dir>/render).
#
# See README.md for the complete pipeline.

import argparse, os, re, sys, importlib.util
import numpy as np
import bpy
import mathutils
import bmesh

# Chrono::Blender add-on, from the Chrono source tree containing this script (src/importer_blender)
CHRONO_IMPORT = os.environ.get(
    'CHRONO_BLENDER_IMPORTER',
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', 'importer_blender', 'for_blender_5.0',
                 'chrono_import.py'))

argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
ap = argparse.ArgumentParser()
ap.add_argument('run_dir')
ap.add_argument('--camera', default='chase', choices=['chase', 'side'], help='camera following the tank')
ap.add_argument('--first', type=int, default=0, help='first frame')
ap.add_argument('--last', type=int, default=-1, help='last frame (default: last frame of the run)')
ap.add_argument('--step', type=int, default=1, help='frame increment')
ap.add_argument('--frames', type=int, nargs='+', help='render only these frames (overrides --first, --last, --step)')
ap.add_argument('--samples', type=int, default=128, help='Cycles samples per pixel')
ap.add_argument('--res', type=int, nargs=2, default=[1920, 1080], help='image resolution')
ap.add_argument('--out-dir', help='output directory for the frames (default: <run_dir>/render)')
ap.add_argument('--no-cues', action='store_true', help='no lane markings and cones')
ap.add_argument('--save-blend', action='store_true', help='also save the scene (at the first frame) to <run_dir>/tank_trailer.blend')
args = ap.parse_args(argv)

run_dir = os.path.abspath(args.run_dir)
blender_dir = os.path.join(run_dir, 'blender')
render_dir = os.path.abspath(args.out_dir) if args.out_dir else os.path.join(run_dir, 'render')
os.makedirs(render_dir, exist_ok=True)

# Fluid frame offsets (frame -> translation of the fluid surface into the world)
offsets = {}
with open(os.path.join(run_dir, 'fluid_offset.txt')) as f:
    for line in f:
        if line.startswith('#'):
            continue
        v = line.split()
        offsets[int(v[0])] = mathutils.Vector((float(v[2]), float(v[3]), float(v[4])))
num_frames = len(offsets)
last = num_frames - 1 if args.last < 0 else args.last

# -----------------------------------------------------------------------------
# Load the Chrono::Blender add-on and import the simulation

if not os.path.isfile(CHRONO_IMPORT):
    sys.exit(f'Chrono::Blender add-on not found: {CHRONO_IMPORT} (set CHRONO_BLENDER_IMPORTER)')

bpy.ops.wm.read_factory_settings(use_empty=True)
spec = importlib.util.spec_from_file_location('chrono_import', CHRONO_IMPORT)
chrono_import = importlib.util.module_from_spec(spec)
spec.loader.exec_module(chrono_import)
chrono_import.register()
bpy.ops.import_chrono.data(filepath=os.path.join(blender_dir, 'exported.assets.py'))
scene = bpy.context.scene

# -----------------------------------------------------------------------------
# Materials

def principled(name):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    return mat, mat.node_tree.nodes.get('Principled BSDF')


def set_thin_glass(mat):
    """Thin glass shell: Fresnel-weighted mix of transparency and reflection, without refraction (the tank is
    modeled as a solid cylinder, which would otherwise act as a lens)."""
    mat.use_nodes = True
    nt = mat.node_tree
    nt.nodes.clear()
    transparent = nt.nodes.new('ShaderNodeBsdfTransparent')
    transparent.inputs['Color'].default_value = (0.97, 0.98, 1.0, 1.0)
    glossy = nt.nodes.new('ShaderNodeBsdfGlossy')
    glossy.inputs['Roughness'].default_value = 0.03
    fresnel = nt.nodes.new('ShaderNodeFresnel')
    fresnel.inputs['IOR'].default_value = 1.5
    mix = nt.nodes.new('ShaderNodeMixShader')
    out = nt.nodes.new('ShaderNodeOutputMaterial')
    scale = nt.nodes.new('ShaderNodeMath')
    scale.operation = 'MULTIPLY'
    scale.inputs[1].default_value = 0.5
    nt.links.new(fresnel.outputs['Fac'], scale.inputs[0])
    nt.links.new(scale.outputs['Value'], mix.inputs['Fac'])
    nt.links.new(transparent.outputs['BSDF'], mix.inputs[1])
    nt.links.new(glossy.outputs['BSDF'], mix.inputs[2])
    nt.links.new(mix.outputs['Shader'], out.inputs['Surface'])


def color_socket(sockets, name):
    """Color socket of a Mix node (it has float, vector and color sockets with the same name)."""
    return next(x for x in sockets if x.name == name and x.type == 'RGBA')


def set_asphalt(mat):
    mat.use_nodes = True
    nt = mat.node_tree
    nt.nodes.clear()
    bsdf = nt.nodes.new('ShaderNodeBsdfPrincipled')
    out = nt.nodes.new('ShaderNodeOutputMaterial')
    coord = nt.nodes.new('ShaderNodeTexCoord')
    noise = nt.nodes.new('ShaderNodeTexNoise')
    noise.inputs['Scale'].default_value = 0.6
    noise.inputs['Detail'].default_value = 12
    ramp = nt.nodes.new('ShaderNodeValToRGB')
    ramp.color_ramp.elements[0].color = (0.035, 0.035, 0.038, 1)
    ramp.color_ramp.elements[1].color = (0.11, 0.105, 0.10, 1)
    bump = nt.nodes.new('ShaderNodeBump')
    bump.inputs['Strength'].default_value = 0.15
    fine = nt.nodes.new('ShaderNodeTexNoise')
    fine.inputs['Scale'].default_value = 400
    nt.links.new(coord.outputs['Object'], noise.inputs['Vector'])
    nt.links.new(coord.outputs['Object'], fine.inputs['Vector'])
    nt.links.new(noise.outputs['Fac'], ramp.inputs['Fac'])
    # Aggregate speckle (light stones, ~2-3 cm) and darker patches (~1 m): texture that shows ground motion
    speckle = nt.nodes.new('ShaderNodeTexVoronoi')
    speckle.inputs['Scale'].default_value = 40
    speckle.inputs['Randomness'].default_value = 1.0
    sramp = nt.nodes.new('ShaderNodeValToRGB')
    sramp.color_ramp.elements[0].position = 0.8
    sramp.color_ramp.elements[0].color = (0, 0, 0, 1)
    sramp.color_ramp.elements[1].position = 0.95
    sramp.color_ramp.elements[1].color = (1, 1, 1, 1)
    geom = nt.nodes.new('ShaderNodeNewGeometry')  # world position (the patch mesh is scaled, object coords are not in meters)
    nt.links.new(geom.outputs['Position'], speckle.inputs['Vector'])
    nt.links.new(speckle.outputs['Color'], sramp.inputs['Fac'])
    patches = nt.nodes.new('ShaderNodeTexNoise')
    patches.inputs['Scale'].default_value = 0.5
    patches.inputs['Detail'].default_value = 4
    pramp = nt.nodes.new('ShaderNodeValToRGB')
    pramp.color_ramp.elements[0].position = 0.45
    pramp.color_ramp.elements[0].color = (0.55, 0.55, 0.55, 1)
    pramp.color_ramp.elements[1].position = 0.6
    pramp.color_ramp.elements[1].color = (1, 1, 1, 1)
    nt.links.new(geom.outputs['Position'], patches.inputs['Vector'])
    nt.links.new(patches.outputs['Fac'], pramp.inputs['Fac'])
    mix_patch = nt.nodes.new('ShaderNodeMix')
    mix_patch.data_type = 'RGBA'
    mix_patch.blend_type = 'MULTIPLY'
    mix_patch.inputs['Factor'].default_value = 1.0
    nt.links.new(ramp.outputs['Color'], color_socket(mix_patch.inputs, 'A'))
    nt.links.new(pramp.outputs['Color'], color_socket(mix_patch.inputs, 'B'))
    mix_speck = nt.nodes.new('ShaderNodeMix')
    mix_speck.data_type = 'RGBA'
    color_socket(mix_speck.inputs, 'B').default_value = (0.2, 0.19, 0.18, 1)
    nt.links.new(sramp.outputs['Color'], mix_speck.inputs['Factor'])
    nt.links.new(color_socket(mix_patch.outputs, 'Result'), color_socket(mix_speck.inputs, 'A'))
    nt.links.new(color_socket(mix_speck.outputs, 'Result'), bsdf.inputs['Base Color'])
    nt.links.new(fine.outputs['Fac'], bump.inputs['Height'])
    nt.links.new(bump.outputs['Normal'], bsdf.inputs['Normal'])
    bsdf.inputs['Roughness'].default_value = 0.85
    nt.links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])


def set_metal(mat, color, roughness):
    mat.use_nodes = True
    nt = mat.node_tree
    nt.nodes.clear()
    bsdf = nt.nodes.new('ShaderNodeBsdfPrincipled')
    out = nt.nodes.new('ShaderNodeOutputMaterial')
    bsdf.inputs['Base Color'].default_value = (*color, 1.0)
    bsdf.inputs['Metallic'].default_value = 1.0
    bsdf.inputs['Roughness'].default_value = roughness
    nt.links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])


# The tank entry in the state files lists its shapes and materials: shell (cylinder), then the two saddles
state0 = open(os.path.join(blender_dir, 'output', 'state00000.py')).read()
m = re.search(r"make_chrono_object_assetlist\('tank',.*?\[(.*?)\],\s*\)", state0, re.S)
tank_materials = re.findall(r"\['(material_\d+)'", m.group(1))
set_thin_glass(bpy.data.materials[tank_materials[0]])
set_metal(bpy.data.materials[tank_materials[1]], (0.25, 0.25, 0.27), 0.4)

m = re.search(r"make_chrono_object_assetlist\('patch_1',.*?\[(.*?)\],\s*\)", state0, re.S)
if not m:
    sys.exit('Terrain patch not found in the Blender output: the demo must be run in vehicle mode')
set_asphalt(bpy.data.materials[re.findall(r"\['(material_\d+)'", m.group(1))[0]])

water, wb = principled('water')
wb.inputs['Base Color'].default_value = (0.55, 0.75, 0.95, 1.0)
wb.inputs['Roughness'].default_value = 0.01
wb.inputs['IOR'].default_value = 1.33
wb.inputs['Transmission Weight'].default_value = 1.0
# light absorption in the water volume (blue tint for thick water)
nt = water.node_tree
absorb = nt.nodes.new('ShaderNodeVolumeAbsorption')
absorb.inputs['Color'].default_value = (0.35, 0.6, 0.85, 1.0)
absorb.inputs['Density'].default_value = 0.6
nt.links.new(absorb.outputs['Volume'], nt.nodes['Material Output'].inputs['Volume'])

# -----------------------------------------------------------------------------
# Fluid surface object (outside the Chrono collections, so that the add-on does not delete it)

fluid_mesh = bpy.data.meshes.new('fluid')
fluid = bpy.data.objects.new('fluid', fluid_mesh)
scene.collection.objects.link(fluid)
fluid.data.materials.append(water)


def read_ply(path):
    with open(path, 'rb') as f:
        header = b''
        while not header.endswith(b'end_header\n'):
            header += f.readline()
        h = header.decode()
        nv = int(re.search(r'element vertex (\d+)', h).group(1))
        nf = int(re.search(r'element face (\d+)', h).group(1))
        has_normals = 'property float nx' in h
        vdt = np.dtype([('p', '<f4', 3)] + ([('n', '<f4', 3)] if has_normals else []))
        verts = np.frombuffer(f.read(nv * vdt.itemsize), dtype=vdt)
        fdt = np.dtype([('c', 'u1'), ('i', '<u4', 3)])
        faces = np.frombuffer(f.read(nf * fdt.itemsize), dtype=fdt)
        assert np.all(faces['c'] == 3)
    return verts['p'], faces['i']


def load_fluid(frame):
    path = os.path.join(run_dir, 'surfaces', f'fluid_{frame:05d}.ply')
    new_mesh = bpy.data.meshes.new('fluid')
    if os.path.exists(path):
        v, t = read_ply(path)
        new_mesh.vertices.add(len(v))
        new_mesh.vertices.foreach_set('co', v.ravel())
        new_mesh.loops.add(t.size)
        new_mesh.loops.foreach_set('vertex_index', t.ravel().astype(np.int32))
        new_mesh.polygons.add(len(t))
        new_mesh.polygons.foreach_set('loop_start', np.arange(0, t.size, 3, dtype=np.int32))
        new_mesh.update()
        new_mesh.shade_smooth()
    new_mesh.materials.append(water)
    old = fluid.data
    fluid.data = new_mesh
    bpy.data.meshes.remove(old)
    fluid.location = offsets.get(frame, mathutils.Vector())


# -----------------------------------------------------------------------------
# Visual cues on the ground: lane markings and traffic cones along the double lane change.
# The demo path starts at (-40, -15) with run 30, ramp 10, length 5 and lane offset 3 (to the left):
#   original lane y = -15 for x < -10 and x > 15, offset lane y = -12 for 0 < x < 5.

LANE_Y = -15.0
LANE_WIDTH = 3.0
LINE_WIDTH = 0.12
LINE_Z = 0.004  # just above the terrain surface (z = 0)
X_ROAD = (-145.0, 145.0)


def add_quads(name, quads, mat):
    """Flat mesh made of axis-aligned rectangles (x0, x1, y0, y1) at height LINE_Z."""
    verts, faces = [], []
    for x0, x1, y0, y1 in quads:
        k = len(verts)
        verts += [(x0, y0, LINE_Z), (x1, y0, LINE_Z), (x1, y1, LINE_Z), (x0, y1, LINE_Z)]
        faces.append((k, k + 1, k + 2, k + 3))
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, [], faces)
    mesh.materials.append(mat)
    obj = bpy.data.objects.new(name, mesh)
    scene.collection.objects.link(obj)
    return obj


def make_cone_mesh():
    """Traffic cone (0.5 m tall) on a square base, with a white reflective band (second material)."""
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0, matrix=mathutils.Matrix.Diagonal((0.36, 0.36, 0.03, 1.0)) @
                          mathutils.Matrix.Translation((0, 0, 0.5)))
    bmesh.ops.create_cone(bm, cap_ends=False, segments=24, radius1=0.13, radius2=0.025, depth=0.47,
                          matrix=mathutils.Matrix.Translation((0, 0, 0.03 + 0.235)))
    mesh = bpy.data.meshes.new('cone')
    bm.to_mesh(mesh)
    bm.free()
    for poly in mesh.polygons:
        poly.use_smooth = True
    return mesh


if not args.no_cues:
    paint, pb = principled('paint')
    pb.inputs['Base Color'].default_value = (0.85, 0.85, 0.8, 1.0)
    pb.inputs['Roughness'].default_value = 0.6

    # Solid edge lines of a two-lane road (original lane and offset lane), dashed center line
    y_right = LANE_Y - LANE_WIDTH / 2
    y_center = LANE_Y + LANE_WIDTH / 2
    y_left = LANE_Y + 3 * LANE_WIDTH / 2
    quads = [(X_ROAD[0], X_ROAD[1], y - LINE_WIDTH / 2, y + LINE_WIDTH / 2) for y in (y_right, y_left)]
    dash, gap = 2.0, 3.0
    x = X_ROAD[0]
    while x + dash <= X_ROAD[1]:
        quads.append((x, x + dash, y_center - LINE_WIDTH / 2, y_center + LINE_WIDTH / 2))
        x += dash + gap
    add_quads('lane_lines', quads, paint)

    # Cone material: orange, with a white band between heights 0.28 and 0.36 (object coordinates)
    cone_mat = bpy.data.materials.new('cone')
    cone_mat.use_nodes = True
    nt = cone_mat.node_tree
    cb = nt.nodes['Principled BSDF']
    cb.inputs['Roughness'].default_value = 0.45
    coord = nt.nodes.new('ShaderNodeTexCoord')
    sep = nt.nodes.new('ShaderNodeSeparateXYZ')
    band = nt.nodes.new('ShaderNodeValToRGB')
    band.color_ramp.interpolation = 'CONSTANT'
    # constant color ramp over height: black base, orange, white band (0.28-0.36), orange
    els = band.color_ramp.elements
    els[0].position, els[0].color = 0.0, (0.05, 0.05, 0.05, 1)
    els[1].position, els[1].color = 0.035, (0.9, 0.18, 0.01, 1)
    els.new(0.28).color = (0.9, 0.9, 0.9, 1)
    els.new(0.36).color = (0.9, 0.18, 0.01, 1)
    nt.links.new(coord.outputs['Object'], sep.inputs['Vector'])
    nt.links.new(sep.outputs['Z'], band.inputs['Fac'])
    nt.links.new(band.outputs['Color'], cb.inputs['Base Color'])

    cone_mesh = make_cone_mesh()
    cone_mesh.materials.append(cone_mat)

    # Cone gates on both sides of the lane the vehicle must be in (cones on the lane lines)
    gates = [
        (np.arange(-30.0, -14.0 + 0.1, 4.0), LANE_Y),                  # entry: original lane
        (np.array([0.0, 2.5, 5.0]), LANE_Y + LANE_WIDTH),             # offset lane
        (np.arange(15.0, 27.0 + 0.1, 4.0), LANE_Y),                   # exit: original lane
    ]
    n = 0
    for xs, yc in gates:
        for x in xs:
            for y in (yc - LANE_WIDTH / 2, yc + LANE_WIDTH / 2):
                obj = bpy.data.objects.new(f'cone_{n}', cone_mesh)
                obj.location = (x, y, 0.0)
                scene.collection.objects.link(obj)
                n += 1


# -----------------------------------------------------------------------------
# Camera following the tank (tank position = fluid frame offset, since the mirror body is at the origin)

cam_data = bpy.data.cameras.new('camera')
cam_data.lens = 32
camera = bpy.data.objects.new('camera', cam_data)
scene.collection.objects.link(camera)
scene.camera = camera

CAMERA_OFFSETS = {
    'chase': mathutils.Vector((-5.5, -4.5, 2.6)),  # behind and to the right of the trailer, looking forward
    'side': mathutils.Vector((1.0, -7.5, 1.6)),    # side view of the tank
}
cam_offset = CAMERA_OFFSETS[args.camera]
cam_aim = mathutils.Vector((1.2, 0, -0.3))  # look slightly ahead of the tank, toward the hitch

# Smooth the camera path: average the tank position over a window of frames
def tank_position(frame, window=12):
    pts = [offsets[f] for f in range(max(0, frame - window), min(num_frames, frame + window + 1))]
    return sum(pts, mathutils.Vector()) / len(pts)


def update_camera(frame):
    p = tank_position(frame)
    camera.location = p + cam_offset
    direction = (p + cam_aim) - camera.location
    camera.rotation_euler = direction.to_track_quat('-Z', 'Y').to_euler()


def on_frame(scene_, *_):
    f = scene_.frame_current
    load_fluid(f)
    update_camera(f)
    # Line shapes (driver path, suspension springs) are imported as grease pencil strokes; do not render them
    for o in bpy.data.objects:
        if o.type in ('GREASEPENCIL', 'GPENCIL'):
            o.hide_render = True


bpy.app.handlers.frame_change_post.append(on_frame)

# -----------------------------------------------------------------------------
# World and lights

world = bpy.data.worlds.new('world')
scene.world = world
world.use_nodes = True
wnt = world.node_tree
env = wnt.nodes.new('ShaderNodeTexEnvironment')
env.image = bpy.data.images.load(os.path.join(bpy.utils.system_resource('DATAFILES'), 'studiolights', 'world', 'sunrise.exr'))
bg = wnt.nodes['Background']
bg.inputs['Strength'].default_value = 1.0
wnt.links.new(env.outputs['Color'], bg.inputs['Color'])

sun_data = bpy.data.lights.new('sun', 'SUN')
sun_data.energy = 2.5
sun_data.angle = np.radians(2.0)
sun = bpy.data.objects.new('sun', sun_data)
sun.rotation_euler = (np.radians(50), 0, np.radians(-35))
scene.collection.objects.link(sun)

# -----------------------------------------------------------------------------
# Render settings

scene.render.engine = 'CYCLES'
prefs = bpy.context.preferences.addons['cycles'].preferences
for dev_type in ('OPTIX', 'CUDA'):
    try:
        prefs.compute_device_type = dev_type
        prefs.get_devices()
        if any(d.type == dev_type for d in prefs.devices):
            for d in prefs.devices:
                d.use = d.type == dev_type
            scene.cycles.device = 'GPU'
            break
    except TypeError:
        continue
scene.cycles.samples = args.samples
scene.cycles.use_denoising = True
scene.cycles.max_bounces = 16
scene.cycles.transmission_bounces = 16
scene.cycles.transparent_max_bounces = 16
scene.render.resolution_x, scene.render.resolution_y = args.res
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = 'PNG'
scene.view_settings.view_transform = 'AgX'
scene.frame_start = args.first
scene.frame_end = last

if args.save_blend:
    scene.frame_set(args.first)
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(run_dir, 'tank_trailer.blend'))

for frame in (args.frames or range(args.first, last + 1, args.step)):
    scene.frame_set(frame)
    scene.render.filepath = os.path.join(render_dir, f'{args.camera}_{frame:05d}.png')
    bpy.ops.render.render(write_still=True)
    print(f'rendered frame {frame}', flush=True)
