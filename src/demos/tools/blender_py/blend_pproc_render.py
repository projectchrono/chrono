import bpy
import math
import os
import glob
import reader
import mathutils
from blend_ChParticle_utils import renderPsys
import multiprocessing


# resolution reduction for the rendered image, LOW speeds up things for development, HIGH is slow but produces HD images
res_reduction = {
    'HIGH' : 1,
    'MID' : 4,
    'LOW' : 16}

# function to orient a camera to look at a point
def look_at(obj_camera, point, up):
    loc_camera = obj_camera.location
    direction = point - loc_camera
    # point the cameras '-Z' and use its 'Y' as up
    rot_quat = direction.to_track_quat('-Z', up)

    # assume we're using euler rotation
    obj_camera.rotation_euler = rot_quat.to_euler()

# converts creates Blender objects from .dat chrono output format
def convertshape(shapedata, meshespaths):
    # Triangular Mesh
    if shapedata[0] == 5:
        for dirpath in meshespaths:
            file_path = dirpath + "/" + shapedata[-1] + '.obj'
            if os.path.isfile(file_path ):
                imported_object_3 = bpy.ops.import_scene.obj(filepath=file_path )
                obj_object = bpy.context.object
                olist =bpy.context.selected_objects
                for o in olist:
                    o.location.x = shapedata[3]
                    o.location.y = shapedata[4]
                    o.location.z = shapedata[5]
                    o.rotation_mode = 'QUATERNION'
                    q = (shapedata[6], shapedata[7], shapedata[8], shapedata[9])
                    o.rotation_quaternion = q
                return True

        return False
    # sphere, box cylinder
    elif shapedata[0]==0 or shapedata[0]==2 or shapedata[0]==3:
        obj = None
        # Sphere
        if shapedata[0] == 0:
            bpy.ops.mesh.primitive_ico_sphere_add(size=shapedata[13], location=(shapedata[3], shapedata[4], shapedata[5]),
                                                  rotation=mathutils.Quaternion(shapedata[6], shapedata[7], shapedata[8], shapedata[9]).to_euler())
            obj = bpy.context.active_object


        # Box
        elif shapedata[0] == 2:
            bpy.ops.mesh.primitive_cube_add(size=1, location=(shapedata[3], shapedata[4], shapedata[5]),
                                                  rotation=mathutils.Quaternion((shapedata[6], shapedata[7], shapedata[8], shapedata[9])).to_euler())
            obj = bpy.context.active_object
            obj.scale= ( shapedata[13]*2, shapedata[14]*2, shapedata[15]*2)


        # Cylinder
        elif shapedata[0] == 3:
            bodypos = mathutils.Vector((shapedata[3], shapedata[4], shapedata[5]))
            bodyrot = mathutils.Quaternion((shapedata[6], shapedata[7], shapedata[8], shapedata[9]))
            #p1 & p2
            p1 = mathutils.Vector((shapedata[14], shapedata[15], shapedata[16]))
            p2 = mathutils.Vector((shapedata[17], shapedata[18], shapedata[19]))
            # p2 - p1 vector
            axis = p2 - p1
            #cyl center in loc ref then transform
            glob_center = p1 + axis/2
            glob_center.rotate(bodyrot)
            # cyl center = body_ref +  (p1 + (p2-p1)/2) in abs ref
            center =  bodypos +  glob_center
            # direction -> Quat -> EulerAng
            ang = (bodyrot @ axis.to_track_quat('Z', 'X')).to_euler()
            #ang = axis.to_track_quat('Z', 'X').to_euler()
            bpy.ops.mesh.primitive_cylinder_add(location=center, rotation=ang, radius=shapedata[13], depth=axis.length)
            obj = bpy.context.active_object


        mat = bpy.data.materials.new("Blue")
        # Activate its nodes
        mat.use_nodes = True
        # Get the principled BSDF (created by default)
        principled = mat.node_tree.nodes['Principled BSDF']
        # Assign the color
        principled.inputs['Base Color'].default_value = (shapedata[10], shapedata[11], shapedata[12],1)
        # Assign the material to the object
        obj.data.materials.append(mat)
        return True
    else:
        print('Unrecognized shape type')
        return False

# go through all shapes, find specific one, return true + frame of the shape
def find_camera_target(exported_shapes, body_id, type_id, shape_name = ''):
    for shapedata in exported_shapes:
        if shapedata[1] == body_id:
            # if looking for mesh, is shape is mesh
            if type_id == 5 and shapedata[0] == 5 and shape_name == shapedata[-1]:
                return True, [shapedata[3],shapedata[4],shapedata[5]], [shapedata[6],shapedata[7],shapedata[8], shapedata[9]]
            # if looking for generic shape on body:
            elif type_id == shapedata[0] :
                return True, [shapedata[3],shapedata[4],shapedata[5]], [shapedata[6],shapedata[7],shapedata[8], shapedata[9]]


def render_image(dirpaths, filepath,file_index, out_dir, meshes_prefixes,res,  targ,
                 camera_mode, camera_pos, use_sky, a_up, light_loc, light_energy):

    # path to the chrono and chrono-particle systems outputs. Processed only if they exists
    dat_path = dirpaths[0] + filepath + ".dat"
    #particle_path = dirpaths[0] + filepath + ".chpf"


    # delete cached objects (initial cube and shapes from previous frames
    for oldobj in bpy.data.objects:
        if oldobj != bpy.data.objects['Camera']:
            bpy.data.objects.remove(oldobj)

    # IF there is a chrono .dat output, it gets processed
    expshapes = []
    for dir in dirpaths:
        if (os.path.exists(dir + filepath + ".dat")):
            expshapes += reader.import_shapes(dir + filepath + ".dat")
    for shapedata in expshapes:
        isconv = convertshape(shapedata, meshes_prefixes)
        if not isconv:
            print('Error while importing meshes')

    if (os.path.exists(dat_path)):
        targetfound, targ_pos, targ_or = find_camera_target(expshapes, targ["bodyid"], targ["shapetypeid"], targ["name"])
        if not targetfound:
                print('Could not find target object')


    scene = bpy.context.scene
    scene.objects.keys()

    if camera_mode == 'Lookat':
        obj_camera = bpy.data.objects["Camera"]
        obj_camera.location = camera_pos
        look_at(obj_camera, mathutils.Vector(targ_pos), a_up)


    elif camera_mode == 'Follow':
        print('Follow camera mode')
        obj_camera = bpy.data.objects["Camera"]
        # get the distance in the global ref frame:
        global_dist = mathutils.Vector(targ["distfrom"])
        global_dist.rotate(mathutils.Quaternion(targ_or))
        moving_camera_pos = mathutils.Vector(targ_pos) + global_dist
        obj_camera.location = moving_camera_pos
        look_at(obj_camera, mathutils.Vector(targ_pos), a_up)
    else:
        obj_camera = bpy.data.objects["Camera"]
        obj_camera.location = camera_pos
        look_at(obj_camera, mathutils.Vector(targ["position"]), a_up)


    # IF there is a chrono .chpf output, it gets processed
    for dirpath in dirpaths:
        partpath = dirpath + filepath + ".chpf"
        if (os.path.exists(partpath)):
            renderPsys(partpath, bpy.data.objects["Camera"])

    # distance above which objects are not rendered
    #TODO: make this settable!!
    bpy.context.scene.camera.data.clip_end = 1000
    scene.cycles.device = 'GPU'
    prefs = bpy.context.preferences
    cprefs = prefs.addons['cycles'].preferences

    # Attempt to set GPU device types if available
    for compute_device_type in ('CUDA', 'OPENCL', 'NONE'):
        try:
            cprefs.compute_device_type = compute_device_type
            break
        except TypeError:
            pass

    # Enable all CPU and GPU devices
    cprefs.get_devices()
    for device in cprefs.devices:
        device.use = True


    if use_sky:
        sky_texture = bpy.context.scene.world.node_tree.nodes.new("ShaderNodeTexSky")
        bg = bpy.context.scene.world.node_tree.nodes["Background"]
        bpy.context.scene.world.node_tree.links.new(bg.inputs["Color"], sky_texture.outputs["Color"])
        sky_texture.sky_type = 'PREETHAM'
        #sky_texture.turbidity = 2.0
        #sky_texture.ground_albedo = 0.4
        ##sky_texture.sun_direction = mathutils.Vector((1.0, 0.0, 1.0))  # add `import mathutils` at the beginning of the script

    # create light datablock, set attributes
    light_data = bpy.data.lights.new(name="light_2.80", type='POINT')
    light_data.energy = light_energy
    # create new object with our light datablock
    light_object = bpy.data.objects.new(name="light_2.80", object_data=light_data)
    # link light object
    bpy.context.collection.objects.link(light_object)
    # make it active
    bpy.context.view_layer.objects.active = light_object
    # change location
    light_object.location = light_loc

    bpy.context.scene.render.engine = 'CYCLES'
    bpy.context.scene.cycles.device = 'GPU'
    #bpy.context.scene.render.resolution_percentage = 200
    bpy.context.scene.cycles.samples = 256
    bpy.context.scene.render.resolution_x = 3840/res_reduction[res]
    bpy.context.scene.render.resolution_y = 2160/res_reduction[res]
    bpy.context.scene.render.filepath = out_dir + "/bl_render_out_" + str(file_index) + ".png"
    #bpy.context.scene.render.image_settings.compression = 500
    bpy.context.scene.render.image_settings.color_mode = 'RGBA'
    bpy.context.scene.render.image_settings.file_format = 'PNG'
    #bpy.ops.render.render(write_still=True)

    bpy.ops.render.render(write_still=True)
    print('Rendered frame')

def bl_render(meshes_prefixes, out_dir, datadirs, res, camera_mode, use_sky, camera_pos, targ, up='Z', light_loc=(10, 10, 15), light_energy=13000 ):
    if up == 'Z':
        a_up = 'Y'
    elif up == 'Y':
        a_up = 'Z'
    else:
        print('Invalid up-axis choice')

    # populates the list with all the chrono output files, truncating the suffix (since there might be both .dat and .chpf)
    datafiles = []
    #for file in os.listdir(datadir):
    for root, dirs, files in os.walk(datadirs[0]):
        for file in files:
            files.sort()
            if file.endswith(".dat") or file.endswith(".chpf"):
                datafiles.append(file[:file.rfind(".")])


    if os.name == 'nt':
        for file_ind, datafile in enumerate(datafiles):
            render_image(datadirs, datafile, file_ind, out_dir, meshes_prefixes, res, targ,
                         camera_mode, camera_pos, use_sky, a_up, light_loc, light_energy)

    elif os.name == 'posix':
        print('MULTIPROCESSING ACTIVE')
        pool = multiprocessing.Pool(processes=multiprocessing.cpu_count()-1)
        pool.starmap(render_image, [(datadirs, datafile, file_ind, out_dir, meshes_prefixes, res, targ,
                         camera_mode, camera_pos, use_sky, a_up, light_loc, light_energy)
                                for file_ind, datafile in enumerate(datafiles)])
        pool.terminate()
