import bpy
import math
import os
import glob
import reader
import mathutils


# resolution reduction for the rendered image, LOW speeds up things for development, HIGH is slow but produces HD images
res_reduction = {
    'HIGH' : 1,
    'MID' : 4,
    'LOW' : 16}

def look_at(obj_camera, point):
    loc_camera = obj_camera.location
    
    print(loc_camera)

    direction = point - loc_camera
    # point the cameras '-Z' and use its 'Y' as up
    rot_quat = direction.to_track_quat('-Z', 'Y')

    # assume we're using euler rotation
    obj_camera.rotation_euler = rot_quat.to_euler()

def convertshape(shapedata, meshespaths):
    old_numob = len(bpy.context.selected_objects)
    if shapedata[0] == 5:
        for dirpath in meshespaths:
            file_path = dirpath + shapedata[-1] + '.obj'
            if os.path.isfile(file_path ):
                imported_object_3 = bpy.ops.import_scene.obj(filepath=file_path )
                obj_object = bpy.context.object
                #bpy.ops.transform.rotate(value=(math.pi * 0.5), orient_axis='X')  # value = Angle
                #bpy.ops.transform.translate(value=(float(shapedata[3]), float(shapedata[4]), float(shapedata[5])))
                new_numob = len(bpy.context.selected_objects)
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


def bl_render(meshes_prefixes, out_dir, datadir, res, camera_mode, use_sky, camera_pos, targ_bodyid, targ_shapetypeid, targ_name, camera_dist, light_loc=(10, 10, 15), light_energy=13000 ):
    datafiles = []
    #for file in os.listdir(datadir):
    for root, dirs, files in os.walk(datadir):
        for file in files:
            if file.endswith(".dat"):
                datafiles.append(file)
            
    for file_ind, datafile in enumerate(datafiles):
    
        expshapes = reader.import_shapes(datadir+datafile)
        # optional: delete cahed objects
        for oldobj in bpy.data.objects : 
            bpy.data.objects.remove(oldobj)
    
        for shapedata in expshapes:
            isconv = convertshape(shapedata, meshes_prefixes)
            if not isconv:
                print('Error while importing meshes')
        
    
        targetfound, targ_pos, targ_or = find_camera_target(expshapes, targ_bodyid, targ_shapetypeid, targ_name)
        if not targetfound:
                print('Could not find target object')
        
        scene = bpy.context.scene
        scene.objects.keys()
        
        bpy.ops.object.camera_add(enter_editmode=False, align='WORLD', location=(1,1,1),
                                      rotation=(1.2548917, 0.0139800873, -0.6300638))
        scene.camera = bpy.context.object
        
        if camera_mode == 'Lookat':
            obj_camera = bpy.data.objects["Camera"]
            obj_camera.location = camera_pos
            look_at(obj_camera, mathutils.Vector(targ_pos))
            
            
        elif camera_mode == 'Follow':
            print('Follow camera mode')
            obj_camera = bpy.data.objects["Camera"]
            # get the distance in the global ref frame:
            global_dist = mathutils.Vector(camera_dist)
            global_dist.rotate(mathutils.Quaternion(targ_or))
            moving_camera_pos = mathutils.Vector(targ_pos) + global_dist
            obj_camera.location = moving_camera_pos 
            #obj_camera.location = mathutils.Vector((1,2,3))
            print(moving_camera_pos )
            look_at(obj_camera, mathutils.Vector(targ_pos))
        
        
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
            #sky_texture.sky_type = 'HOSEK_WILKIE' # or 'PREETHAM'
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
        bpy.context.scene.render.filepath = out_dir + "/bl_render_out_" + str(file_ind) + ".png"
        #bpy.context.scene.render.image_settings.compression = 500
        bpy.context.scene.render.image_settings.color_mode = 'RGBA'
        bpy.context.scene.render.image_settings.file_format = 'PNG'
        #bpy.ops.render.render(write_still=True)
        
        
        
        bpy.ops.render.render(write_still=True)
        print('Rendered frame')