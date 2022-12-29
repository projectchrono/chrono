bl_info = {
    "name": "Chrono import",
    "blender": (2, 80, 0),
    "category": "Import-Export",
    "location": "File > Import-Export",
    "description": "Import ProjectChrono simulations",
    "author": "Alessandro Tasora",
    "version": (0, 0, 1),
    "wiki_url": "http://projectchrono.org",
    "doc_url": "http://projectchrono.org",
}

import bpy
import bmesh
import numpy as np
import mathutils
import os

#
# Globals, to keep things simple with callbacks
#

chrono_collection_objects = None
chrono_collection_assets = None
chrono_collection_cameras = None
empty_mesh = None
chrono_filename = None

#
# utility functions to be used in assets.py
#


def make_bsdf_material(nameID, colorRGB, metallic=0, specular=0, specular_tint=0, roughness=0.5, index_refraction=1.450, transmission=0, emissionRGB=(0,0,0,1), emission_strength=1, bump_map=0, bump_height=1.0):
    new_mat = bpy.data.materials.new(name=nameID)

    new_mat.use_nodes = True

    if new_mat.node_tree:
        new_mat.node_tree.links.clear()
        new_mat.node_tree.nodes.clear()
        
    nodes = new_mat.node_tree.nodes
    links = new_mat.node_tree.links
    output = nodes.new(type='ShaderNodeOutputMaterial')
    shader = nodes.new(type='ShaderNodeBsdfPrincipled')
    
    if type(colorRGB) == tuple:
        nodes["Principled BSDF"].inputs[0].default_value = colorRGB
    if type(colorRGB) == str:
        if os.path.exists(colorRGB):
            texturenode = nodes.new(type="ShaderNodeTexImage")
            texturenode.image = bpy.data.images.load(colorRGB)
            links.new(texturenode.outputs[0], nodes["Principled BSDF"].inputs[0])
            
    if type(metallic) == float or type(metallic) == int:
        nodes["Principled BSDF"].inputs[6].default_value = metallic
    if type(metallic) == str:
        if os.path.exists(metallic):
            texturenode = nodes.new(type="ShaderNodeTexImage")
            texturenode.image = bpy.data.images.load(metallic)
            links.new(texturenode.outputs[0], nodes["Principled BSDF"].inputs[6])
            
    nodes["Principled BSDF"].inputs[7].default_value = specular
    nodes["Principled BSDF"].inputs[8].default_value = specular_tint

    if type(roughness) == float or type(roughness) == int:
        nodes["Principled BSDF"].inputs[9].default_value = roughness
    if type(roughness) == str:
        if os.path.exists(roughness):
            texturenode = nodes.new(type="ShaderNodeTexImage")
            texturenode.image = bpy.data.images.load(roughness)
            links.new(texturenode.outputs[0], nodes["Principled BSDF"].inputs[9])
    
    nodes["Principled BSDF"].inputs[16].default_value = index_refraction

    if type(transmission) == float or type(transmission) == int:
        nodes["Principled BSDF"].inputs[17].default_value = transmission
    if type(transmission) == str:
        if os.path.exists(transmission):
            texturenode = nodes.new(type="ShaderNodeTexImage")
            texturenode.image = bpy.data.images.load(transmission)
            links.new(texturenode.outputs[0], nodes["Principled BSDF"].inputs[17])
    
    if type(emissionRGB) == tuple:
        nodes["Principled BSDF"].inputs[19].default_value = emissionRGB
    if type(emissionRGB) == str:
        if os.path.exists(emissionRGB):
            texturenode = nodes.new(type="ShaderNodeTexImage")
            texturenode.image = bpy.data.images.load(emissionRGB)
            links.new(texturenode.outputs[0], nodes["Principled BSDF"].inputs[19])
            
    nodes["Principled BSDF"].inputs[20].default_value = emission_strength
    
    if type(bump_map) == str:
        if os.path.exists(bump_map):
            texturenode = nodes.new(type="ShaderNodeTexImage")
            texturenode.image = bpy.data.images.load(bump_map)
            bumpnode = nodes.new(type="ShaderNodeBump")
            bumpnode.inputs[0].default_value= bump_height
            links.new(texturenode.outputs[0], bumpnode.inputs[2])
            links.new(bumpnode.outputs[0], nodes["Principled BSDF"].inputs[22])
    
    links.new(shader.outputs[0], output.inputs[0])
    

        


#
# utility functions to be used in output/statexxxyy.py files
#

def make_chrono_object_assetlist(mname,mpos,mrot, masset_list):

    chobject = bpy.data.objects.new(mname, empty_mesh)  
    chobject.rotation_mode = 'QUATERNION'
    chobject.rotation_quaternion = mrot
    chobject.location = mpos
    chrono_collection_objects.objects.link(chobject)
    
    for m in range(len(masset_list)):
        masset = chrono_collection_assets.objects.get(masset_list[m][0])
        if masset:
            chasset = masset.copy() # instanced, no full copy, masset.data is shared
            chasset.rotation_mode = 'QUATERNION'
            chasset.rotation_quaternion = masset_list[m][2]
            chasset.location = masset_list[m][1]
            if len(masset_list[m])>4:
                chasset.scale = masset_list[m][4]
            chrono_collection_objects.objects.link(chasset)
            chasset.parent = chobject
            if masset_list[m][3]:
                chasset.material_slots[-1].link = 'OBJECT'
                chasset.material_slots[-1].material = bpy.data.materials[masset_list[m][3]] 
        else:
            print("not found asset: ",masset_list[m][0])
    
    
def make_chrono_object_clones(mname,mpos,mrot, masset_list, list_clones_posrot):
    
    chobject = bpy.data.objects.new(mname, empty_mesh)  
    chobject.rotation_mode = 'QUATERNION'
    chobject.rotation_quaternion = mrot
    chobject.location = mpos
    chrono_collection_objects.objects.link(chobject)
    
    chassets_group = bpy.data.objects.new("assets_group", empty_mesh)  
    chrono_collection_objects.objects.link(chassets_group)
    chassets_group.parent = chobject
    chassets_group.hide_set(True)
    chassets_group.hide_render = True
    
    for m in range(len(masset_list)):
        masset = chrono_collection_assets.objects.get(masset_list[m][0])
        if masset:
            chasset = masset.copy() # instanced, no full copy, masset.data is shared
            chasset.rotation_mode = 'QUATERNION'
            chasset.rotation_quaternion = masset_list[m][2]
            chasset.location = masset_list[m][1]
            if len(masset_list[m])>4:
                chasset.scale = masset_list[m][4]
            chrono_collection_objects.objects.link(chasset)
            chasset.parent = chassets_group
            if masset_list[m][3]:
                chasset.material_slots[-1].link = 'OBJECT'
                chasset.material_slots[-1].material = bpy.data.materials[masset_list[m][3]]
            chasset.hide_set(True)
        else:
            print("not found asset: ",masset_list[m][0])
        
    ncl = len(list_clones_posrot)
    verts = [(0,0,0)] * (4*ncl)
    faces = [(0,0,0,0)] * ncl
    edges = []
    for ic in range(ncl):
        mpos = mathutils.Vector(list_clones_posrot[ic][0])
        mrot = mathutils.Quaternion(list_clones_posrot[ic][1])
        verts[4*ic]   = (mpos + mrot @ mathutils.Vector((-0.1,-0.1,0)))[:]
        verts[4*ic+1] = (mpos + mrot @ mathutils.Vector(( 0.1,-0.1,0)))[:]
        verts[4*ic+2] = (mpos + mrot @ mathutils.Vector(( 0.1, 0.1,0)))[:]
        verts[4*ic+3] = (mpos + mrot @ mathutils.Vector((-0.1, 0.1,0)))[:]
        faces[ic] = (4*ic, 4*ic+1, 4*ic+2, 4*ic+3)    
    new_mesh = bpy.data.meshes.new('mesh_position_clones')
    new_mesh.from_pydata(verts, edges, faces)
    new_mesh.update()
    chobject.data = new_mesh
    chobject.instance_type = 'FACES'
    chobject.show_instancer_for_render = False
    chobject.show_instancer_for_viewport = False
    
def update_camera_coordinates(mname,mpos,mrot):
    cameraasset = chrono_collection_cameras.objects.get(mname)
    cameraasset.rotation_mode = 'QUATERNION'
    cameraasset.rotation_quaternion = mrot
    cameraasset.location = mpos

 
    
#
# On file selected: 
#
# RESET CHRONO OBJECTS COLLECTION
# LOAD OBJECTS
#

def callback_post(self):

    scene = bpy.context.scene
    cFrame = scene.frame_current
    sFrame = scene.frame_start
    
    global chrono_collection_objects
    global chrono_collection_assets
    global chrono_collection_cameras
    global empty_mesh
    global chrono_filename
    
    chrono_collection_assets = bpy.data.collections.get('chrono_assets')
    chrono_collection_objects = bpy.data.collections.get('chrono_objects')
    chrono_collection_cameras = bpy.data.collections.get('chrono_cameras')
    
    if (chrono_collection_assets and chrono_collection_objects):
        chrono_filename = chrono_collection_assets['chrono_filename'] 
        
        # delete all things in chrono_assets collection
        for obj in chrono_collection_objects.objects:
            bpy.data.objects.remove(obj, do_unlink=True)
          
        # load state file
        proj_dir = os.path.dirname(os.path.abspath(chrono_filename))
        filename = os.path.join(proj_dir, 'output', 'state'+'{:05d}'.format(cFrame)+'.py')
        
        if os.path.exists(filename):
            f = open(filename, "rb")
            exec(compile(f.read(), filename, 'exec'))
            f.close()
     
#
# On file selected: 
#
# PREPARE CHRONO ASSETS COLLECTION
# PREPARE CHRONO OBJECTS COLLECTION
# LOAD ASSETS
#
    
def read_chrono_simulation(context, filepath, setting_materials):
    print("Loading Chrono simulation...")
    
    # PREPARE SCENE
    global chrono_collection_objects
    global chrono_collection_assets
    global chrono_collection_cameras
    global empty_mesh
    global chrono_filename
    
    chrono_collection_cameras = bpy.data.collections.get('chrono_cameras')
    if not chrono_collection_cameras:
        chrono_collection_cameras = bpy.data.collections.new('chrono_cameras')
        bpy.context.scene.collection.children.link(chrono_collection_cameras)
    for obj in chrono_collection_cameras.objects:
            bpy.data.objects.remove(obj, do_unlink=True)
            
    chrono_collection_assets = bpy.data.collections.get('chrono_assets')
    if not chrono_collection_assets:
        chrono_collection_assets = bpy.data.collections.new('chrono_assets')
        bpy.context.scene.collection.children.link(chrono_collection_assets)
    for obj in chrono_collection_assets.objects:
            bpy.data.objects.remove(obj, do_unlink=True)
            
    chrono_collection_assets.hide_render = True
    #chrono_collection_assets.hide_viewport = True
            
    chrono_collection_objects = bpy.data.collections.get('chrono_objects')
    if not chrono_collection_objects:
        chrono_collection_objects = bpy.data.collections.new('chrono_objects')
        bpy.context.scene.collection.children.link(chrono_collection_objects)
    for obj in chrono_collection_objects.objects:
            bpy.data.objects.remove(obj, do_unlink=True)
            
    orphan_mesh = [m for m in bpy.data.meshes if not m.users]
    while orphan_mesh:
        bpy.data.meshes.remove(orphan_mesh.pop())
        
    orphan_particles = [m for m in bpy.data.particles if not m.users]
    while orphan_particles:
        bpy.data.particles.remove(orphan_particles.pop())
        
    orphan_materials = [m for m in bpy.data.materials if not m.users]
    while orphan_materials:
        bpy.data.materials.remove(orphan_materials.pop())
        
           
    # if some sub collection is selected, select the root one, so loading assets.py will work fine  
    bpy.context.view_layer.active_layer_collection = bpy.context.view_layer.layer_collection
       

    empty_mesh = bpy.data.meshes.new('empty_mesh')
    empty_mesh.from_pydata([], [], []) 
    empty_mesh_object = bpy.data.objects.new('empty_mesh_object', empty_mesh)



    # load assets file
    chrono_filename = filepath
    
    # store filename as custom properties of collection, so that one can save the Blender project,
    # reopen, and scrub the timeline without the need of doing File/Import/Chrono Import
    chrono_collection_assets['chrono_filename'] = chrono_filename
    
    f = open(chrono_filename, "rb")
    exec(compile(f.read(), chrono_filename, 'exec'))
    f.close()

    for masset in chrono_collection_assets.objects:
        if not masset.type == 'CAMERA':
            masset.hide_set(True) # not masset.hide_viewport = True otherwise also instances are invisible
            #masset.hide_render = True
            masset.data.materials.append(None) # force a free slot if per-instance material will be assigned


    degp = bpy.context.evaluated_depsgraph_get()

      
    #clear the post frame handler
    bpy.app.handlers.frame_change_post.clear()

    #run the function on each frame
    bpy.app.handlers.frame_change_post.append(callback_post)


    # TEST: Update to a frame where particles are updated
    bpy.context.scene.frame_current = 0


    return {'FINISHED'}









# ImportHelper is a helper class, defines filename and
# invoke() function which calls the file selector.
from bpy_extras.io_utils import ImportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty, IntProperty
from bpy.types import Operator


class ImportChrono(Operator, ImportHelper):
    """Import a Chrono simulation from files saved by Chrono postprocessing module"""
    bl_idname = "import_chrono.data"  # important since its how bpy.ops.import_chrono.data is constructed
    bl_label = "Import Chrono simulation"

    # ImportHelper mixin class uses this
    filename_ext = ".py"

    filter_glob: StringProperty(
        default="*.assets.py",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )

    # List of operator properties, the attributes will be assigned
    # to the class instance from the operator settings before calling.
    setting_materials: BoolProperty(
        name="Create materials",
        description="Turn off if you want to skip all materials assigned from Chrono side",
        default=True,
    )
    #    setting_from: IntProperty(
    #        name="from",
    #        description="Initial frame",
    #        min = 0,
    #        default=0,
    #    )
    #    setting_to: IntProperty(
    #        name="to",
    #        description="Final frame",
    #        min = 0,
    #        default=0,
    #    )
    
    def execute(self, context):
        return read_chrono_simulation(context, self.filepath, self.setting_materials)


# Only needed if you want to add into a dynamic menu.
def menu_func_import(self, context):
    self.layout.operator(ImportChrono.bl_idname, text="Chrono import")


# Register and add to the "file selector" menu (required to use F3 search "Text Import Operator" for quick access).
def register():
    bpy.utils.register_class(ImportChrono)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)
    
        #clear the post frame handler
    bpy.app.handlers.frame_change_post.clear()

    #run the function on each frame
    bpy.app.handlers.frame_change_post.append(callback_post)


def unregister():
    bpy.utils.unregister_class(ImportChrono)
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)
    bpy.app.handlers.frame_change_post.remove(callback_post)


if __name__ == "__main__":
    register()

    # TEST: test call
    #bpy.ops.import_chrono.data('INVOKE_DEFAULT')
