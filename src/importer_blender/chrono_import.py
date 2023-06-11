# 
# CHRONO IMPORT ADD-ON FOR BLENDER
#
# This is an add-on for Blender3D that allows loading simulations performed with 
# Chrono (http://www.projectchrono.org) and saved with the Chrono POSTPROCESS module.
# 
# To install it: 
# - open Blender
# - menu "Edit/Preferences..", go to tab "Add-ons", press "Install" button and select 
#   this .py file
# - check that the Chrono Import add-on is now available in the list, and it is enabled
#
# To use it: 
# - check that you built the c++ Chrono with the POSTPROCESS module enabled (if you use
#   PyChrono, the postprocess module is always available in the AnaConda installation)
# - write a Chrono app that uses instructions like my_blender_exporter.AddAll(); 
#   my_blender_exporter.ExportScript(); and my_blender_exporter.ExportData(); (the latter
#   in the while() simulation loop). See demo_POST_blender1.cpp for an example.
# - run the chrono app,  this will generate files on disk: a single
#   xxx.assets.py file and many state00001.py, state00002.py, ..., in an output/ dir.
# - Open Blender, use menu "File/Import/Chrono import" to load the xxx.assets.py file.
#
# Tips:
# - When scrubbing the timeline, the animation should happen. This may be a bit slower than 
#   expected because the Chrono add-on loads and unloads from disk the content of each time
#   step, for optimizing memory requirements.
# - use the "Chrono" sidebar at the right border of the 3D view to enable/disable the
#   showing of coordinate systems of Chrono bodies and assets, and other view settings.
# - when possible, the Chrono Blender postprocessor and this add-on tend to optimize memory
#   by sharing identical assets. More in detail, assets for whom myasset->IsMutable() is false
#   in Chrono, ex. spheres, boxes, not deformable meshes, etc., will be put in the "chrono_assets"
#   collection in the Blender outliner interface. Click on the eye icon to show-hide them in the viewport. 
#   This allows you to edit and improve those assets by hand in Blender, it is enough that you 
#   do not change their name. 
# - if no material is added to a visual shape in Chrono, you can add by hand in Blender to 
#   the asset object available in "chrono_assets" collection. (link to data, not to object);
#   if a material is added to a visual shape in Chrono, it overrides the material you add to
#   the asset object available in "chrono_assets" collection (unless you disable "Chrono materials" in
#   the Chrono sidebar panel).



bl_info = {
    "name": "Chrono import",
    "blender": (2, 80, 0),
    "category": "Import-Export",
    "location": "File > Import-Export",
    "description": "Import ProjectChrono simulations",
    "author": "Alessandro Tasora",
    "version": (0, 0, 3),
    "wiki_url": "https://api.projectchrono.org/development/introduction_chrono_blender.html",
    "doc_url": "https://api.projectchrono.org/development/introduction_chrono_blender.html",
}

import bpy
import bmesh
import numpy as np
import mathutils
import os
import math
from enum import Enum
from bpy.types import (Operator,
                       Panel,
                       PropertyGroup,
                       UIList)
from bpy.props import (IntProperty,
                       BoolProperty,
                       StringProperty,
                       FloatProperty,
                       FloatVectorProperty,
                       EnumProperty,
                       PointerProperty,
                       CollectionProperty)

#
# Globals, to keep things simple with callbacks
#

chrono_frame_objects = None
chrono_assets = None
chrono_frame_assets = None
chrono_materials = []
chrono_frame_materials = []
chrono_images = []
chrono_frame_images = []
chrono_cameras = None
empty_mesh = None
chrono_csys = None
chrono_view_asset_csys = False
chrono_view_asset_csys_size = 0.15
chrono_view_item_csys = False
chrono_view_item_csys_size = 0.25
chrono_view_link_csys = True
chrono_view_link_csys_size = 0.25
chrono_view_materials = True
chrono_view_contacts = False
chrono_gui_doupdate = True

#
# utility functions to be used in assets.py  or   output/statexxxyy.py files
#

def create_chrono_path( nameID,
                        list_points, 
                        RGBAcolor, 
                        line_width,
                        my_list_materials, my_collection):
    
    gpencil_data = bpy.data.grease_pencils.new(nameID)
    gpencil = bpy.data.objects.new(nameID, gpencil_data)
    my_collection.objects.link(gpencil)

    gp_layer = gpencil_data.layers.new("chrono_lines")
    gp_layer.use_lights = False # for Cycles
    gp_frame = gp_layer.frames.new(bpy.context.scene.frame_current)

    gp_stroke = gp_frame.strokes.new()
    gp_stroke.line_width = line_width

    gp_stroke.points.add(len(list_points))

    for item, value in enumerate(list_points):
        gp_stroke.points[item].co = value
        
    mat = bpy.data.materials.new(name="chrono_path_mat")
    bpy.data.materials.create_gpencil_data(mat)
    gpencil.data.materials.append(mat)
    mat.grease_pencil.color = RGBAcolor
    
    my_list_materials.append(mat)
    
    return gpencil
    
    
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
    
    return new_mat
    

# helper to add scalar attributes as arrays of floats to: faces (mdomain='FACE') or vertexes (mdomain='POINT') of meshes
def add_mesh_data_floats(mesh_object, attribute_scalars, attribute_name='myval', mdomain='FACE'):
    myattr = mesh_object.data.attributes.new(name=attribute_name, type='FLOAT', domain=mdomain)
    iv =0
    for mval in attribute_scalars:
        myattr.data[iv].value = mval
        iv +=1

# helper to add vector attributes as arrays of (x,y,z) to: faces (mdomain='FACE') or vertexes (mdomain='POINT') of meshes
def add_mesh_data_vectors(mesh_object, attribute_vectors, attribute_name='chrono_color', mdomain='FACE'):
    myattr = mesh_object.data.attributes.new(name=attribute_name, type='FLOAT_VECTOR', domain=mdomain)
    iv =0
    for mval in attribute_vectors:
        myattr.data[iv].vector = mval
        iv +=1

# Helper function to generate a simple const color material. Operates in two situations:
# - to colorize meshes:
# - to colorize n object instances made by geometry node: if so, the material must be applied
# to the object used as sample for instances, and use per_instance=True. 
def make_material_color_const(nameID, color_const=(1,0,0,0)):

    new_mat = bpy.data.materials.new(name=nameID)
    new_mat.use_nodes = True
    if new_mat.node_tree:
        new_mat.node_tree.links.clear()
        new_mat.node_tree.nodes.clear() 
    nodes = new_mat.node_tree.nodes
    links = new_mat.node_tree.links
    output = nodes.new(type='ShaderNodeOutputMaterial')
    shader = nodes.new(type='ShaderNodeBsdfPrincipled')
    shader.inputs[0].default_value = color_const
    links.new(shader.outputs[0], output.inputs[0])
    return new_mat

# Helper function to generate a multi-color material. Operates in two situations:
# - to colorize meshes: once assigned to a mesh with a float or vector attribute, 
# renders it as color (per face or per vertex, depending on the attribute's mesh data domain).
# - to colorize n object instances made by geometry node: if so, the material must be applied
# to the object used as sample for instances, and use per_instance=True. Instances must have the attribute.
def make_material_color_attribute(nameID, colname, per_instance=False):

    new_mat = bpy.data.materials.new(name=nameID)
    new_mat.use_nodes = True
    if new_mat.node_tree:
        new_mat.node_tree.links.clear()
        new_mat.node_tree.nodes.clear() 
    nodes = new_mat.node_tree.nodes
    links = new_mat.node_tree.links
    output = nodes.new(type='ShaderNodeOutputMaterial')
    shader = nodes.new(type='ShaderNodeBsdfPrincipled')
    links.new(shader.outputs[0], output.inputs[0])
    colattribute = nodes.new(type='ShaderNodeAttribute')
    colattribute.attribute_name = colname
    if not per_instance: 
        colattribute.attribute_type = 'GEOMETRY'
    else:
        colattribute.attribute_type = 'INSTANCER'
    links.new(colattribute.outputs[0], shader.inputs[0])
    return new_mat
    
# some colormaps used by  make_material_falsecolor
colormap_jet = [
 [0,     (0,0,0.56, 1)],
 [0.125, (0,0,1 ,1)   ],
 [0.375, (0,1,1, 1)   ],
 [0.625, (1,1,0, 1 )  ],
 [0.875, (1,0,0, 1 )  ],
 [1,     (0.5,0,0, 1) ]
]
colormap_cooltowarm = [
 [0,         	(0.229999504,	0.298998934,	0.754000139, 1) ],
 [0.142857143,	(0.406839976,	0.537716815,	0.934353077, 1) ],
 [0.285714286,	(0.602704657,	0.731255644,    0.999993038, 1) ],
 [0.428571429,  (0.78841419,	0.845877766,	0.939423093, 1) ],
 [0.571428571,	(0.930635713,	0.820337799,	0.761004578, 1) ], 
 [0.714285714,	(0.967788492,	0.657029313,	0.537326447, 1) ],
 [0.857142857,	(0.88710666,	0.413948424,	0.324564482, 1) ],
 [1	,           (0.706000136,   0.015991824,	0.150000072, 1) ],
]
colormap_viridis = [
 [0,           (0.267003985,    0.004872566,    0.329415069, 1) ],
 [0.142857143, (0.274741032,	0.196973267,	0.497250443, 1) ],
 [0.285714286, (0.212671237,	0.359101377, 	0.551635047, 1) ],
 [0.428571429, (0.152958099,	0.498051451,	0.557685327, 1) ],
 [0.571428571, (0.122053592,	0.632105543,	0.530848866, 1) ],
 [0.714285714, (0.290013937,	0.758845119,	0.427827161, 1) ],
 [0.857142857, (0.622182341,	0.853814293,	0.226247911, 1) ],
 [1,           (0.993248149,	0.906154763,	0.143935944, 1) ]
]
colormap_cool = [
 [0,     (0,1,1, 1)],
 [1,     (1,0,1, 1)]
]
colormap_hot = [
 [0,     (0.039,0,0, 1)],
 [0.375, (1,0,0, 1)    ],
 [0.75,  (1,1,0, 1)    ],
 [1,     (1,1,1, 1)    ]
]
colormap_winter = [
 [0,     (0,0,1, 1)  ],
 [1,     (0,1,0.5, 1)]
]

colormaps = {
"colormap_jet":colormap_jet,
"colormap_cooltowarm":colormap_cooltowarm,
"colormap_viridis":colormap_viridis,
"colormap_hot":colormap_hot,
"colormap_cool":colormap_cool,
"colormap_winter":colormap_winter,
}


# Helper function to generate a falsecolor material. Operates in two situations:
# - to colorize mehes: once assigned to a mesh with a float or vector attribute, 
# renders it as falsecolor (per face or per vertex, depending on the attribute's mesh data domain).
# - to colorize n object instances made by geometry node: if so, the material must be applied
# to the object used as sample for instances, and use per_instance=True. Instances must have the attribute.
def make_material_falsecolor(nameID, attrname, attr_min=0.0, attr_max=1.0, colormap=colormap_cooltowarm, per_instance=False):
    
    new_mat = bpy.data.materials.new(name=nameID)
    new_mat.use_nodes = True
    if new_mat.node_tree:
        new_mat.node_tree.links.clear()
        new_mat.node_tree.nodes.clear()  
    nodes = new_mat.node_tree.nodes
    links = new_mat.node_tree.links
    output = nodes.new(type='ShaderNodeOutputMaterial')
    shader = nodes.new(type='ShaderNodeBsdfPrincipled')
    links.new(shader.outputs[0], output.inputs[0])
    ramp = nodes.new(type='ShaderNodeValToRGB')
    ramp.color_ramp.color_mode = 'HSV'
    for i in range(1,len(colormap)-1):
        ramp.color_ramp.elements.new(colormap[i][0])
    for i in range(0,len(colormap)):
        ramp.color_ramp.elements[i].color = (colormap[i][1]) 
    links.new(ramp.outputs[0], shader.inputs[0])
    maprange = nodes.new(type='ShaderNodeMapRange')
    maprange.inputs[1].default_value = attr_min
    maprange.inputs[2].default_value = attr_max
    links.new(maprange.outputs[0], ramp.inputs[0])
    vectnorm = nodes.new(type="ShaderNodeVectorMath")
    vectnorm.operation = 'LENGTH'
    links.new(vectnorm.outputs[1], maprange.inputs[0])
    multiply = nodes.new(type="ShaderNodeVectorMath")
    multiply.operation = 'MULTIPLY'
    multiply.name = 'Multiply'
    multiply.inputs[1].default_value = (1,1,1)
    links.new(multiply.outputs[0], vectnorm.inputs[0])
    colattribute = nodes.new(type='ShaderNodeAttribute')
    colattribute.attribute_name = attrname
    if not per_instance: 
        colattribute.attribute_type = 'GEOMETRY'
    else:
        colattribute.attribute_type = 'INSTANCER'
    links.new(colattribute.outputs[1], multiply.inputs[0])
    return new_mat



# Add an item to collection of settings about mesh visualization, but
# if a mesh setting is already existing with the correspoiding name id, just reuse it (so
# that one can have setting persistency through frames, even if the mesh is non-mutable and regenerated.
def setup_meshsetting(new_object):
    meshsetting = bpy.context.scene.ch_meshsetting.get(new_object.name)
    if not meshsetting:
        meshsetting = bpy.context.scene.ch_meshsetting.add()
        meshsetting.name = new_object.name
        meshsetting.property_index = 0
    return meshsetting

# Add an item to collection of settings about glyphs visualization, but
# if a glyph setting is already existing with the correspoiding name id, just reuse it (so
# that one can have setting persistency through frames, even if the mesh is non-mutable and regenerated.
def setup_glyph_setting(obj_name,
                           glyph_type = 'POINT',
                           color_type = 'CONST',
                           property_index_color = 0,
                           const_color =(1,0,0),
                           dir_type = 'CONST',
                           const_dir = (1,0,0),
                           property_index_dir = 0,
                           length_type = 'CONST',
                           property_index_length = 0,
                           length_scale = 0.01, 
                           width_type = 'CONST',
                           property_index_width = 0,
                           width_scale=0.01,
                           basis_type = 'CONST',               
                           property_index_basis = 0,
                           const_basis=(1,0,0,0),
                           eigenvalues_type = 'CONST',
                           const_eigenvalues = (1,1,1),
                           property_index_eigenvalues = 0,
                           do_tip = True
                           ):
    glyphsetting = bpy.context.scene.ch_glyph_setting.get(obj_name)
    if not glyphsetting:
        glyphsetting = bpy.context.scene.ch_glyph_setting.add()
        glyphsetting.name = obj_name
        glyphsetting.glyph_type = glyph_type
        glyphsetting.property_index_dir = property_index_dir
        glyphsetting.property_index_length = property_index_length
        glyphsetting.property_index_width  = property_index_width
        glyphsetting.property_index_color  = property_index_color
        glyphsetting.property_index_basis  = property_index_basis
        glyphsetting.property_index_eigenvalues  = property_index_eigenvalues
        glyphsetting.dir_type = dir_type
        glyphsetting.length_type = length_type
        glyphsetting.width_type = width_type
        glyphsetting.color_type = color_type
        glyphsetting.basis_type = basis_type
        glyphsetting.eigenvalues_type = eigenvalues_type
        glyphsetting.length_scale = length_scale
        glyphsetting.width_scale = width_scale
        glyphsetting.const_color = const_color
        glyphsetting.const_dir = const_dir
        glyphsetting.const_basis = const_basis
        glyphsetting.const_eigenvalues = const_eigenvalues
        glyphsetting.use_x = True
        glyphsetting.use_y = True
        glyphsetting.use_z = True
        glyphsetting.do_tip = do_tip
        glyphsetting.material = make_material_color_const(obj_name+"_color")
    return glyphsetting

    
# Add an item to collection of properties of a given mesh visualization,
# but if a property is already existing with the corresponding name id, just reuse it
def setup_property_scalar(meshsetting, propname, min=0, max=1, mcolormap='colormap_viridis', matname='a_falsecolor', per_instance=False):
    property = meshsetting.property.get(propname)
    if not property:
        property = meshsetting.property.add()
        property.name = propname
        property.min = min
        property.max = max
        property.colorm = mcolormap
        property.type = 'SCALAR'
        property.mat = make_material_falsecolor(matname, propname, min, max, colormaps[mcolormap], per_instance=per_instance)
    return property

# Add an item to collection of properties of a given mesh visualization,
# but if a property is already existing with the corresponding name id, just reuse it
def setup_property_color(meshsetting, colname, matname='a_meshcolor', per_instance=False):
    property = meshsetting.property.get(colname)
    if not property:
        property = meshsetting.property.add()
        property.name = colname
        property.type = 'COLOR'
        property.mat = make_material_color_attribute(matname, colname, per_instance=per_instance)
    return property

# Add an item to collection of properties of a given mesh visualization,
# but if a property is already existing with the corresponding name id, just reuse it
def setup_property_vector(meshsetting, propname, min=0, max=1, mvectplot='NORM', mcolormap='colormap_viridis', matname='a_falsecolor', per_instance=False):
    property = meshsetting.property.get(propname)
    if not property:
        property = meshsetting.property.add()
        property.name = propname
        property.min = min
        property.max = max
        property.colorm = mcolormap
        property.type = 'VECTOR'
        property.vect_plot = mvectplot
        property.mat = make_material_falsecolor(matname, propname, min, max, colormaps[mcolormap], per_instance=per_instance)
    return property

# Add an item to collection of properties of a given mesh visualization,
# but if a property is already existing with the corresponding name id, just reuse it
def setup_property_quaternion(meshsetting, propname, min=0, max=1, mvectplot='NORM', mcolormap='colormap_viridis', matname='a_falsecolor', per_instance=False):
    property = meshsetting.property.get(propname)
    if not property:
        property = meshsetting.property.add()
        property.name = propname
        property.min = min
        property.max = max
        property.colorm = mcolormap
        property.type = 'QUATERNION'
        property.mat = make_material_falsecolor(matname, propname, min, max, colormaps[mcolormap], per_instance=per_instance)
    return property

# Attach a falsecolor material to the object, and set properties of falsecolor
def update_meshsetting_falsecolor_material(new_object, meshsetting, propname):
    mat = None
    if (meshsetting.property_index >=0): 
        selected_prop = meshsetting.property[meshsetting.property_index]
        if (selected_prop.name ==propname):
            mat = selected_prop.mat
            mat.node_tree.nodes['Map Range'].inputs[1].default_value = selected_prop.min
            mat.node_tree.nodes['Map Range'].inputs[2].default_value = selected_prop.max
            ramp = mat.node_tree.nodes['ColorRamp']
            colormap = colormaps[selected_prop.colorm]
            for i in range(0,len(ramp.color_ramp.elements)-2):
                ramp.color_ramp.elements.remove(ramp.color_ramp.elements[1]) # leave start-end. Also, clear() doe not exist
            for i in range(1,len(colormap)-1):
                ramp.color_ramp.elements.new(colormap[i][0])
            for i in range(0,len(colormap)):
                ramp.color_ramp.elements[i].color = (colormap[i][1]) 
            if selected_prop.type=='VECTOR': #or selected_prop.type=='SCALAR':
                mat.node_tree.nodes['Multiply'].inputs[1].default_value = (1,1,1)
                if selected_prop.type=='VECTOR':
                    links = mat.node_tree.links
                    if selected_prop.vect_plot =='X':
                        mat.node_tree.nodes['Multiply'].inputs[1].default_value = (1,0,0)
                    if selected_prop.vect_plot =='Y':
                        mat.node_tree.nodes['Multiply'].inputs[1].default_value = (0,1,0)
                    if selected_prop.vect_plot =='Z':
                        mat.node_tree.nodes['Multiply'].inputs[1].default_value = (0,0,1)
            new_object.data.materials.clear()
            new_object.data.materials.append(mat)
    return mat

# Attach a color material to the object, and set properties 
def update_meshsetting_color_material(new_object, meshsetting, propname):
    mat = None
    if (meshsetting.property_index >=0): 
        selected_prop = meshsetting.property[meshsetting.property_index]
        if (selected_prop.name ==propname):
            mat = selected_prop.mat
            new_object.data.materials.clear()
            new_object.data.materials.append(mat)
    return mat

# Attach a falsecolor material to the glyphs, and set properties 
def update_glyphsetting_material(new_object, meshsetting, propname):
    mat = None
    if (meshsetting.property_index_color >=0): 
        selected_prop = meshsetting.property[meshsetting.property_index_color]
        if (selected_prop.name ==propname):
            mat = selected_prop.mat
            if mat.node_tree.nodes.get('Map Range'):
                mat.node_tree.nodes['Map Range'].inputs[1].default_value = selected_prop.min
                mat.node_tree.nodes['Map Range'].inputs[2].default_value = selected_prop.max
                ramp = mat.node_tree.nodes['ColorRamp']
                colormap=[]
                if meshsetting.color_type == 'CONST':
                    colormap = [[0,(meshsetting.const_color.r,meshsetting.const_color.g,meshsetting.const_color.b,1)],[1,(meshsetting.const_color.r,meshsetting.const_color.g,meshsetting.const_color.b,1)]]
                if meshsetting.color_type == 'PROPERTY':
                    colormap = colormaps[selected_prop.colorm]
                for i in range(0,len(ramp.color_ramp.elements)-2):
                    ramp.color_ramp.elements.remove(ramp.color_ramp.elements[1]) # leave start-end. Also, clear() doe not exist
                for i in range(1,len(colormap)-1):
                    ramp.color_ramp.elements.new(colormap[i][0])
                for i in range(0,len(colormap)):
                    ramp.color_ramp.elements[i].color = (colormap[i][1]) 
                if selected_prop.type=='VECTOR': #or selected_prop.type=='SCALAR':
                    mat.node_tree.nodes['Multiply'].inputs[1].default_value = (1,1,1)
                    if selected_prop.type=='VECTOR':
                        links = mat.node_tree.links
                        if selected_prop.vect_plot =='X':
                            mat.node_tree.nodes['Multiply'].inputs[1].default_value = (1,0,0)
                        if selected_prop.vect_plot =='Y':
                            mat.node_tree.nodes['Multiply'].inputs[1].default_value = (0,1,0)
                        if selected_prop.vect_plot =='Z':
                            mat.node_tree.nodes['Multiply'].inputs[1].default_value = (0,0,1)
            new_object.data.materials.clear()
            new_object.data.materials.append(mat)
    return mat



def make_chrono_csys(mpos,mrot, mparent, symbol_scale=0.1):
    csys = bpy.data.objects['chrono_csys']
    csys_obj = csys.copy() # instanced, no full copy, data is shared
    csys_obj.rotation_mode = 'QUATERNION'
    csys_obj.rotation_quaternion = mrot
    csys_obj.location = mpos
    csys_obj.scale = (symbol_scale,symbol_scale,symbol_scale)
    if mparent:
        csys_obj.parent = mparent
    csys_obj.select_set(False)
    return csys_obj


def make_chrono_object_assetlist(mname,mpos,mrot, masset_list):

    chobject = bpy.data.objects.new(mname, empty_mesh)  
    chobject.rotation_mode = 'QUATERNION'
    chobject.rotation_quaternion = mrot
    chobject.location = mpos
    chrono_frame_objects.objects.link(chobject)
    
    if chrono_view_item_csys:
        mcsys = make_chrono_csys((0,0,0),(1,0,0,0), chobject, chrono_view_item_csys_size)
        chrono_frame_objects.objects.link(mcsys)
        
    for m in range(len(masset_list)):
        masset = chrono_assets.objects.get(masset_list[m][0])
        if not masset:
            masset = chrono_frame_assets.objects.get(masset_list[m][0])
        if masset:
            chasset = masset.copy() # instanced, no full copy, masset.data is shared
            chasset.rotation_mode = 'QUATERNION'
            chasset.rotation_quaternion = masset_list[m][2]
            chasset.location = masset_list[m][1]
            if len(masset_list[m])>4:
                chasset.scale = masset_list[m][4]
            chrono_frame_objects.objects.link(chasset)
            chasset.parent = chobject
            if chrono_view_materials:
                while len(chasset.data.materials) < len(masset_list[m][3]):
                    chasset.data.materials.append(None)
                for i, matname in enumerate(masset_list[m][3]):
                    chasset.material_slots[i].link = 'OBJECT'
                    chasset.material_slots[i].material = bpy.data.materials[matname]
            if chrono_view_asset_csys:
                mcsys = make_chrono_csys(chasset.location,chasset.rotation_quaternion, chobject, chrono_view_asset_csys_size)
                chrono_frame_objects.objects.link(mcsys) 
        else:
            print("not found asset: ",masset_list[m][0])
    
    
def make_chrono_object_clones(mname,mpos,mrot, 
                                masset_list, 
                                list_clones_posrot):
    
    chobject = bpy.data.objects.new(mname, empty_mesh)  
    chobject.rotation_mode = 'QUATERNION'
    chobject.rotation_quaternion = mrot
    chobject.location = mpos
    chrono_frame_objects.objects.link(chobject)
    
    chassets_group = bpy.data.objects.new("assets_group", empty_mesh)  
    chrono_frame_objects.objects.link(chassets_group)
    chassets_group.parent = chobject
    chassets_group.hide_set(True)
    chassets_group.hide_render = True
    
    if chrono_view_item_csys:
        mcsys = make_chrono_csys((0,0,0),(1,0,0,0), chassets_group, chrono_view_item_csys_size)
        chrono_frame_objects.objects.link(mcsys)
    
    for m in range(len(masset_list)):
        masset = chrono_assets.objects.get(masset_list[m][0])
        if not masset:
            masset = chrono_frame_assets.objects.get(masset_list[m][0])
        if masset:
            chasset = masset.copy() # instanced, no full copy, masset.data is shared
            chasset.rotation_mode = 'QUATERNION'
            chasset.rotation_quaternion = masset_list[m][2]
            chasset.location = masset_list[m][1]
            if len(masset_list[m])>4:
                chasset.scale = masset_list[m][4]
            chrono_frame_objects.objects.link(chasset)
            chasset.parent = chassets_group
            if chrono_view_materials:
                while len(chasset.data.materials) < len(masset_list[m][3]):
                    chasset.data.materials.append(None)
                for i, matname in enumerate(masset_list[m][3]):
                    chasset.material_slots[i].link = 'OBJECT'
                    chasset.material_slots[i].material = bpy.data.materials[matname]
            if chrono_view_asset_csys:
                mcsys = make_chrono_csys(chasset.location,chasset.rotation_quaternion, chassets_group, chrono_view_asset_csys_size)
                chrono_frame_objects.objects.link(mcsys)
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
    
    
   
def make_chrono_glyphs_objects(mname,mpos,mrot, 
                        masset_list, 
                        list_clones_posrot, 
                        list_attributes, 
                        color_attr_name="", 
                        color_min=0, color_max=1, 
                        def_colormap=colormap_cooltowarm):
    
    if len(list_clones_posrot) == 0:
        return None
    
    chcollection = None
    chasset = None
    new_objects =[] # to return a pair of [sample instance, geometry node instancer]
    
    if len(masset_list) == 1:
        # simplified if just one object
        mfalsecolor=None
        if color_attr_name:
            mfalsecolor = make_material_falsecolor(mname+'_falsecolor', color_attr_name, color_min, color_max, def_colormap, per_instance=True)
        
        masset = None
        if type(masset_list[0][0]) == str:
            masset = chrono_assets.objects.get(masset_list[0][0])
            if not masset:
                masset = chrono_frame_assets.objects.get(masset_list[0][0])
        else:
            masset = masset_list[0][0]
             
        chasset = masset.copy() 
        chasset.data = masset.data.copy() # full copy?
        
        chrono_frame_objects.objects.link(chasset)
        
        while len(chasset.data.materials) < 1:
            chasset.data.materials.append(None)
        chasset.material_slots[0].link = 'OBJECT'
        chasset.material_slots[0].material = mfalsecolor
        chasset.hide_set(True)
        
        new_objects.append(chasset)
                
    else:        # for multiple assets, geometry nodes must use a collection instance, so wrap them
        chcollection = bpy.data.collections.new(mname+'_glyph_instance')
        chrono_frame_objects.children.link(chcollection)
        
        chassets_group = bpy.data.objects.new("glyph_assets_group", empty_mesh)  
        chcollection.objects.link(chassets_group)
        chassets_group.hide_set(True)
        chassets_group.hide_render = True
        chassets_group.hide_viewport = True

        mfalsecolor=None
        if color_attr_name:
            mfalsecolor = make_material_falsecolor(mname+'_falsecolor', color_attr_name, color_min, color_max, def_colormap, per_instance=True)
        
        for m in range(len(masset_list)):
            masset = chrono_assets.objects.get(masset_list[m][0])
            if not masset:
                masset = chrono_frame_assets.objects.get(masset_list[m][0])
            if masset:
                chasset = masset.copy() 
                chasset.data = masset.data.copy() # full copy?
                chasset.rotation_mode = 'QUATERNION'
                chasset.rotation_quaternion = masset_list[m][2]
                chasset.location = masset_list[m][1]
                if len(masset_list[m])>4:
                    chasset.scale = masset_list[m][4]
                chcollection.objects.link(chasset)
                chasset.parent = chassets_group                
                #if chrono_view_materials:
                #    while len(chasset.data.materials) < len(masset_list[m][3]):
                #        chasset.data.materials.append(None)
                #    for i, matname in enumerate(masset_list[m][3]):
                #        chasset.material_slots[i].link = 'OBJECT'
                #        chasset.material_slots[i].material = bpy.data.materials[matname]
                #if chrono_view_asset_csys:
                #    mcsys = make_chrono_csys(chasset.location,chasset.rotation_quaternion, chassets_group, chrono_view_asset_csys_size)
                #    chrono_frame_objects.objects.link(mcsys)
                while len(chasset.data.materials) < 1:
                    chasset.data.materials.append(None)
                chasset.material_slots[0].link = 'OBJECT'
                chasset.material_slots[0].material = mfalsecolor
                chasset.hide_set(True)    
                new_objects.append(chasset)
            else:
                print("not found asset for glyphs: ",masset_list[m][0])
            
        
    ncl = len(list_clones_posrot)
    verts = [(0,0,0)] * (ncl)
    faces = [] 
    edges = []
    for ic in range(ncl):
        verts[ic] = list_clones_posrot[ic][0]
    new_mesh = bpy.data.meshes.new('mesh_position_glyphs')
    new_mesh.from_pydata(verts, edges, faces)
    new_mesh.update()

    chobject = bpy.data.objects.new(mname, new_mesh)  
    chobject.rotation_mode = 'QUATERNION'
    chobject.rotation_quaternion = mrot
    chobject.location = mpos
    chrono_frame_objects.objects.link(chobject)
    
    chobject.modifiers.new(name="Chrono clones", type='NODES')
    node_group = bpy.data.node_groups.new('GeometryNodes', 'GeometryNodeTree')
    inNode = node_group.nodes.new('NodeGroupInput')
    node_group.outputs.new('NodeSocketGeometry', 'Geometry')
    outNode = node_group.nodes.new('NodeGroupOutput')
    node_group.inputs.new('NodeSocketGeometry', 'Geometry')

    if len(masset_list) == 1:
        node_objinfo = node_group.nodes.new('GeometryNodeObjectInfo') 
        node_objinfo.inputs[0].default_value = chasset
        node_objinfo.inputs[1].default_value = True
    else:
        node_objinfo = node_group.nodes.new('GeometryNodeCollectionInfo') 
        node_objinfo.inputs[0].default_value = chcollection 

    node_onpoints = node_group.nodes.new('GeometryNodeInstanceOnPoints')

    node_group.links.new(inNode.outputs['Geometry'], node_onpoints.inputs['Points'])
    node_group.links.new(node_onpoints.outputs['Instances'], outNode.inputs['Geometry'])
    node_group.links.new(node_objinfo.outputs['Geometry'], node_onpoints.inputs['Instance'])

    # optional rotation 
    if (len(list_clones_posrot[0])>1 and len(list_clones_posrot[0][1])==3) :
        rot_attr = [(0,0,0)] * (ncl)
        for ic in range(ncl):
            rot_attr[ic] = list_clones_posrot[ic][1]
        add_mesh_data_vectors(chobject, rot_attr, 'ch_rot', mdomain='POINT')
        node_namattr = node_group.nodes.new('GeometryNodeInputNamedAttribute')
        node_namattr.inputs['Name'].default_value = 'ch_rot'
        node_namattr.data_type = 'FLOAT_VECTOR'
        node_group.links.new(node_namattr.outputs['Attribute'], node_onpoints.inputs['Rotation'])
        
    # optional scale 
    if (len(list_clones_posrot[0])>2 and len(list_clones_posrot[0][2])==3) :
        scale_attr = [(1,1,1)] * (ncl)
        for ic in range(ncl):
            scale_attr[ic] = list_clones_posrot[ic][2]
        add_mesh_data_vectors(chobject, scale_attr, 'ch_scale', mdomain='POINT')
        node_namattr = node_group.nodes.new('GeometryNodeInputNamedAttribute')
        node_namattr.inputs['Name'].default_value = 'ch_scale'
        node_namattr.data_type = 'FLOAT_VECTOR'
        node_group.links.new(node_namattr.outputs['Attribute'], node_onpoints.inputs['Scale'])
    
    # optional scalar or vector per-point properties for falsecolor 
    for ia in range(len(list_attributes)):
        if type(list_attributes[ia][1][0]) == tuple:
            if len(list_attributes[ia][1][0])==3:
                my_attr = [(0,0,0)] * (ncl)
                for ic in range(ncl):
                    my_attr[ic] = list_attributes[ia][1][ic]
                add_mesh_data_vectors(chobject, my_attr, list_attributes[ia][0], mdomain='POINT')
        else:
            my_attr = [0] * (ncl)
            for ic in range(ncl):
                my_attr[ic] = list_attributes[ia][1][ic]
            add_mesh_data_floats(chobject, my_attr, list_attributes[ia][0], mdomain='POINT')

    chobject.modifiers[-1].node_group = node_group
    
    new_objects.append(chobject)
    
    return new_objects
 
    
# Macro to generate vector glyphs, placed at position in list 'list_clones_pos'.
# One can pass a list of attributes each in ["my_attr_name", [value,value,value,...]] format, 
# where value can be float, (x,y,z) vector tuple, quaternion tuple, etc.
# Vectors are assumed with x y z from 'dir' property in same absolute space, 
# but if a 'basis' parameter is passed with quaternion rotations of n frames, 
# then each dir is assumed with x y z expressed in the basis of that local frames. 
def make_chrono_glyphs_vectors(mname,mpos,mrot, 
                          list_clones_pos, 
                          list_attributes=[],
                          dir=mathutils.Vector((1,0,0)),  # mathutils.Vector, or name of some vector attribute with list of (x,y,z)
                          basis=mathutils.Quaternion((1,0,0,0)), # mathutils.Quaternion const, or name of some quaternion property with list of (w,x,y,z)
                          length=0.1,               # float const, or float prop., or vector prop. names, ex. if "V" then arrow length=|V|*length_factor, if "my_float" arrow lenght=my_float*length_factor
                          length_factor = 1.0,      # length=|v|*length_factor
                          color='',                 # (r,g,b)color, or float prop. ex. "my_float". In the second case, the falsecolor of colormap will be used.
                          color_min=0, color_max=1, 
                          colormap=colormap_cooltowarm,    
                          thickness = 0.1,         # float const, or float prop. name, ex. if "my_float" then arrow width =my_float*length_factor
                          thickness_factor = 0.01,  # if thickness not const, then thickness=|v|*thickness_factor
                          do_arrow_tip = True,      # vector extends with a pointed cone
                          components = (True,True,True) # to select only x,y,z components, or xy xz yz pairs. Default xyz. Affects both 'dir' and 'lenght' (if vector property)
                          ): 
    new_objects = []
    vcomponents = mathutils.Vector(components)
    ncl = len(list_clones_pos)
    if ncl == 0:
        return []
    list_clones_posrotscale = np.empty((ncl, 3,3)).tolist()
    
    # default fallbacks if inputs are wrong or missing
    mdirection = mathutils.Vector((1,0,0))
    mbasis     = mathutils.Quaternion((1,0,0,0))
    mthickness = 0.002
    mlength    = 0.01 
    # cases where dir,len,thickness,rot are given via floats or vectors etc, not via "my_attr" ids
    mdirection = dir if not type(dir)==str else mdirection
    mbasis = basis if not type(basis)==str else mbasis
    mthickness = thickness if not type(thickness)==str else mthickness
    mlength = length if not type(length)==str else mlength 
    # cases where dir,len,thickness,rot are given via "my_attr" ids
    attr_dir = [i for i in list_attributes if i[0]==dir]
    attr_dir_isscalar = True if (attr_dir and type(attr_dir[0][1][0]) in (int,float)) else False
    attr_dir_isvect   = True if (attr_dir and not attr_dir_isscalar and len(attr_dir[0][1][0]) == 3) else False
    attr_basis = [i for i in list_attributes if i[0]==basis]
    attr_basis_isquat   = True if (attr_basis and len(attr_basis[0][1][0]) == 4) else False
    attr_thickness = [i for i in list_attributes if i[0]==thickness]
    attr_thickness_isscalar = True if (attr_thickness and type(attr_thickness[0][1][0]) in (int,float)) else False
    attr_thickness_isvect   = True if (attr_thickness and not attr_thickness_isscalar and len(attr_thickness[0][1][0]) == 3) else False
    attr_length = [i for i in list_attributes if i[0]==length]
    attr_length_isscalar = True if (attr_length and type(attr_length[0][1][0]) in (int,float)) else False
    attr_length_isvect   = True if (attr_length and not attr_length_isscalar and len(attr_length[0][1][0]) == 3) else False
    
    for i in range(ncl):
        if attr_dir_isvect:
            mdirection = mathutils.Vector(attr_dir[0][1][i])* vcomponents
        if attr_dir_isscalar:
            mdirection = mathutils.Vector((attr_dir[0][1][i],0,0))

        if attr_thickness_isvect:
            mthickness =(mathutils.Vector(attr_thickness[0][1][i])* vcomponents).length*thickness_factor
        if attr_thickness_isscalar:
            mthickness = attr_thickness[0][1][i]*thickness_factor
            
        if attr_length_isvect:
            mlength = (mathutils.Vector(attr_length[0][1][i])* vcomponents).length*length_factor
        if attr_length_isscalar:
            mlength = attr_length[0][1][i]*length_factor
        
        if attr_basis_isquat:
            mbasis = mathutils.Quaternion(attr_basis[0][1][i])
            
        tot_rot =  mbasis @ mdirection.to_track_quat('Z', 'Y') 
        
        list_clones_posrotscale[i][0] = list_clones_pos[i]
        list_clones_posrotscale[i][1] = tot_rot.to_euler()
        list_clones_posrotscale[i][2] = (mthickness,mthickness, mlength)
    my_o = make_chrono_glyphs_objects(mname,mpos,mrot,
     [
      [bpy.data.objects['chrono_cylinder'], (0,0,0), (1,0,0,0), ""]
     ],
     list_clones_posrotscale,
     list_attributes,
     color, color_min, color_max, colormap
     )
    new_objects.append(my_o)
    if do_arrow_tip:
        list_clones_posrotscale = np.empty((ncl, 3,3)).tolist()
        for i in range(ncl):
            if attr_dir_isvect:
                mdirection = mathutils.Vector(attr_dir[0][1][i])* vcomponents
            if attr_dir_isscalar:
                mdirection = mathutils.Vector((attr_dir[0][1][i],0,0))

            if attr_thickness_isvect:
                mthickness = (mathutils.Vector(attr_thickness[0][1][i])* vcomponents).length*thickness_factor
            if attr_thickness_isscalar:
                mthickness = attr_thickness[0][1][i]*thickness_factor
                
            if attr_length_isvect:
                mlength = (mathutils.Vector(attr_length[0][1][i])* vcomponents).length*length_factor
            if attr_length_isscalar:
                mlength = attr_length[0][1][i]*length_factor
                
            if attr_basis_isquat:
                mbasis = mathutils.Quaternion(attr_basis[0][1][i])
                
            tot_rot = mbasis @ mdirection.to_track_quat('Z', 'Y')
            
            displ = mdirection.normalized()*mlength
            displ.rotate(mbasis)
            
            list_clones_posrotscale[i][0] = mathutils.Vector(list_clones_pos[i])+displ
            list_clones_posrotscale[i][1] = tot_rot.to_euler()
            list_clones_posrotscale[i][2] = (mthickness*1.4,mthickness*1.4,mthickness*2)
        my_o = make_chrono_glyphs_objects(mname+'_tip',mpos,mrot,
         [
          [bpy.data.objects['chrono_cone'], (0,0,0), (1,0,0,0), ""]
         ],
         list_clones_posrotscale,
         list_attributes,
         color, color_min, color_max, colormap
         )
        new_objects.append(my_o)
    return new_objects
        
# Macro to generate point glyphs, placed at position in list 'list_clones_pos'.
# One can pass a list of attributes each in ["my_attr_name", [value,value,value,...]] format, 
# where item can be float, (x,y,z) vector tuple, etc.
def make_chrono_glyphs_points(mname,mpos,mrot, 
                          list_clones_pos, 
                          list_attributes=[], 
                          color='', 
                          color_min=0, color_max=1, 
                          colormap=colormap_cooltowarm, 
                          point_geometry = 'chrono_sphere',
                          thickness = 0.1, # float const, or if='my_attr' then dot thickness=my_attr*thickness_factor
                          thickness_factor = 0.01,  # if thickness not const, then thickness=my_attr*thickness_factor
                          ): 
    new_objects = []
    ncl = len(list_clones_pos)
    if ncl == 0:
        return []
    list_clones_posrotscale = np.empty((ncl, 3,3)).tolist()
    
    # default fallbacks if inputs are wrong or missing
    mthickness = 0.002
    # cases where dir,len,thickness,rot are given via floats or vectors etc, not via "my_attr" ids
    mthickness = thickness if not type(thickness)==str else mthickness
    # cases where dir,len,thickness,rot are given via "my_attr" ids
    attr_thickness = [i for i in list_attributes if i[0]==thickness]
    attr_thickness_isscalar = True if (attr_thickness and type(attr_thickness[0][1][0]) in (int,float)) else False
    attr_thickness_isvect   = True if (attr_thickness and not attr_thickness_isscalar and len(attr_thickness[0][1][0]) == 3) else False

    for i in range(ncl):
        if attr_thickness_isvect:
            mthickness = mathutils.Vector(attr_thickness[0][1][i]).length*thickness_factor
        if attr_thickness_isscalar:
            mthickness = attr_thickness[0][1][i]*thickness_factor
            
        list_clones_posrotscale[i][0] = list_clones_pos[i]
        list_clones_posrotscale[i][1] = (0,0,0)
        list_clones_posrotscale[i][2] = (mthickness,mthickness,mthickness)
    my_o = make_chrono_glyphs_objects(mname,mpos,mrot,
     [
      [bpy.data.objects[point_geometry], (0,0,0), (1,0,0,0), ""]
     ],
     list_clones_posrotscale,
     list_attributes,
     color, color_min, color_max, colormap
     )
    new_objects.append(my_o)
    return new_objects

# Macro to generate coordsys glyphs, placed at position in list 'list_clones_pos'.
# Coordsystems are assumed basis from 'basis' property passed with quaternion rotations.
# One can pass a list of attributes each in ["my_attr_name", [value,value,value,...]] format, 
# where item can be float, (x,y,z) vector tuple, etc.
def make_chrono_glyphs_coordsys(mname,mpos,mrot, 
                          list_clones_pos, 
                          list_attributes=[], 
                          basis=mathutils.Quaternion((1,0,0,0)), # mathutils.Quaternion const, or name of some quaternion property with list of (w,x,y,z)
                          eigenvalues=mathutils.Vector((1,1,1)), 
                          color='', 
                          color_min=0, color_max=1, 
                          colormap=colormap_cooltowarm, 
                          coordsys_geometry = 'chrono_csys',
                          thickness = 0.1, # float const, or if='my_attr' then coordsys thickness=my_attr*thickness_factor
                          thickness_factor = 0.01,  # if thickness not const, then thickness=my_attr*thickness_factor
                          ): 
    new_objects = []
    ncl = len(list_clones_pos)
    if ncl == 0:
        return []
    list_clones_posrotscale = np.empty((ncl, 3,3)).tolist()
    
    # default fallbacks if inputs are wrong or missing
    mthickness = 0.002
    mbasis     = mathutils.Quaternion((1,0,0,0))
    meigs     = mathutils.Vector((1,1,1))
    # cases where dir,len,thickness,rot are given via floats or vectors etc, not via "my_attr" ids
    mthickness = thickness if not type(thickness)==str else mthickness
    mbasis = basis if not type(basis)==str else mbasis
    meigs = eigenvalues if not type(eigenvalues)==str else meigs
    # cases where dir,len,thickness,rot are given via "my_attr" ids
    attr_thickness = [i for i in list_attributes if i[0]==thickness]
    attr_thickness_isscalar = True if (attr_thickness and type(attr_thickness[0][1][0]) in (int,float)) else False
    attr_thickness_isvect   = True if (attr_thickness and not attr_thickness_isscalar and len(attr_thickness[0][1][0]) == 3) else False
    attr_eigenvalues = [i for i in list_attributes if i[0]==eigenvalues]
    attr_eigenvalues_isscalar = True if (attr_eigenvalues and type(attr_eigenvalues[0][1][0]) in (int,float)) else False
    attr_eigenvalues_isvect   = True if (attr_eigenvalues and not attr_eigenvalues_isscalar and len(attr_eigenvalues[0][1][0]) == 3) else False
    attr_basis = [i for i in list_attributes if i[0]==basis]
    attr_basis_isquat   = True if (attr_basis and len(attr_basis[0][1][0]) == 4) else False
    
    for i in range(ncl):
        if attr_thickness_isvect:
            mthickness = mathutils.Vector(attr_thickness[0][1][i]).length*thickness_factor
        if attr_thickness_isscalar:
            mthickness = attr_thickness[0][1][i]*thickness_factor
            
        if attr_eigenvalues_isvect:
            meigs = mathutils.Vector(attr_eigenvalues[0][1][i])
        if attr_eigenvalues_isscalar:
            meigs = mathutils.Vector((attr_eigenvalues[0][1][i],attr_eigenvalues[0][1][i],attr_eigenvalues[0][1][i]))
            
        if attr_basis_isquat:
            mbasis = mathutils.Quaternion(attr_basis[0][1][i])
            
        list_clones_posrotscale[i][0] = list_clones_pos[i]
        list_clones_posrotscale[i][1] = mbasis.to_euler()
        list_clones_posrotscale[i][2] = (abs(meigs[0]*mthickness),abs(meigs[1]*mthickness),abs(meigs[2]*mthickness))
    my_o = make_chrono_glyphs_objects(mname,mpos,mrot,
     [
      [bpy.data.objects[coordsys_geometry], (0,0,0), (1,0,0,0), ""]
     ],
     list_clones_posrotscale,
     list_attributes,
     color, color_min, color_max, colormap
     )
    new_objects.append(my_o)
    return new_objects
   
    
# Same as make_chrono_glyphs_points and make_chrono_glyphs_vectors etc., but uses a 
# glyphsetting object to store input parameters like thickness etc, this because glyphsetting
# can be persistent through frames and hence adjustable through the GUI.
# Before calling this, one must call: setup_glyph_setting(...) 
# and setup_property_scalar(..), setup_setup_property_vector(..) per each property in list_attributes
def update_make_glyphs(glyphsetting, mname, mpos, mrot, list_clones_pos, list_attributes):
 
    mlength = glyphsetting.length_scale
    mlength_factor = glyphsetting.length_scale
    if glyphsetting.length_type == 'PROPERTY':
        mlength = glyphsetting.property[glyphsetting.property_index_length].name
        
    mthickness = glyphsetting.width_scale
    mthickness_factor = glyphsetting.width_scale
    if glyphsetting.width_type == 'PROPERTY':
        mthickness = glyphsetting.property[glyphsetting.property_index_width].name
        
    mdir = glyphsetting.const_dir
    if glyphsetting.dir_type == 'PROPERTY':
        mdir = glyphsetting.property[glyphsetting.property_index_dir].name
        
    meigenvalues = glyphsetting.const_eigenvalues
    if glyphsetting.eigenvalues_type == 'PROPERTY':
        meigenvalues = glyphsetting.property[glyphsetting.property_index_eigenvalues].name
    
    mbasis = mathutils.Quaternion((1,0,0,0))
    if  len(glyphsetting.property): 
        mbasis = glyphsetting.property[glyphsetting.property_index_basis].name
    
    new_objs = []
    
    if glyphsetting.glyph_type == 'VECTOR':
        new_objs = make_chrono_glyphs_vectors(mname,mpos,mrot, 
                          list_clones_pos, 
                          list_attributes,
                          dir = mdir,
                          #basis = mbasis,
                          length = mlength, 
                          length_factor = mlength_factor,
                          color='',            
                          color_min=0, color_max=1, 
                          colormap=colormap_cooltowarm,    
                          thickness = mthickness,
                          thickness_factor = mthickness_factor,
                          do_arrow_tip = glyphsetting.do_tip,      # vector extends with a pointed cone
                          components = (glyphsetting.use_x,glyphsetting.use_y,glyphsetting.use_z))
                          
    if glyphsetting.glyph_type == 'VECTOR LOCAL':
        new_objs = make_chrono_glyphs_vectors(mname,mpos,mrot, 
                          list_clones_pos, 
                          list_attributes,
                          dir = mdir,
                          basis = mbasis,
                          length = mlength, 
                          length_factor = mlength_factor,
                          color='',            
                          color_min=0, color_max=1, 
                          colormap=colormap_cooltowarm,    
                          thickness = mthickness,
                          thickness_factor = mthickness_factor,
                          do_arrow_tip = glyphsetting.do_tip,      # vector extends with a pointed cone
                          components = (glyphsetting.use_x,glyphsetting.use_y,glyphsetting.use_z))

    if glyphsetting.glyph_type == 'POINT':
        new_objs = make_chrono_glyphs_points(mname,mpos,mrot, 
                          list_clones_pos, 
                          list_attributes, 
                          color='',             
                          color_min=0, color_max=1, 
                          colormap=colormap_cooltowarm, 
                          point_geometry = 'chrono_sphere', 
                          thickness = mthickness,
                          thickness_factor = mthickness_factor)
                          
    if glyphsetting.glyph_type == 'COORDSYS':
        new_objs = make_chrono_glyphs_coordsys(mname,mpos,mrot, 
                          list_clones_pos, 
                          list_attributes, 
                          basis = mbasis,
                          color='',             
                          color_min=0, color_max=1, 
                          colormap=colormap_cooltowarm, 
                          coordsys_geometry = 'chrono_csys', 
                          thickness = mthickness,
                          thickness_factor = mthickness_factor)
                          
    if glyphsetting.glyph_type == 'TENSOR':
        new_objs = make_chrono_glyphs_coordsys(mname,mpos,mrot, 
                          list_clones_pos, 
                          list_attributes, 
                          basis = mbasis,
                          eigenvalues = meigenvalues,
                          color='',             
                          color_min=0, color_max=1, 
                          colormap=colormap_cooltowarm, 
                          coordsys_geometry = 'chrono_sphere', 
                          thickness = mthickness,
                          thickness_factor = mthickness_factor)
                          
    for mobj in new_objs:
        for mprop in list_attributes:
            mat = update_glyphsetting_material(mobj[0],glyphsetting, mprop[0])
    
        if glyphsetting.color_type == 'CONST':
            glyphsetting.material.node_tree.nodes['Principled BSDF'].inputs[0].default_value = (glyphsetting.const_color.r,glyphsetting.const_color.g,glyphsetting.const_color.b,1)
            mobj[0].data.materials.clear()
            mobj[0].data.materials.append(glyphsetting.material)
    
    return new_objs
    
    
def update_camera_coordinates(mname,mpos,mrot):
    cameraasset = chrono_cameras.objects.get(mname)
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
    #print("dbg0")
    scene = bpy.context.scene
    cFrame = scene.frame_current
    sFrame = scene.frame_start
    
    global chrono_frame_objects
    global chrono_assets
    global chrono_frame_assets
    global chrono_materials
    global chrono_frame_materials
    global chrono_images
    global chrono_frame_images
    global chrono_cameras
    global chrono_csys
    global empty_mesh
    global chrono_view_asset_csys
    global chrono_view_asset_csys_size
    global chrono_view_item_csys
    global chrono_view_item_csys_size
    global chrono_view_link_csys
    global chrono_view_link_csys_size
    global chrono_view_materials
    global chrono_view_contacts
    global chrono_gui_doupdate
    
    chrono_assets = bpy.data.collections.get('chrono_assets')
    chrono_frame_assets = bpy.data.collections.get('chrono_frame_assets')
    chrono_frame_objects = bpy.data.collections.get('chrono_frame_objects')
    chrono_cameras = bpy.data.collections.get('chrono_cameras')
    
    chrono_gui_doupdate = False

    if (chrono_assets and chrono_frame_objects and chrono_frame_assets):
        
        # delete all things in chrono_frame_objects collection
        for obj in chrono_frame_objects.objects:
            bpy.data.objects.remove(obj, do_unlink=True)
            
        # delete all things in chrono_frame_assets collection
        for obj in chrono_frame_assets.objects:
            bpy.data.objects.remove(obj, do_unlink=True)
          
        # delete unused materials, meshes, etc.
        orphan_mesh = [m for m in bpy.data.meshes if not m.users]
        while orphan_mesh:
            bpy.data.meshes.remove(orphan_mesh.pop())
            
        orphan_particles = [m for m in bpy.data.particles if not m.users]
        while orphan_particles:
            bpy.data.particles.remove(orphan_particles.pop())
        
        # remove just materials added as per-frame materials    
        for mmat in chrono_frame_materials:
            bpy.data.materials.remove(mmat)
        chrono_frame_materials = [] # empty list 
            
        # orphan_materials = [m for m in bpy.data.materials if not m.users]
        #while orphan_materials:
        #    bpy.data.materials.remove(orphan_materials.pop())
            
        orphan_image = [m for m in bpy.data.images if not m.users]
        while orphan_image:
            bpy.data.images.remove(orphan_image.pop())
            
        orphan_grease_pencils = [m for m in bpy.data.grease_pencils if not m.users]
        while orphan_grease_pencils:
            bpy.data.grease_pencils.remove(orphan_grease_pencils.pop())
            
        orphan_node_groups = [m for m in bpy.data.node_groups if not m.users]
        while orphan_node_groups:
            bpy.data.node_groups.remove(orphan_node_groups.pop())
        
        # Load state file, that will fill the chrono_frame_objects and (if necessary) 
        # the chrono_frame_assets collections
        # This for loop may be run more than once if one has imported multiple projects
        # with the "merge" feature, that can be used for cosimulation / parallel stuff
        
        for fileitem in scene.chrono_filenames:
            
            # Retrieve filename
            chrono_filename = fileitem.filename
            
            proj_dir = os.path.dirname(os.path.abspath(chrono_filename))
            filename = os.path.join(proj_dir, 'output', 'state'+'{:05d}'.format(cFrame)+'.py')
            
            if os.path.exists(filename):
                f = open(filename, "rb")
                exec(compile(f.read(), filename, 'exec'))
                f.close()
                
        # in case something was added to chrono_frame_assets, make it invisible 
        for masset in chrono_frame_assets.objects:
            masset.hide_set(True) # not masset.hide_viewport = True otherwise also instances are invisible

    chrono_gui_doupdate = True
    
     
#
# On file selected: 
#
# PREPARE CHRONO CAMERAS COLLECTION
# PREPARE CHRONO ASSETS COLLECTION
# PREPARE CHRONO FRAME ASSETS COLLECTION
# PREPARE CHRONO OBJECTS COLLECTION
# LOAD ASSETS (NON MUTABLE)
#
    
def read_chrono_simulation(context, filepath, setting_materials, setting_merge):
    print("Loading Chrono simulation...")
    
    # PREPARE SCENE
    global chrono_frame_objects
    global chrono_assets
    global chrono_frame_assets
    global chrono_materials
    global chrono_frame_materials
    global chrono_images
    global chrono_frame_images
    global chrono_cameras
    global chrono_csys
    global empty_mesh
    global chrono_view_asset_csys
    global chrono_view_asset_csys_size
    global chrono_view_item_csys
    global chrono_view_item_csys_size
    global chrono_view_link_csys
    global chrono_view_link_csys_size
    global chrono_view_materials
    global chrono_view_contacts
    global chrono_gui_doupdate
    
    # (this is needed to avoid crashes when pressing F12 for rendering)
    bpy.context.scene.render.use_lock_interface = True

    # disable Chrono GUI update, for performance when changing properties calling Update(), will be re-enabled at the end of this 
    chrono_gui_doupdate = False

    bpy.context.scene.ch_meshsetting_index = -1
    bpy.context.scene.ch_meshsetting.clear()
    
    bpy.context.scene.ch_glyph_setting_index = -1
    bpy.context.scene.ch_glyph_setting.clear()
    
    chrono_cameras = bpy.data.collections.get('chrono_cameras')
    if not chrono_cameras:
        chrono_cameras = bpy.data.collections.new('chrono_cameras')
        bpy.context.scene.collection.children.link(chrono_cameras)
            
    chrono_assets = bpy.data.collections.get('chrono_assets')
    if not chrono_assets:
        chrono_assets = bpy.data.collections.new('chrono_assets')
        bpy.context.scene.collection.children.link(chrono_assets)
        
    chrono_frame_assets = bpy.data.collections.get('chrono_frame_assets')
    if not chrono_frame_assets:
        chrono_frame_assets = bpy.data.collections.new('chrono_frame_assets')
        bpy.context.scene.collection.children.link(chrono_frame_assets)
              
    chrono_frame_objects = bpy.data.collections.get('chrono_frame_objects')
    if not chrono_frame_objects:
        chrono_frame_objects = bpy.data.collections.new('chrono_frame_objects')
        bpy.context.scene.collection.children.link(chrono_frame_objects)

            
    chrono_assets.hide_render = True
    chrono_frame_assets.hide_render = True
    
    
    # DELETE OLD OBJECTS (if not merging)
    
    if not setting_merge:
        
        for obj in chrono_cameras.objects:
            bpy.data.objects.remove(obj, do_unlink=True)
            
        for obj in chrono_assets.objects:
            bpy.data.objects.remove(obj, do_unlink=True)
            
        for obj in chrono_frame_assets.objects:
            bpy.data.objects.remove(obj, do_unlink=True)
            
        for obj in chrono_frame_objects.objects:
            bpy.data.objects.remove(obj, do_unlink=True)
            
        # remove materials added as per-frame materials    
        for mmat in chrono_frame_materials:
            bpy.data.materials.remove(mmat)
        chrono_frame_materials = [] # empty list
        
        # remove materials added as immutable materials    
        for mmat in chrono_materials:
            bpy.data.materials.remove(mmat)
        chrono_materials = [] # empty list
    
    
    # CLEANUP orphaned data, if any
    
    orphan_mesh = [m for m in bpy.data.meshes if not m.users]
    while orphan_mesh:
        bpy.data.meshes.remove(orphan_mesh.pop())
        
    orphan_particles = [m for m in bpy.data.particles if not m.users]
    while orphan_particles:
        bpy.data.particles.remove(orphan_particles.pop())
    
    # remove all orphan materials (maybe too aggressive ?)
    #orphan_materials = [m for m in bpy.data.materials if not m.users]
    #while orphan_materials:
    #    bpy.data.materials.remove(orphan_materials.pop())
        
    orphan_image = [m for m in bpy.data.images if not m.users]
    while orphan_image:
        bpy.data.images.remove(orphan_image.pop())
        
    orphan_grease_pencils = [m for m in bpy.data.grease_pencils if not m.users]
    while orphan_grease_pencils:
        bpy.data.grease_pencils.remove(orphan_grease_pencils.pop())
        
    orphan_node_groups = [m for m in bpy.data.node_groups if not m.users]
    while orphan_node_groups:
        bpy.data.node_groups.remove(orphan_node_groups.pop())
           
    # if some sub collection is selected, select the root one, so loading assets.py will work fine  
    bpy.context.view_layer.active_layer_collection = bpy.context.view_layer.layer_collection
       
    # create template for all chrono physics items
    empty_mesh = bpy.data.meshes.get('empty_mesh')
    if not empty_mesh:
        empty_mesh = bpy.data.meshes.new('empty_mesh')
        empty_mesh.from_pydata([], [], []) 
        empty_mesh_object = bpy.data.objects.new('empty_mesh_object', empty_mesh)

    # create template for all chrono coordinate systems
    chrono_csys = bpy.data.objects.get('chrono_csys')
    if not chrono_csys:
        bpy.ops.mesh.primitive_cylinder_add(vertices=10, radius=0.02, depth=1.0, calc_uvs=False, rotation=(0,0,0),location=(0,0,0.5))
        cyZ = bpy.context.object
        matZ = make_bsdf_material('axis_z',(0,0,1,1)) #for render
        matZ.diffuse_color = (0,0,1,1) #for viewport  
        cyZ.data.materials.append(matZ) 
        bpy.ops.mesh.primitive_cylinder_add(vertices=10, radius=0.02, depth=1.0, calc_uvs=False, rotation=(math.pi/2.0,0,0),location=(0,0.5,0))
        cyY = bpy.context.object
        matY = make_bsdf_material('axis_y',(0,1,0,1)) #for render
        matY.diffuse_color = (0,1,0,1) #for viewport
        cyY.data.materials.append(matY)
        bpy.ops.mesh.primitive_cylinder_add(vertices=10, radius=0.02, depth=1.0, calc_uvs=False, rotation=(0,math.pi/2.0,0),location=(0.5,0,0))
        cyX = bpy.context.object
        matX = make_bsdf_material('axis_x',(1,0,0,1)) #for render
        matX.diffuse_color = (1,0,0,1) #for viewport
        cyX.data.materials.append(matX)
        bpy.ops.mesh.primitive_cube_add(size=0.1,calc_uvs=False, location=(0,0,0))
        cyO = bpy.context.object
        matO = make_bsdf_material('axis_origin',(1,1,1,1)) #for render
        matO.diffuse_color = (1,1,1,1) #for viewport
        cyO.data.materials.append(matO)
        with bpy.context.temp_override(selected_editable_objects=[cyZ,cyY,cyX,cyO]):
            bpy.ops.object.join()
        chrono_csys = bpy.context.object
        with bpy.context.temp_override(selected_editable_objects=[chrono_csys]):
            bpy.ops.object.shade_smooth(use_auto_smooth=True,auto_smooth_angle=1.1)
        chrono_csys.visible_shadow = False
        chrono_csys.name ='chrono_csys'
        chrono_csys.select_set(False)
        bpy.context.scene.collection.objects.unlink(chrono_csys)
    
    # create template for primitives (for glyphs etc)
    chrono_cube = bpy.data.objects.get('chrono_cube')
    if not chrono_cube:
        bpy.ops.mesh.primitive_cube_add(size=1,calc_uvs=True)
        chrono_cube = bpy.context.object
        chrono_cube.name = 'chrono_cube'
        bpy.context.scene.collection.objects.unlink(chrono_cube)

    chrono_cylinder = bpy.data.objects.get('chrono_cylinder')
    if not chrono_cylinder:
        bpy.ops.mesh.primitive_cylinder_add(vertices=20, radius=0.5, depth=1.0, calc_uvs=True, rotation=(0,0,0))
        chrono_cylinder = bpy.context.object
        chrono_cylinder.name = 'chrono_cylinder'
        chrono_cylinder.data.transform(mathutils.Matrix.Translation((0,0,0.5)))
        chrono_cylinder.data.use_auto_smooth = 1
        chrono_cylinder.data.auto_smooth_angle = 0.8
        for f in chrono_cylinder.data.polygons:
            f.use_smooth = True
        bpy.context.scene.collection.objects.unlink(chrono_cylinder)

    chrono_cone = bpy.data.objects.get('chrono_cone')
    if not chrono_cone:
        bpy.ops.mesh.primitive_cone_add(vertices=20, radius1=0.5, radius2=0.0, depth=1.0, calc_uvs=True, rotation=(0,0,0))
        chrono_cone = bpy.context.object
        chrono_cone.name = 'chrono_cone'
        chrono_cone.data.transform(mathutils.Matrix.Translation((0,0,0.5)))
        chrono_cone.data.use_auto_smooth = 1
        chrono_cone.data.auto_smooth_angle = 0.8
        for f in chrono_cone.data.polygons:
            f.use_smooth = True
        bpy.context.scene.collection.objects.unlink(chrono_cone)
        
    chrono_sphere = bpy.data.objects.get('chrono_sphere')
    if not chrono_sphere:
        bpy.ops.mesh.primitive_uv_sphere_add(segments=10, ring_count=10, radius=0.5, calc_uvs=True)
        chrono_sphere = bpy.context.object
        chrono_sphere.name = 'chrono_sphere'
        chrono_sphere.data.use_auto_smooth = 1
        chrono_sphere.data.auto_smooth_angle = 0.8
        for f in chrono_sphere.data.polygons:
            f.use_smooth = True
        bpy.context.scene.collection.objects.unlink(chrono_sphere)

    
    chrono_filename = filepath
    
    # Store filename as custom property of scene, so that one can save the Blender project,
    # reopen, and scrub the timeline without the need of doing File/Import/Chrono Import.
    # If in merge mode, the filename is appended to a list, so the scrubbing can reload N outputs.
    
    if not setting_merge:
        bpy.context.scene.chrono_filenames.clear()
        
    fileitem = bpy.context.scene.chrono_filenames.add()
    fileitem.filename = chrono_filename
    
    # Load the xxx.assets.py file to create the non-mutable assets 
    # that will fill the chrono_assets collection.
    # This is executed once per each import. 
    f = open(chrono_filename, "rb")
    exec(compile(f.read(), chrono_filename, 'exec'))
    f.close()
    
    # in case something was added to chrono_assets, make it invisible
    for masset in chrono_assets.objects:
        masset.hide_set(True) # not masset.hide_viewport = True otherwise also instances are invisible

    degp = bpy.context.evaluated_depsgraph_get()

    # restore Chrono GUI update, disabled a beginning for performance
    chrono_gui_doupdate = True
    
    #clear the post frame handler
    bpy.app.handlers.frame_change_post.clear()

    #run the function on each frame
    bpy.app.handlers.frame_change_post.append(callback_post)


    # TEST: Update to a frame where particles are updated
    bpy.context.scene.frame_current = 0


    return {'FINISHED'}



# Sidebar GUI ----------------------------------------------



def UpdatedFunction(self, context):
    global chrono_view_asset_csys
    global chrono_view_asset_csys_size
    global chrono_view_item_csys
    global chrono_view_item_csys_size
    global chrono_view_link_csys
    global chrono_view_link_csys_size
    global chrono_view_materials
    global chrono_view_contacts
    global chrono_gui_doupdate
    #print("In update func...")
    if chrono_gui_doupdate:
        chrono_view_asset_csys = self.chrono_show_assets_coordsys
        chrono_view_asset_csys_size = self.chrono_assets_coordsys_size
        chrono_view_item_csys = self.chrono_show_item_coordsys
        chrono_view_item_csys_size = self.chrono_item_coordsys_size
        chrono_view_links_csys = self.chrono_show_links_coordsys
        chrono_view_links_csys_size = self.chrono_links_coordsys_size
        chrono_view_materials = self.chrono_show_materials
        chrono_view_contacts = self.chrono_show_contacts
        callback_post(self) # force reload 
    
    
def UpdatedMeshsetting(self, context):
    global chrono_view_asset_csys
    global chrono_view_asset_csys_size
    global chrono_view_item_csys
    global chrono_view_item_csys_size
    global chrono_view_link_csys
    global chrono_view_link_csys_size
    global chrono_view_materials
    global chrono_view_contacts
    global chrono_gui_doupdate
    #print("In UpdatedMeshsetting func...")
    if chrono_gui_doupdate:
        callback_post(self) # force reload 
        
        
class Chrono_operator(Operator):
    """ example operator """
    bl_idname = "demo.operator"
    bl_label = "I'm a Skeleton Operator"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return context.mode == "OBJECT"

    def execute(self, context):

        self.report({'INFO'},
            f"execute()")

        return {'FINISHED'}


class GUI_properties(UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        row = layout.row()
        split = row.split(factor=0.5)
        if item.type == 'SCALAR':
            split.label(icon="PARTICLE_DATA", text=item.name)
        if item.type == 'COLOR':
            split.label(icon="COLOR", text=item.name)
        if item.type == 'VECTOR':
            split.label(icon="ORIENTATION_LOCAL", text=item.name)
        if item.type == 'QUATERNION':
            split.label(icon="ORIENTATION_GIMBAL", text=item.name)

    def invoke(self, context, event):
        pass  

class GUI_meshsettins(UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        row = layout.row()
        row.label(icon="OUTLINER_OB_MESH", text=item.name)

    def invoke(self, context, event):
        pass   
    
    
class Chrono_sidebar(Panel):
    """Chrono settings for 3D view"""
    bl_label = "Chrono view"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Chrono"

    def draw(self, context):
        col = self.layout.column(align=True)
        scn = bpy.context.scene
        
        col.prop(context.scene, "chrono_show_item_coordsys")
        col.prop(context.scene, "chrono_item_coordsys_size")
        col.prop(context.scene, "chrono_show_assets_coordsys")
        col.prop(context.scene, "chrono_assets_coordsys_size")
        col.prop(context.scene, "chrono_show_links_coordsys")
        col.prop(context.scene, "chrono_links_coordsys_size")
        col.prop(context.scene, "chrono_show_materials")
        col.prop(context.scene, "chrono_show_contacts")
        
        row0 = self.layout.row()
        row0.label(text="Objects for falsecolor rendering:")
        
        row1 = self.layout.row()
        row1.template_list("GUI_meshsettins", "", scn, "ch_meshsetting", scn, "ch_meshsetting_index", rows=5)

        if scn.ch_meshsetting_index >= 0:
            msetting = scn.ch_meshsetting[scn.ch_meshsetting_index]
            row = self.layout.row()
            split = row.split(factor=0.2)
            col1 = split.column()
            col2 = split.column()
            col1.label(text="color")
            col2.template_list("GUI_properties", "",  msetting, "property",  msetting, "property_index", rows=4)
            if msetting.property_index >=0:
                row = self.layout.row()
                split = row.split(factor=0.2)
                col1 = split.column()
                col2 = split.column()
                col1.label(text="")
                if msetting.property[msetting.property_index].type == 'VECTOR':
                    col2.prop(msetting.property[msetting.property_index], "vect_plot", text="Vect to falsecolor")
                if msetting.property[msetting.property_index].type != 'COLOR':
                    col2.prop(msetting.property[msetting.property_index], "min", emboss=False, text="Min")
                    col2.prop(msetting.property[msetting.property_index], "max", emboss=False, text="Max")
                    col2.prop(msetting.property[msetting.property_index], "colorm", text="Colormap")
        
        row2 = self.layout.row()
        row2.label(text="Glyphs:")
        
        row3 = self.layout.row()
        row3.template_list("GUI_meshsettins", "", scn, "ch_glyph_setting", scn, "ch_glyph_setting_index", rows=3)
        
        if scn.ch_glyph_setting_index >= 0:
            msetting = scn.ch_glyph_setting[scn.ch_glyph_setting_index]
            row = self.layout.row()
            split = row.split(factor=0.2)
            col1 = split.column()
            col2 = split.column()
            col1.label(text="type")
            col2.prop(msetting, "glyph_type", text="")
            
            if msetting.glyph_type == 'VECTOR' or msetting.glyph_type == 'VECTOR LOCAL':
                row = self.layout.row()
                split = row.split(factor=0.2)
                col1 = split.column()
                col2 = split.column()
                rowl = col2.row()
                rowl.prop(msetting, "do_tip", text="pointed tip")
                rowl.prop(msetting, "use_x", text="x")
                rowl.prop(msetting, "use_y", text="y")
                rowl.prop(msetting, "use_z", text="z")
                
                row = self.layout.row()
                split = row.split(factor=0.2)
                col1 = split.column()
                col2 = split.column()
                col1.label(text="dir")
                col2.prop(msetting, "dir_type", text="")
                if msetting.dir_type == 'CONST':
                    col2.prop(msetting, "const_dir", text="direction")
                else:
                    col2.template_list("GUI_properties", "",  msetting, "property",  msetting, "property_index_dir", rows=3)
                
                if msetting.glyph_type == 'VECTOR LOCAL':
                    row = self.layout.row()
                    split = row.split(factor=0.2)
                    col1 = split.column()
                    col2 = split.column()
                    col1.label(text="basis")
                    col2.template_list("GUI_properties", "",  msetting, "property",  msetting, "property_index_basis", rows=3)
                    
                row = self.layout.row()
                split = row.split(factor=0.2)
                col1 = split.column()
                col2 = split.column()
                col1.label(text="length")
                col2.prop(msetting, "length_type", text="")
                if msetting.length_type == 'CONST':
                    col2.prop(msetting, "length_scale", text="length")
                else:
                    col2.template_list("GUI_properties", "",  msetting, "property",  msetting, "property_index_length", rows=3)
                    col2.prop(msetting, "length_scale", text="scaling factor")
                    
            if msetting.glyph_type == 'COORDSYS':
                row = self.layout.row()
                split = row.split(factor=0.2)
                col1 = split.column()
                col2 = split.column()
                col1.label(text="basis")
                col2.template_list("GUI_properties", "",  msetting, "property",  msetting, "property_index_basis", rows=3)
                
            if msetting.glyph_type == 'TENSOR':
                
                row = self.layout.row()
                split = row.split(factor=0.2)
                col1 = split.column()
                col2 = split.column()
                col1.label(text="basis")
                col2.template_list("GUI_properties", "",  msetting, "property",  msetting, "property_index_basis", rows=3)
                
                row = self.layout.row()
                split = row.split(factor=0.2)
                col1 = split.column()
                col2 = split.column()
                col1.label(text="eigenvals")
                col2.prop(msetting, "eigenvalues_type", text="")
                if msetting.eigenvalues_type == 'CONST':
                    col2.prop(msetting, "const_eigenvalues", text="eigs")
                else:
                    col2.template_list("GUI_properties", "",  msetting, "property",  msetting, "property_index_eigenvalues", rows=3)
                    
            row = self.layout.row()
            split = row.split(factor=0.2)
            col1 = split.column()
            col2 = split.column()
            col1.label(text="width")
            col2.prop(msetting, "width_type", text="")
            if msetting.width_type == 'CONST':
                col2.prop(msetting, "width_scale", text="width")
            else:
                col2.template_list("GUI_properties", "",  msetting, "property",  msetting, "property_index_width", rows=3)
                col2.prop(msetting, "width_scale", text="scaling factor")
            row = self.layout.row()
            split = row.split(factor=0.2)
            col1 = split.column()
            col2 = split.column()
            col1.label(text="color")
            col2.prop(msetting, "color_type", text="")
            if msetting.color_type == 'CONST':
                col2.prop(msetting, "const_color", text="color")
            else:
                col2.template_list("GUI_properties", "",  msetting, "property",  msetting, "property_index_color", rows=3)
                if msetting.property_index_width >=0:
                    if msetting.property[msetting.property_index_width].type != 'COLOR':
                        col2.prop(msetting.property[msetting.property_index_width], "min", emboss=False, text="Min")
                        col2.prop(msetting.property[msetting.property_index_width], "max", emboss=False, text="Max")
                        col2.prop(msetting.property[msetting.property_index_width], "colorm", text="Colormap")


class CUSTOM_filenamesCollection(PropertyGroup):
    filename: StringProperty(
        default="",
        maxlen=1024,  
    )                                  
  
class CUSTOM_propertyCollection(PropertyGroup):
    #name: StringProperty() -> Instantiated by default
    min: FloatProperty(update=UpdatedMeshsetting)
    max: FloatProperty(update=UpdatedMeshsetting)
    colorm: EnumProperty(name = "Colormaps",
        description = "The colormap for rendering the selected data",
        items = [
        ('colormap_jet', 'Jet', ""),
        ('colormap_cooltowarm', 'Cool to warm', ""),
        ('colormap_viridis', 'Viridis', ""),
        ('colormap_hot', 'Hot', ""),
        ('colormap_cool', 'Cool', ""),
        ('colormap_winter', 'Winter', "")],
        update=UpdatedMeshsetting)
    mat: PointerProperty(name="material", type=bpy.types.Material)
    type: EnumProperty(name = "type", items= [
        ('COLOR','',''),
        ('SCALAR','',''),
        ('VECTOR','',''),
        ('QUATERNION','','')])
    vect_plot: EnumProperty(name = "vect_plot", items= [
        ('NORM','Norm',''),
        ('X','x',''),
        ('Y','y',''),
        ('Z','z','')],
        update=UpdatedMeshsetting)
    
class CUSTOM_meshsettingCollection(PropertyGroup):
    #name: StringProperty() -> Instantiated by default
    property_index: IntProperty(update=UpdatedMeshsetting)
    property: CollectionProperty(type=CUSTOM_propertyCollection)
    
class CUSTOM_glyphsettingCollection(PropertyGroup):
    #name: StringProperty() -> Instantiated by default
    glyph_type: EnumProperty(name = "glyph_type", items= [
        ('POINT','Point',''),
        ('VECTOR','Vector',''),
        ('VECTOR LOCAL','Vector local',''),
        ('COORDSYS','Coordsys',''),
        ('TENSOR','Tensor',''),],
        update=UpdatedMeshsetting)
    property: CollectionProperty(type=CUSTOM_propertyCollection)
    property_index_dir: IntProperty(update=UpdatedMeshsetting)
    property_index_length: IntProperty(update=UpdatedMeshsetting)
    property_index_width: IntProperty(update=UpdatedMeshsetting)
    property_index_color: IntProperty(update=UpdatedMeshsetting)
    property_index_basis: IntProperty(update=UpdatedMeshsetting)
    property_index_eigenvalues: IntProperty(update=UpdatedMeshsetting)
    dir_type: EnumProperty(name = "dir_type", items= [
        ('CONST','Constant',''),
        ('PROPERTY','Property','')],
        update=UpdatedMeshsetting)
    length_type: EnumProperty(name = "lenght_type", items= [
        ('CONST','Constant',''),
        ('PROPERTY','Property','')],
        update=UpdatedMeshsetting)
    width_type: EnumProperty(name = "width_type", items= [
        ('CONST','Constant',''),
        ('PROPERTY','Property','')],
        update=UpdatedMeshsetting)
    color_type: EnumProperty(name = "color_type", items= [
        ('CONST','Constant',''),
        ('PROPERTY','Property','')],
        update=UpdatedMeshsetting)
    basis_type: EnumProperty(name = "color_type", items= [
        ('CONST','Constant',''),
        ('PROPERTY','Property','')],
        update=UpdatedMeshsetting)
    eigenvalues_type: EnumProperty(name = "eigenvalues_type", items= [
        ('CONST','Constant',''),
        ('PROPERTY','Property','')],
        update=UpdatedMeshsetting)
    const_dir:  FloatVectorProperty(subtype='XYZ',update=UpdatedMeshsetting)
    length_scale: FloatProperty(min=0, precision=5, update=UpdatedMeshsetting)
    width_scale:  FloatProperty(min=0, soft_max=0.5, step=1, precision=5, update=UpdatedMeshsetting)
    const_color:  FloatVectorProperty(subtype='COLOR',update=UpdatedMeshsetting)
    const_basis:  FloatVectorProperty(subtype='QUATERNION', size=4, update=UpdatedMeshsetting)
    const_eigenvalues:  FloatVectorProperty(size=3, update=UpdatedMeshsetting)
    use_x:  BoolProperty(update=UpdatedMeshsetting)
    use_y:  BoolProperty(update=UpdatedMeshsetting)
    use_z:  BoolProperty(update=UpdatedMeshsetting)
    do_tip:  BoolProperty(update=UpdatedMeshsetting)
    material: PointerProperty(name="material", type=bpy.types.Material)
 
sidebar_classes = [
    Chrono_operator,
    Chrono_sidebar,
    GUI_meshsettins,
    GUI_properties,
    CUSTOM_propertyCollection,
    CUSTOM_meshsettingCollection,
    CUSTOM_glyphsettingCollection,
    CUSTOM_filenamesCollection,
]






# Menu in File/Import/.. GUI ----------------------------------------------


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
    setting_merge: BoolProperty(
        name="Merge",
        description="If true, does not delete last imported Chrono simulation. Useful for merging results from parallel simulations, or cosimulations.",
        default=False,
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
        return read_chrono_simulation(context, self.filepath, self.setting_materials, self.setting_merge)


# Only needed if you want to add into a dynamic menu.
def menu_func_import(self, context):
    self.layout.operator(ImportChrono.bl_idname, text="Chrono import")



# REGISTER THE ADD-ON ----------------------------------------------


def register():
    
    
    bpy.utils.register_class(ImportChrono)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)
    
        #clear the post frame handler
    bpy.app.handlers.frame_change_post.clear()

    #run the function on each frame
    bpy.app.handlers.frame_change_post.append(callback_post)

    # sidebar UI:
    for c in sidebar_classes:
        bpy.utils.register_class(c)
        
    bpy.types.Scene.chrono_show_item_coordsys = bpy.props.BoolProperty(
        name='Show items coordsys',
        default=False,
        update=UpdatedFunction
    )
    bpy.types.Scene.chrono_item_coordsys_size = bpy.props.FloatProperty(
        name='Item coordsys size',
        default=0.15,
        update=UpdatedFunction
    )
    bpy.types.Scene.chrono_show_assets_coordsys = bpy.props.BoolProperty(
        name='Show assets coordsys',
        default=False,
        update=UpdatedFunction
    )
    bpy.types.Scene.chrono_assets_coordsys_size = bpy.props.FloatProperty(
        name='Assets coordsys size',
        default=0.10,
        update=UpdatedFunction
    )
    bpy.types.Scene.chrono_show_links_coordsys = bpy.props.BoolProperty(
        name='Show links coordsys',
        default=True,
        update=UpdatedFunction
    )
    bpy.types.Scene.chrono_links_coordsys_size = bpy.props.FloatProperty(
        name='Links coordsys size',
        default=0.20,
        update=UpdatedFunction
    )
    bpy.types.Scene.chrono_show_materials = bpy.props.BoolProperty(
        name='Use Chrono materials',
        default=True,
        update=UpdatedFunction
    )
    bpy.types.Scene.chrono_show_contacts = bpy.props.BoolProperty(
        name='Show contacts',
        default=True,
        update=UpdatedFunction
    )
    
    bpy.types.Scene.chrono_filenames = bpy.props.CollectionProperty(
        type=CUSTOM_filenamesCollection, 
        description = "Projects exported from Chrono postprocesor",
    )
    
    # Custom mesh properties
    bpy.types.Scene.ch_meshsetting = bpy.props.CollectionProperty(
        type=CUSTOM_meshsettingCollection, 
        description = "Assets with data attached from Chrono, that can be rendered in falsecolor",
    )
    bpy.types.Scene.ch_meshsetting_index = bpy.props.IntProperty(
        default = -1
    )

    # custom glyph properties
    bpy.types.Scene.ch_glyph_setting = bpy.props.CollectionProperty(
        type=CUSTOM_glyphsettingCollection, 
        description = "Assets with data attached from Chrono, that can be rendered in falsecolor",
    )
    bpy.types.Scene.ch_glyph_setting_index = bpy.props.IntProperty(
        default = -1
    )

    
    
    

def unregister():
    
    del bpy.types.Scene.chrono_show_item_coordsys
    del bpy.types.Scene.chrono_item_coordsys_size
    del bpy.types.Scene.chrono_show_assets_coordsys
    del bpy.types.Scene.chrono_assets_coordsys_size
    del bpy.types.Scene.chrono_show_links_coordsys
    del bpy.types.Scene.chrono_links_coordsys_size
    del bpy.types.Scene.chrono_show_materials
    del bpy.types.Scene.chrono_show_contacts
    del bpy.types.Scene.ch_meshsetting
    del bpy.types.Scene.ch_meshsetting_index 
    del bpy.types.Scene.ch_glyph_setting
    del bpy.types.Scene.ch_glyph_setting_index 
    del bpy.types.Scene.chrono_filenames
    for c in sidebar_classes:
        bpy.utils.unregister_class(c)
        

        
    bpy.utils.unregister_class(ImportChrono)
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)
    bpy.app.handlers.frame_change_post.remove(callback_post)

    # sidebar UI:
    


# The following is executed all times one runs this chrono_import.py script in Blender
# scripting editor: it effectively register the add-on "by hand".

if __name__ == "__main__":
    
    # register the add-on just like it was one of the registered add-ons in Edit/preferences...
    register()

    # The following bpy.context.scene.... stuff canNOT be put in register() 
    # because they give a "Restricted content error", but still can be called
    # here, i.e. when running this script in the Scripting editor:
    
    bpy.context.scene.ch_meshsetting.clear()
    bpy.context.scene.ch_glyph_setting.clear()
    
    # (this is needed to avoid crashes when pressing F12 for rendering)
    bpy.context.scene.render.use_lock_interface = True
    
    # TEST: test call
    #bpy.ops.import_chrono.data('INVOKE_DEFAULT')
