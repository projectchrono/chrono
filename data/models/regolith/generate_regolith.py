import bpy
import random
import os

# Directory where the OBJ files will be saved
export_directory = "/tmp/regolith/"  # Change this to your desired path

def create_granular_particle():
    # Create a base shape (e.g., icosphere for a round particle)
    bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=2, radius=0.5, location=(0, 0, 0))  # Generate at origin
    particle = bpy.context.object

    # First Displace Modifier for Coarse Roughness
    bpy.ops.object.modifier_add(type='DISPLACE')
    displace_mod = particle.modifiers[-1]

    # Create a texture for coarse displacement
    tex_name = "CoarseRoughTexture_" + str(random.randint(0, 10000))
    tex = bpy.data.textures.new(name=tex_name, type='VORONOI')
    tex.noise_intensity = random.uniform(0.8, 1.2)
    tex.distance_metric = 'MINKOVSKY'
    tex.minkovsky_exponent = random.uniform(0.6, 0.9)

    displace_mod.texture = tex
    displace_mod.strength = random.uniform(0.1, 0.3)

    bpy.ops.object.modifier_apply(modifier=displace_mod.name)

    # Second Displace Modifier for Fine Surface Detail
    bpy.ops.object.modifier_add(type='DISPLACE')
    displace_mod2 = particle.modifiers[-1]
    tex_name2 = "FineRoughTexture_" + str(random.randint(0, 10000))
    tex2 = bpy.data.textures.new(name=tex_name2, type='CLOUDS')
    tex2.noise_scale = random.uniform(0.1, 0.15)
    displace_mod2.texture = tex2
    displace_mod2.strength = random.uniform(0.02, 0.08)

    bpy.ops.object.modifier_apply(modifier=displace_mod2.name)

    # Subdivision Surface Modifier for smoothing
    bpy.ops.object.modifier_add(type='SUBSURF')
    subsurf_mod = particle.modifiers[-1]
    subsurf_mod.levels = 1
    bpy.ops.object.modifier_apply(modifier=subsurf_mod.name)

    # Random rotation for variety
    particle.rotation_euler = (random.uniform(0, 360), random.uniform(0, 360), random.uniform(0, 360))

    # Randomize the shape: flat, elongated, or round
    shape_variation = random.choice(['round', 'flat', 'elongated'])

    if shape_variation == 'flat':
        particle.scale = (random.uniform(0.008, 0.012),
                          random.uniform(0.008, 0.012),
                          random.uniform(0.004, 0.006))
    elif shape_variation == 'elongated':
        elongation_axis = random.choice(['x', 'y', 'z'])
        if elongation_axis == 'x':
            particle.scale = (random.uniform(0.015, 0.02),
                              random.uniform(0.008, 0.012),
                              random.uniform(0.008, 0.012))
        elif elongation_axis == 'y':
            particle.scale = (random.uniform(0.008, 0.012),
                              random.uniform(0.015, 0.02),
                              random.uniform(0.008, 0.012))
        elif elongation_axis == 'z':
            particle.scale = (random.uniform(0.008, 0.012),
                              random.uniform(0.008, 0.012),
                              random.uniform(0.015, 0.02))
    else:
        particle.scale = (random.uniform(0.008, 0.012),
                          random.uniform(0.008, 0.012),
                          random.uniform(0.008, 0.012))

    # Create a new material for lunar regolith appearance
    mat = bpy.data.materials.new(name="LunarRegolithMaterial")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links

    # Clear default nodes
    for node in nodes:
        nodes.remove(node)

    # Add necessary nodes
    output_node = nodes.new(type='ShaderNodeOutputMaterial')
    principled_bsdf = nodes.new(type='ShaderNodeBsdfPrincipled')

    # Set base color to dark gray (lunar regolith color)
    principled_bsdf.inputs['Base Color'].default_value = (0.3, 0.3, 0.3, 1.0)

    # Increase roughness for a dusty look
    principled_bsdf.inputs['Roughness'].default_value = 0.9

    # Add noise texture for bump mapping (surface roughness)
    noise_texture = nodes.new(type='ShaderNodeTexNoise')
    noise_texture.inputs['Scale'].default_value = 30.0
    noise_texture.inputs['Detail'].default_value = 2.0

    # Add bump node to connect noise texture for surface detail
    bump_node = nodes.new(type='ShaderNodeBump')
    bump_node.inputs['Strength'].default_value = 0.3  # Moderate bump effect

    # Connect noise texture to bump node
    links.new(noise_texture.outputs['Fac'], bump_node.inputs['Height'])

    # Connect bump node to Principled BSDF normal input
    links.new(bump_node.outputs['Normal'], principled_bsdf.inputs['Normal'])

    # Connect BSDF to Material Output
    links.new(principled_bsdf.outputs['BSDF'], output_node.inputs['Surface'])

    # Assign the material to the particle
    particle.data.materials.append(mat)

    return particle, mat

def export_particle_as_obj(particle, file_name):
    # Select the particle before exporting
    bpy.ops.object.select_all(action='DESELECT')  # Deselect all
    particle.select_set(True)  # Select the particle to export

    # Export the particle as an OBJ file with the material
    bpy.ops.wm.obj_export(filepath=file_name, export_selected_objects=True, export_materials=True)

def delete_particle_and_material(particle, material):
    # Delete the particle and its material
    bpy.data.objects.remove(particle, do_unlink=True)
    bpy.data.materials.remove(material, do_unlink=True)

# Number of granular particles to generate and export
num_particles = 10

for i in range(num_particles):
    # Step 1: Generate a particle
    particle, material = create_granular_particle()

    # Step 2: Export the particle as OBJ with the material
    obj_file_name = os.path.join(export_directory, f"particle_{i+1}.obj")
    export_particle_as_obj(particle, obj_file_name)

    # Step 3: Delete the particle and material
    delete_particle_and_material(particle, material)

    print(f"Particle {i+1} exported and deleted.")

