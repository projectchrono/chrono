import os, sys
import bpy
import mathutils
modpath = os.path.join('..', '..', '..', 'chrono_thirdparty', 'chpf')
sys.path.append(modpath)
import chpf



#Creates a Blender particle system. We need the camera pose to place the
def renderPsys(particle_path, cam):

    # read the particle data from the .chpf file
    particle_data = chpf.read(particle_path)
    context = bpy.context
    # instance object: this is the "mother of all spheres", all ps spheres are instances of this one
    # we set radius one to get the proper particle radius by scaling the instances by a factor r (r = particle radius)
    # TODO: place the instance object BEHIND the camera
    bpy.ops.mesh.primitive_ico_sphere_add(radius=1, location=(50, 50, 50))
    ico = context.object
    # ico.hide_set(True)


    # cube with ps: this is the emitter of the particles. We do not really use it
    # since the particle position is determined by the simulation data but it is needed anyway
    bpy.ops.mesh.primitive_cube_add(size=0.0001)
    cube = context.object

    # ps creation, that requires an emitter (the cube)
    ps = cube.modifiers.new("SomeName", 'PARTICLE_SYSTEM').particle_system
    psname = ps.name
    ps.settings.count = len(particle_data)
    #number of frames the ps will last. We actually need 2 but why being cheap?
    ps.settings.lifetime = 1000
    # The particle system starts at frame 1
    ps.settings.frame_start = ps.settings.frame_end = 1
    # we want to replicate the object itself (not Wireframe)
    ps.settings.render_type = "OBJECT"
    # The object that we replicate is the sphere we created
    ps.settings.instance_object = ico

    def particle_handler(scene, depsgraph):
        # we access the ps from the emitter
        ob = depsgraph.objects.get(cube.name)
        if ob:
            ps = ob.particle_systems[psname]
            f = scene.frame_current
            # Iterating through all particles we set radius and position
            for m, particle in enumerate(ps.particles):
                setattr(particle, "location", particle_data[m][0:3])
                setattr(particle, "size", particle_data[m][3])

    # Clear the post frame handler
    bpy.app.handlers.frame_change_post.clear()
    # Register our particleSetter with the post frame handler
    bpy.app.handlers.frame_change_post.append(particle_handler)
    # Trigger frame update
    bpy.context.scene.frame_current = 2

    # Move base object and the emitter BEHIND the camera. The camera looks towards the -Z axis
    dist = mathutils.Vector((0,0,10))
    dist.rotate(cam.rotation_euler)
    dump_pos = dist + cam.location
    ico.location = dump_pos
    cube.location = dump_pos

