#-------------------------------------------------------------------------------
# Name:        demo_python_3
#
# This file shows how to create and populate the ChParticleClones object.
# Also, shows how to use POV ray for postprocessing, thanks to the
# utility functions in the unit_POSTPROCESS of Chrono::Engine.
#
#-------------------------------------------------------------------------------
#!/usr/bin/env python

def main():
    pass

if __name__ == '__main__':
    main()


# Load the Chrono::Engine unit and the postprocessing unit!!!
import ChronoEngine_python_core as chrono
import ChronoEngine_python_postprocess as postprocess

# We will create two directories for saving some files, we need this:
import os


# Create a physical system,
my_system = chrono.ChSystem()


# Set the default margins for collision detection, this is epecially
# important for very large or very small objects.
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)



# Create the set of the particle clones (many rigid bodies that
# share the same mass and collision shape, so they are memory efficient
# in case you want to simulate granular material)

body_particles = chrono.ChParticlesClones()
body_particles.SetMass(0.01);
inertia = 2/5*(pow(0.005,2))*0.01;
body_particles.SetInertiaXX(chrono.ChVectorD(inertia,inertia,inertia));

# Collision shape (shared by all particle clones) Must be defined BEFORE adding particles
body_particles.GetCollisionModel().ClearModel()
body_particles.GetCollisionModel().AddSphere(0.005)
body_particles.GetCollisionModel().BuildModel()
body_particles.SetCollide(True)

# add particles
for ix in range(0,5):
    for iy in range(0,5):
        for iz in range(0,3):
            body_particles.AddParticle(chrono.ChCoordsysD(chrono.ChVectorD(ix/100,0.1+iy/100, iz/100)))

# Visualization shape (shared by all particle clones)
body_particles_shape = chrono.ChSphereShape()
body_particles_shape.GetSphereGeometry().rad = 0.005
body_particles.GetAssets().push_back(body_particles_shape)

my_system.Add(body_particles)




# Create the floor: a simple fixed rigid body with a collision shape
# and a visualization shape

body_floor = chrono.ChBody()
body_floor.SetBodyFixed(True)

# Collision shape
body_floor.GetCollisionModel().ClearModel()
body_floor.GetCollisionModel().AddBox(0.1, 0.02, 0.1) # hemi sizes
body_floor.GetCollisionModel().BuildModel()
body_floor.SetCollide(True)

# Visualization shape
body_floor_shape = chrono.ChBoxShape()
body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.1, 0.02, 0.1)
body_floor_shape.SetColor(chrono.ChColor(0.5,0.5,0.5))
body_floor.GetAssets().push_back(body_floor_shape)

my_system.Add(body_floor)



# Create boxes that fall
# This is just for fun.

for ix in range(0,2):
    for iz in range(0,4):
        body_brick = chrono.ChBody()
        body_brick.SetPos(chrono.ChVectorD(0.05+ix*0.021,0.04,0+iz*0.021))
        body_brick.SetMass(0.02);
        inertia = 2/5*(pow(0.01,2))*0.02;
        body_brick.SetInertiaXX(chrono.ChVectorD(inertia,inertia,inertia));

        # Collision shape
        body_brick.GetCollisionModel().ClearModel()
        body_brick.GetCollisionModel().AddBox(0.01, 0.01, 0.01) # hemi sizes
        body_brick.GetCollisionModel().BuildModel()
        body_brick.SetCollide(True)

        # Visualization shape
        body_brick_shape = chrono.ChBoxShape()
        body_brick_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.01, 0.01, 0.01)
        body_brick.GetAssets().push_back(body_brick_shape)

        my_system.Add(body_brick)



# ---------------------------------------------------------------------
#
#  Render a short animation by generating scripts
#  to be used with POV-Ray
#

pov_exporter = postprocess.ChPovRay(my_system)

 # Sets some file names for in-out processes.
pov_exporter.SetTemplateFile        ("../../../data/_template_POV.pov")
pov_exporter.SetOutputScriptFile    ("rendering_frames.pov")
if not os.path.exists("output"):
    os.mkdir("output")
if not os.path.exists("anim"):
    os.mkdir("anim")
pov_exporter.SetOutputDataFilebase("output/my_state")
pov_exporter.SetPictureFilebase("anim/picture")

pov_exporter.SetCamera(chrono.ChVectorD(0.2,0.3,0.5), chrono.ChVectorD(0,0,0), 35)
pov_exporter.SetLight(chrono.ChVectorD(-2,2,-1), chrono.ChColor(1.1,1.2,1.2), True)
pov_exporter.SetPictureSize(640,480)
pov_exporter.SetAmbientLight(chrono.ChColor(2,2,2))

 # Add additional POV objects/lights/materials in the following way
pov_exporter.SetCustomPOVcommandsScript(
'''
light_source{ <1,3,1.5> color rgb<1.1,1.1,1.1> }
Grid(0.05,0.04, rgb<0.7,0.7,0.7>, rgbt<1,1,1,1>)
''')

 # Tell which physical items you want to render
pov_exporter.AddAll()

 # Tell that you want to render the contacts
pov_exporter.SetShowContacts(True,
                            postprocess.ChPovRay.SYMBOL_VECTOR_SCALELENGTH,
                            0.2,    # scale
                            0.0007, # width
                            0.1,    # max size
                            True,0,0.5 ) # colormap on, blue at 0, red at 0.5

 # 1) Create the two .pov and .ini files for POV-Ray (this must be done
 #    only once at the beginning of the simulation).
pov_exporter.ExportScript()

#my_system.SetSolverType(chrono.ChSolver.Type_PMINRES)
my_system.SetMaxItersSolverSpeed(50)


 # Perform a short simulation
while (my_system.GetChTime() < 0.7) :

    my_system.DoStepDynamics(0.005)

    print ('time=', my_system.GetChTime() )

    # 2) Create the incremental nnnn.dat and nnnn.pov files that will be load
    #    by the pov .ini script in POV-Ray (do this at each simulation timestep)
    pov_exporter.ExportData()



