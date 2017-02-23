#-------------------------------------------------------------------------------
# Name:        demo_masonry
#
# This file shows how to
#   - create a small stack of bricks,
#   - create a support that shakes like an earthquake, with imposed motion law
#   - simulate the bricks that fall
#   - output the postprocessing data for rendering the animation with POVray
#-------------------------------------------------------------------------------
#!/usr/bin/env python

def main():
    pass

if __name__ == '__main__':
    main()


# Load the Chrono::Engine unit and the postprocessing unit!!!
import ChronoEngine_python_core as chrono
import ChronoEngine_python_postprocess as postprocess
import ChronoEngine_python_irrlicht as chronoirr


# We will create two directories for saving some files, we need this:
import os
import math



# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

my_system = chrono.ChSystem()


# Set the default outward/inward shape margins for collision detection,
# this is epecially important for very large or very small objects.
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)

# Maybe you want to change some settings for the solver. For example you
# might want to use SetMaxItersSolverSpeed to set the number of iterations
# per timestep, etc.

#my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) # precise, more slow
my_system.SetMaxItersSolverSpeed(70)



# Create a contact material (surface property)to share between all objects.
# The rolling and spinning parameters are optional - if enabled they double
# the computational time.
brick_material = chrono.ChMaterialSurface()
brick_material.SetFriction(0.5)
brick_material.SetDampingF(0.2)
brick_material.SetCompliance (0.0000001)
brick_material.SetComplianceT(0.0000001)
# brick_material.SetRollingFriction(rollfrict_param)
# brick_material.SetSpinningFriction(0)
# brick_material.SetComplianceRolling(0.0000001)
# brick_material.SetComplianceSpinning(0.0000001)



# Create the set of bricks in a vertical stack, along Y axis

nbricks_on_x = 1
nbricks_on_y = 6

size_brick_x = 0.25
size_brick_y = 0.12
size_brick_z = 0.12
density_brick = 1000;    # kg/m^3
mass_brick = density_brick * size_brick_x * size_brick_y * size_brick_z;
inertia_brick = 2/5*(pow(size_brick_x,2))*mass_brick; # to do: compute separate xx,yy,zz inertias

for ix in range(0,nbricks_on_x):
    for iy in range(0,nbricks_on_y):
        # create it
        body_brick = chrono.ChBody()
        # set initial position
        body_brick.SetPos(chrono.ChVectorD(ix*size_brick_x, (iy+0.5)*size_brick_y, 0 ))
        # set mass properties
        body_brick.SetMass(mass_brick)
        body_brick.SetInertiaXX(chrono.ChVectorD(inertia_brick,inertia_brick,inertia_brick))
        # set collision surface properties
        body_brick.SetMaterialSurface(brick_material)

        # Collision shape
        body_brick.GetCollisionModel().ClearModel()
        body_brick.GetCollisionModel().AddBox(size_brick_x/2, size_brick_y/2, size_brick_z/2) # must set half sizes
        body_brick.GetCollisionModel().BuildModel()
        body_brick.SetCollide(True)

        # Visualization shape, for rendering animation
        body_brick_shape = chrono.ChBoxShape()
        body_brick_shape.GetBoxGeometry().Size = chrono.ChVectorD(size_brick_x/2, size_brick_y/2, size_brick_z/2)
        if iy%2==0 :
            body_brick_shape.SetColor(chrono.ChColor(0.65, 0.65, 0.6)) # set gray color only for odd bricks
        body_brick.GetAssets().push_back(body_brick_shape)

        my_system.Add(body_brick)


# Create the room floor: a simple fixed rigid body with a collision shape
# and a visualization shape

body_floor = chrono.ChBody()
body_floor.SetBodyFixed(True)
body_floor.SetPos(chrono.ChVectorD(0, -2, 0 ))
body_floor.SetMaterialSurface(brick_material)

# Collision shape
body_floor.GetCollisionModel().ClearModel()
body_floor.GetCollisionModel().AddBox(3, 1, 3) # hemi sizes
body_floor.GetCollisionModel().BuildModel()
body_floor.SetCollide(True)

# Visualization shape
body_floor_shape = chrono.ChBoxShape()
body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(3, 1, 3)
body_floor.GetAssets().push_back(body_floor_shape)

body_floor_texture = chrono.ChTexture()
body_floor_texture.SetTextureFilename('../../../data/concrete.jpg')
body_floor.GetAssets().push_back(body_floor_texture)

my_system.Add(body_floor)



# Create the shaking table, as a box

size_table_x = 1;
size_table_y = 0.2;
size_table_z = 1;

body_table = chrono.ChBody()
body_table.SetPos(chrono.ChVectorD(0, -size_table_y/2, 0 ))
body_table.SetMaterialSurface(brick_material)

# Collision shape
body_table.GetCollisionModel().ClearModel()
body_table.GetCollisionModel().AddBox(size_table_x/2, size_table_y/2, size_table_z/2) # hemi sizes
body_table.GetCollisionModel().BuildModel()
body_table.SetCollide(True)

# Visualization shape
body_table_shape = chrono.ChBoxShape()
body_table_shape.GetBoxGeometry().Size = chrono.ChVectorD(size_table_x/2, size_table_y/2, size_table_z/2)
body_table_shape.SetColor(chrono.ChColor(0.4,0.4,0.5))
body_table.GetAssets().push_back(body_table_shape)

body_table_texture = chrono.ChTexture()
body_table_texture.SetTextureFilename('../../../data/concrete.jpg')
body_table.GetAssets().push_back(body_table_texture)

my_system.Add(body_table)


# Create a constraint that blocks free 3 x y z translations and 3 rx ry rz rotations
# of the table respect to the floor, and impose that the relative imposed position
# depends on a specified motion law.

link_shaker = chrono.ChLinkLockLock()
link_shaker.Initialize(body_table, body_floor, chrono.CSYSNORM)
my_system.Add(link_shaker)

# ..create the function for imposed x horizontal motion, etc.
mfunY = chrono.ChFunction_Sine(0,1.5,0.001)  # phase, frequency, amplitude
link_shaker.SetMotion_Y(mfunY)

# ..create the function for imposed y vertical motion, etc.
mfunZ = chrono.ChFunction_Sine(0,1.5,0.12)  # phase, frequency, amplitude
link_shaker.SetMotion_Z(mfunZ)

# Note that you could use other types of ChFunction_ objects, or create
# your custom function by class inheritance (see demo_python.py), or also
# set a function for table rotation , etc.




# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(my_system)

myapplication.AddTypicalSky('../../../data/skybox/')
myapplication.AddTypicalCamera(chronoirr.vector3df(0.5,0.5,1.0))
myapplication.AddLightWithShadow(chronoirr.vector3df(2,4,2),    # point
                                 chronoirr.vector3df(0,0,0),    # aimpoint
                                 9,                 # radius (power)
                                 1,9,               # near, far
                                 30)                # angle of FOV

            # ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
			# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
			# If you need a finer control on which item really needs a visualization proxy in
			# Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

myapplication.AssetBindAll();

			# ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
			# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

myapplication.AssetUpdateAll();

            # If you want to show shadows because you used "AddLightWithShadow()'
            # you must remember this:
myapplication.AddShadowAll();

# ---------------------------------------------------------------------
#
#  Run the simulation
#

myapplication.SetStepManage(True)
myapplication.SetTimestep(0.001)
myapplication.SetTryRealtime(True)

while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    for substep in range(0,5):
        myapplication.DoStep()
    myapplication.EndScene()


