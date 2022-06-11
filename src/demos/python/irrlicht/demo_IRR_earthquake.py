# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2019 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# ---------------------------------------------------------------------
#
#  Create the simulation sys and add items
#

sys = chrono.ChSystemNSC()


# Set the default outward/inward shape margins for collision detection,
# this is epecially important for very large or very small objects.
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)

# Maybe you want to change some settings for the solver. For example you
# might want to use SetSolverMaxIterations to set the number of iterations
# per timestep, etc.

#sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) # precise, more slow
sys.SetSolverMaxIterations(70)



# Create a contact material (surface property)to share between all objects.
# The rolling and spinning parameters are optional - if enabled they double
# the computational time.
brick_material = chrono.ChMaterialSurfaceNSC()
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

        # Collision shape
        body_brick.GetCollisionModel().ClearModel()
        body_brick.GetCollisionModel().AddBox(brick_material, size_brick_x/2, size_brick_y/2, size_brick_z/2) # must set half sizes
        body_brick.GetCollisionModel().BuildModel()
        body_brick.SetCollide(True)

        # Visualization shape, for rendering animation
        body_brick_shape = chrono.ChBoxShape()
        body_brick_shape.GetBoxGeometry().Size = chrono.ChVectorD(size_brick_x/2, size_brick_y/2, size_brick_z/2)
        if iy%2==0 :
            body_brick_shape.SetColor(chrono.ChColor(0.65, 0.65, 0.6)) # set gray color only for odd bricks
        body_brick.AddVisualShape(body_brick_shape)

        sys.Add(body_brick)


# Create the room floor: a simple fixed rigid body with a collision shape
# and a visualization shape

body_floor = chrono.ChBody()
body_floor.SetBodyFixed(True)
body_floor.SetPos(chrono.ChVectorD(0, -2, 0 ))

# Collision shape
body_floor.GetCollisionModel().ClearModel()
body_floor.GetCollisionModel().AddBox(brick_material, 3, 1, 3) # hemi sizes
body_floor.GetCollisionModel().BuildModel()
body_floor.SetCollide(True)

# Visualization shape
body_floor_shape = chrono.ChBoxShape()
body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(3, 1, 3)
body_floor_shape.SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'))
body_floor.AddVisualShape(body_floor_shape)

sys.Add(body_floor)



# Create the shaking table, as a box

size_table_x = 1;
size_table_y = 0.2;
size_table_z = 1;

body_table = chrono.ChBody()
body_table.SetPos(chrono.ChVectorD(0, -size_table_y/2, 0 ))

# Collision shape
body_table.GetCollisionModel().ClearModel()
body_table.GetCollisionModel().AddBox(brick_material, size_table_x/2, size_table_y/2, size_table_z/2) # hemi sizes
body_table.GetCollisionModel().BuildModel()
body_table.SetCollide(True)

# Visualization shape
body_table_shape = chrono.ChBoxShape()
body_table_shape.GetBoxGeometry().Size = chrono.ChVectorD(size_table_x/2, size_table_y/2, size_table_z/2)
body_table_shape.SetColor(chrono.ChColor(0.4,0.4,0.5))
body_table_shape.SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'))
body_table.AddVisualShape(body_table_shape)

sys.Add(body_table)


# Create a constraint that blocks free 3 x y z translations and 3 rx ry rz rotations
# of the table respect to the floor, and impose that the relative imposed position
# depends on a specified motion law.

link_shaker = chrono.ChLinkLockLock()
link_shaker.Initialize(body_table, body_floor, chrono.CSYSNORM)
sys.Add(link_shaker)

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
#  Create an Irrlicht application to visualize the sys
#
vis = chronoirr.ChVisualSystemIrrlicht()
sys.SetVisualSystem(vis)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Earthquake demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5,0.5,1.0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(0, 3, 6),    # point
                       chrono.ChVectorD(0, 3, 6),    # aimpoint
                       9,                 # radius (power)
                       1,9,               # near, far
                       30)                # angle of FOV

##vis.EnableShadows()

# ---------------------------------------------------------------------
#
#  Run the simulation
#

while vis.Run():
    vis.BeginScene() 
    vis.DrawAll()
    vis.EndScene()
    for substep in range(0,5):
        sys.DoStepDynamics(1e-4)


