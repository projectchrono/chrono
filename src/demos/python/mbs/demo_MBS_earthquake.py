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
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


# Set the default outward/inward shape margins for collision detection,
# this is epecially important for very large or very small objects.
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)

# Maybe you want to change some settings for the solver. For example you
# might want to use SetSolverMaxIterations to set the number of iterations
# per timestep, etc.

#sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) # precise, more slow
sys.GetSolver().AsIterative().SetMaxIterations(70)



# Create a contact material (surface property)to share between all objects.
# The rolling and spinning parameters are optional - if enabled they double
# the computational time.
brick_material = chrono.ChContactMaterialNSC()
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
        body_brick.SetPos(chrono.ChVector3d(ix*size_brick_x, (iy+0.5)*size_brick_y, 0 ))
        # set mass properties
        body_brick.SetMass(mass_brick)
        body_brick.SetInertiaXX(chrono.ChVector3d(inertia_brick,inertia_brick,inertia_brick))       

        # Collision shape
        body_brick_ct_shape = chrono.ChCollisionShapeBox(brick_material, size_brick_x, size_brick_y, size_brick_z)
        body_brick.AddCollisionShape(body_brick_ct_shape)
        body_brick.EnableCollision(True)

        # Visualization shape, for rendering animation
        body_brick_shape = chrono.ChVisualShapeBox(size_brick_x, size_brick_y, size_brick_z)
        if iy%2==0 :
            body_brick_shape.SetColor(chrono.ChColor(0.65, 0.65, 0.6)) # set gray color only for odd bricks
        body_brick.AddVisualShape(body_brick_shape)

        sys.Add(body_brick)


# Create the room floor: a simple fixed rigid body with a collision shape
# and a visualization shape

body_floor = chrono.ChBody()
body_floor.SetFixed(True)
body_floor.SetPos(chrono.ChVector3d(0, -2, 0 ))

# Collision shape
body_floor_ct_shape = chrono.ChCollisionShapeBox(brick_material, 6, 2, 6)
body_floor.AddCollisionShape(body_floor_ct_shape)
body_floor.EnableCollision(True)

# Visualization shape
body_floor_shape = chrono.ChVisualShapeBox(6, 2, 6)
body_floor_shape.SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'))
body_floor.AddVisualShape(body_floor_shape)

sys.Add(body_floor)



# Create the shaking table, as a box

size_table_x = 1;
size_table_y = 0.2;
size_table_z = 1;

body_table = chrono.ChBody()
body_table.SetPos(chrono.ChVector3d(0, -size_table_y/2, 0 ))

# Collision shape
body_table_ct_shape = chrono.ChCollisionShapeBox(brick_material, size_table_x, size_table_y, size_table_z)
body_table.AddCollisionShape(body_table_ct_shape)
body_table.EnableCollision(True)

# Visualization shape
body_table_shape = chrono.ChVisualShapeBox(size_table_x, size_table_y, size_table_z)
body_table_shape.SetColor(chrono.ChColor(0.4,0.4,0.5))
body_table_shape.SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'))
body_table.AddVisualShape(body_table_shape)

sys.Add(body_table)


# Create a constraint that blocks free 3 x y z translations and 3 rx ry rz rotations
# of the table respect to the floor, and impose that the relative imposed position
# depends on a specified motion law.

link_shaker = chrono.ChLinkLockLock()
link_shaker.Initialize(body_table, body_floor, chrono.ChFramed())
sys.Add(link_shaker)

# ..create the function for imposed x horizontal motion, etc.
mfunY = chrono.ChFunctionSine(0.001,1.5)  # amplitude, frequency
link_shaker.SetMotionY(mfunY)

# ..create the function for imposed y vertical motion, etc.
mfunZ = chrono.ChFunctionSine(0.12,1.5)  # amplitude, frequency
link_shaker.SetMotionZ(mfunZ)

# Note that you could use other types of ChFunction objects, or create
# your custom function by class inheritance (see demo_python.py), or also
# set a function for table rotation , etc.




# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the sys
#
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Earthquake demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_chrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5,0.5,1.0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(0, 3, 6),    # point
                       chrono.ChVector3d(0, 3, 6),    # aimpoint
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
    vis.Render()
    vis.EndScene()
    for substep in range(0,5):
        sys.DoStepDynamics(1e-4)


