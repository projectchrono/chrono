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

print ("Example: demonstration of using friction models")

# ---------------------------------------------------------------------
#
#  Create the simulation sys and add items
#

sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create all the rigid bodies.
mradius = 0.5
density = 1000

# Shared visualization materials
vis_mat_ball = chrono.ChVisualMaterial()
vis_mat_ball.SetKdTexture(chrono.GetChronoDataFile('textures/bluewhite.png'))

vis_mat_floor = chrono.ChVisualMaterial()
vis_mat_floor.SetKdTexture(chrono.GetChronoDataFile('textures/blue.png'))


# Create some spheres that roll horizontally, with increasing rolling friction values
for bi in range(10):
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(0.4)
    mat.SetRollingFriction((float(bi) / 10.) * 0.05)

    msphereBody = chrono.ChBodyEasySphere(mradius,  # radius size
                                          1000,     # density
                                          True,     # visualization?
                                          True,     # collision?
                                          mat)      # contact material

    # Set some properties
    msphereBody.SetPos(chrono.ChVector3d(-7, mradius - 0.5, -5 + bi * mradius * 2.5))
    msphereBody.GetVisualShape(0).SetMaterial(0, vis_mat_ball)

    # Set initial speed: rolling in horizontal direction
    initial_angspeed = 10
    initial_linspeed = initial_angspeed * mradius
    msphereBody.SetAngVelParent(chrono.ChVector3d(0, 0, -initial_angspeed))
    msphereBody.SetPosDt(chrono.ChVector3d(initial_linspeed, 0, 0))

    # Add to the sys
    sys.Add(msphereBody)

# Create some spheres that spin on place, for a 'drilling friction' case, with increasing spinning friction values
for bi in range(10):
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(0.4)
    mat.SetSpinningFriction((float(bi) / 10) * 0.02)

    msphereBody = chrono.ChBodyEasySphere(mradius,  # radius size
                                          1000,     # density
                                          True,     # visualization?
                                          True,     # collision?
                                          mat)      # contact material
    # Set some properties
    msphereBody.SetPos(chrono.ChVector3d(-8, 1 + mradius - 0.5, -5 + bi * mradius * 2.5))
    msphereBody.GetVisualShape(0).SetMaterial(0, vis_mat_ball)

    # Set initial speed: spinning in vertical direction
    msphereBody.SetAngVelParent(chrono.ChVector3d(0, 20, 0))

    # Add to the sys
    sys.Add(msphereBody)

    # Notes:
    # - setting nonzero spinning friction and/or setting nonzero rolling friction
    #   affects the speed of the solver (each contact eats 2x of CPU time repsect to the
    #   case of simple sliding/staic contact)
    # - avoid using zero spinning friction with nonzero rolling friction.

# Create a container fixed to ground
bin = chrono.ChBody()
bin.SetPos(chrono.ChVector3d(0, -1, 0))
bin.SetFixed(True)
bin.EnableCollision(True)

# Set rolling and spinning friction coefficients for the container.
# By default, the composite material will use the minimum value for an interacting collision pair.
bin_mat = chrono.ChContactMaterialNSC()
bin_mat.SetRollingFriction(1)
bin_mat.SetSpinningFriction(1)

# Add collision geometry and visualization shapes for the floor and the 4 walls
cshape_1 = chrono.ChCollisionShapeBox(bin_mat, 20, 1, 20)
cshape_2 = chrono.ChCollisionShapeBox(bin_mat, 1,  2, 20.99)
cshape_3 = chrono.ChCollisionShapeBox(bin_mat, 1,  2, 20.99)
cshape_4 = chrono.ChCollisionShapeBox(bin_mat, 20.99, 2, 1)
cshape_5 = chrono.ChCollisionShapeBox(bin_mat, 20.99, 2, 1)

bin.AddCollisionShape(cshape_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)  , chrono.QUNIT))
bin.AddCollisionShape(cshape_2, chrono.ChFramed(chrono.ChVector3d(-10, 1, 0), chrono.QUNIT))
bin.AddCollisionShape(cshape_3, chrono.ChFramed(chrono.ChVector3d( 10, 1, 0), chrono.QUNIT))
bin.AddCollisionShape(cshape_4, chrono.ChFramed(chrono.ChVector3d(0, 1, -10), chrono.QUNIT))
bin.AddCollisionShape(cshape_5, chrono.ChFramed(chrono.ChVector3d(0, 1,  10), chrono.QUNIT))

vshape_1 = chrono.ChVisualShapeBox(20, 1, 20)
vshape_1.SetMaterial(0, vis_mat_floor)
bin.AddVisualShape(vshape_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

vshape_2 = chrono.ChVisualShapeBox(1, 2, 20.99)
vshape_2.SetMaterial(0, vis_mat_floor)
bin.AddVisualShape(vshape_2, chrono.ChFramed(chrono.ChVector3d(-10, 1, 0)))

vshape_3 = chrono.ChVisualShapeBox(1, 2, 20.99)
vshape_3.SetMaterial(0, vis_mat_floor)
bin.AddVisualShape(vshape_3, chrono.ChFramed(chrono.ChVector3d(10, 1, 0)))

vshape_4 = chrono.ChVisualShapeBox(20.99, 2, 1)
vshape_4.SetMaterial(0, vis_mat_floor)
bin.AddVisualShape(vshape_4, chrono.ChFramed(chrono.ChVector3d(0, 1, -10)))

vshape_5 = chrono.ChVisualShapeBox(20.99, 2, 1)
vshape_5.SetMaterial(0, vis_mat_floor)
bin.AddVisualShape(vshape_5, chrono.ChFramed(chrono.ChVector3d(0, 1, 10)))

sys.Add(bin)

# ---------------------------------------------------------------------
# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Friction demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_chrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20))
vis.AddTypicalLights()


#  Run the simulation
sys.SetSolverType(chrono.ChSolver.Type_APGD)

while vis.Run():
    vis.BeginScene() 
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(5e-3)





