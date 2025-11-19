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

print ("Example: demonstration of collisions and contacts")

def AddFallingItems(sys):
    # Shared contact materials for falling objects
    mat = chrono.ChContactMaterialSMC()

    # Create falling rigid bodies (spheres and boxes etc.)
    for ix in range(-2, 3):
        for iz in range(-2, 3):
            # add spheres
            mass = 1
            radius = 1.1
            body = chrono.ChBody()
            comp = (2.0 / 5.0) * mass * radius**2
            body.SetInertiaXX(chrono.ChVector3d(comp, comp, comp))
            body.SetMass(mass)
            body.SetPos(chrono.ChVector3d(4.0 * ix + 0.1, 4.0, 4.0 * iz))

            body_ct_shape = chrono.ChCollisionShapeSphere(mat, radius)
            body.AddCollisionShape(body_ct_shape)
            body.EnableCollision(True)

            sphere = chrono.ChVisualShapeSphere(radius)
            sphere.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
            body.AddVisualShape(sphere)

            sys.AddBody(body)

            # add boxes
            mass = 1
            size = chrono.ChVector3d(1.5, 1.5, 1.5)
            body = chrono.ChBody()

            body.SetMass(mass)
            body.SetPos(chrono.ChVector3d(4.0 * ix, 6.0, 4.0 * iz))

            body_ct_shape = chrono.ChCollisionShapeBox(mat, size.x, size.y, size.z)
            body.AddCollisionShape(body_ct_shape)
            body.EnableCollision(True)

            box = chrono.ChVisualShapeBox(size)
            box.SetTexture(chrono.GetChronoDataFile("textures/pinkwhite.png"))
            body.AddVisualShape(box)

            sys.AddBody(body)

def AddContainerWall(body, mat, size, pos, visible=True):
    body_ct_shape = chrono.ChCollisionShapeBox(mat, size.x, size.y, size.z)
    body.AddCollisionShape(body_ct_shape, chrono.ChFramed(pos, chrono.QUNIT))
    if visible:
        box = chrono.ChVisualShapeBox(size)
        box.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
        body.AddVisualShape(box, chrono.ChFramed(pos))

def AddContainer(sys):
    # The fixed body (5 walls)
    fixedBody = chrono.ChBody()

    fixedBody.SetMass(1.0)
    fixedBody.SetFixed(True)
    fixedBody.SetPos(chrono.ChVector3d())
    fixedBody.EnableCollision(True)

    # Contact material for container
    fixed_mat = chrono.ChContactMaterialSMC()

    AddContainerWall(fixedBody, fixed_mat, chrono.ChVector3d(20, 1, 20), chrono.ChVector3d(0, -5, 0))
    AddContainerWall(fixedBody, fixed_mat, chrono.ChVector3d(1, 10, 20.99), chrono.ChVector3d(-10, 0, 0))
    AddContainerWall(fixedBody, fixed_mat, chrono.ChVector3d(1, 10, 20.99), chrono.ChVector3d(10, 0, 0))
    AddContainerWall(fixedBody, fixed_mat, chrono.ChVector3d(20.99, 10, 1), chrono.ChVector3d(0, 0, -10), False)
    AddContainerWall(fixedBody, fixed_mat, chrono.ChVector3d(20.99, 10, 1), chrono.ChVector3d(0, 0, 10))

    sys.AddBody(fixedBody)

    # The rotating mixer body
    rotatingBody = chrono.ChBody()

    rotatingBody.SetMass(10.0)
    rotatingBody.SetInertiaXX(chrono.ChVector3d(50, 50, 50))
    rotatingBody.SetPos(chrono.ChVector3d(0, -1.6, 0))
    rotatingBody.EnableCollision(True)

    # Contact material for mixer body
    mixer_mat = chrono.ChContactMaterialSMC()

    rotatingBody = chrono.ChBodyEasyBox(14, 6, 1,  # x,y,z size
                                        4000,      # density
                                        True,      # visualization?
                                        True,      # collision?
                                        mixer_mat) # contact material
    rotatingBody.SetPos(chrono.ChVector3d(0, -1.6, 0))
    sys.Add(rotatingBody)


    # A motor between the two
    my_motor = chrono.ChLinkMotorRotationSpeed()

    my_motor.Initialize(rotatingBody,
                        fixedBody,
                        chrono.ChFramed(chrono.ChVector3d(0, 0, 0), 
                                        chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_X)))
    mfun = chrono.ChFunctionConst(chrono.CH_PI / 2.0)  # speed w=90°/s
    my_motor.SetSpeedFunction(mfun)

    sys.AddLink(my_motor)

    return rotatingBody

# ---------------------------------------------------------------------
#
#  Create the simulation sys and add items
#

sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Simulation and rendering time-step
time_step = 1e-4
out_step  = 1.0 / 20

# Add fixed and moving bodies
mixer = AddContainer(sys)
AddFallingItems(sys)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Collisions between objects')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_chrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 18, -20))
vis.AddTypicalLights()

# Simulation loop
time     = 0.0
out_time = 0.0

while vis.Run():
    sys.DoStepDynamics(time_step)
    time = sys.GetChTime()
    if (time >= out_time):
        vis.BeginScene() 
        vis.Render()
        vis.EndScene()
        out_time += out_step
   
        # print out contact force and torque
        # frc = mixer.GetAppliedForce()
        # trq = mixer.GetAppliedTorque()
        # print(sys.GetChTime())
        # print("force: ", frc)
        # print("torque: ", trq)



