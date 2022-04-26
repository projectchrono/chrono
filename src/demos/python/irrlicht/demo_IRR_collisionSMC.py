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

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('relative/path/to/data/directory/')

def AddFallingItems(sys):
    # Shared contact materials for falling objects
    mat = chrono.ChMaterialSurfaceSMC()

    # Create falling rigid bodies (spheres and boxes etc.)
    for ix in range(-2, 3):
        for iz in range(-2, 3):
            # add spheres
            mass = 1
            radius = 1.1
            body = chrono.ChBody()
            comp = (2.0 / 5.0) * mass * radius**2
            body.SetInertiaXX(chrono.ChVectorD(comp, comp, comp))
            body.SetMass(mass)
            body.SetPos(chrono.ChVectorD(4.0 * ix + 0.1, 4.0, 4.0 * iz))

            body.GetCollisionModel().ClearModel()
            body.GetCollisionModel().AddSphere(mat, radius)
            body.GetCollisionModel().BuildModel()
            body.SetCollide(True)

            sphere = chrono.ChSphereShape()
            sphere.GetSphereGeometry().rad = radius
            sphere.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
            body.AddVisualShape(sphere)

            sys.AddBody(body)

            # add boxes
            mass = 1
            hsize = chrono.ChVectorD(0.75, 0.75, 0.75)
            body = chrono.ChBody()

            body.SetMass(mass)
            body.SetPos(chrono.ChVectorD(4.0 * ix, 6.0, 4.0 * iz))

            body.GetCollisionModel().ClearModel()
            body.GetCollisionModel().AddBox(mat, hsize.x, hsize.y, hsize.z)
            body.GetCollisionModel().BuildModel()
            body.SetCollide(True)

            box = chrono.ChBoxShape()
            box.GetBoxGeometry().Size = hsize
            box.SetTexture(chrono.GetChronoDataFile("textures/pinkwhite.png"))
            body.AddVisualShape(box)

            sys.AddBody(body)

def AddContainerWall(body, mat, size, pos, visible=True):
    hsize = chrono.ChVectorD(0.5 * size.x, 0.5 * size.y, 0.5 * size.z)
    body.GetCollisionModel().AddBox(mat, hsize.x, hsize.y, hsize.z, pos)
    if visible:
        box = chrono.ChBoxShape()
        box.GetBoxGeometry().Size = hsize
        box.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
        body.AddVisualShape(box, chrono.ChFrameD(pos))

def AddContainer(sys):
    # The fixed body (5 walls)
    fixedBody = chrono.ChBody()

    fixedBody.SetMass(1.0)
    fixedBody.SetBodyFixed(True)
    fixedBody.SetPos(chrono.ChVectorD())
    fixedBody.SetCollide(True)

    # Contact material for container
    fixed_mat = chrono.ChMaterialSurfaceSMC()

    fixedBody.GetCollisionModel().ClearModel()
    AddContainerWall(fixedBody, fixed_mat, chrono.ChVectorD(20, 1, 20), chrono.ChVectorD(0, -5, 0))
    AddContainerWall(fixedBody, fixed_mat, chrono.ChVectorD(1, 10, 20.99), chrono.ChVectorD(-10, 0, 0))
    AddContainerWall(fixedBody, fixed_mat, chrono.ChVectorD(1, 10, 20.99), chrono.ChVectorD(10, 0, 0))
    AddContainerWall(fixedBody, fixed_mat, chrono.ChVectorD(20.99, 10, 1), chrono.ChVectorD(0, 0, -10), False)
    AddContainerWall(fixedBody, fixed_mat, chrono.ChVectorD(20.99, 10, 1), chrono.ChVectorD(0, 0, 10))
    fixedBody.GetCollisionModel().BuildModel()

    sys.AddBody(fixedBody)

    # The rotating mixer body
    rotatingBody = chrono.ChBody()

    rotatingBody.SetMass(10.0)
    rotatingBody.SetInertiaXX(chrono.ChVectorD(50, 50, 50))
    rotatingBody.SetPos(chrono.ChVectorD(0, -1.6, 0))
    rotatingBody.SetCollide(True)

    # Contact material for mixer body
    rot_mat = chrono.ChMaterialSurfaceSMC()

    hsize = chrono.ChVectorD(5, 2.75, 0.5)

    rotatingBody.GetCollisionModel().ClearModel()
    rotatingBody.GetCollisionModel().AddBox(rot_mat, hsize.x, hsize.y, hsize.z)
    rotatingBody.GetCollisionModel().BuildModel()

    box = chrono.ChBoxShape()
    box.GetBoxGeometry().Size = hsize
    rotatingBody.AddVisualShape(box)

    sys.AddBody(rotatingBody)

    # A motor between the two
    my_motor = chrono.ChLinkMotorRotationSpeed()

    my_motor.Initialize(rotatingBody,
                        fixedBody,
                        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), 
                            chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X)))
    mfun = chrono.ChFunction_Const(chrono.CH_C_PI / 2.0)  # speed w=90°/s
    my_motor.SetSpeedFunction(mfun)

    sys.AddLink(my_motor)

    return rotatingBody

# ---------------------------------------------------------------------
#
#  Create the simulation sys and add items
#

sys  = chrono.ChSystemSMC()

# Simulation and rendering time-step
time_step = 1e-4
out_step  = 1.0 / 20

# Add fixed and moving bodies
mixer = AddContainer(sys)
AddFallingItems(sys)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
sys.SetVisualSystem(vis)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Collisions between objects')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 18, -20))
vis.AddTypicalLights()

# Simulation loop
time     = 0.0
out_time = 0.0

while vis.Run():
    sys.DoStepDynamics(time_step)
    time = sys.GetChTime()
    if (time >= out_time):
        vis.BeginScene() 
        vis.DrawAll()
        vis.EndScene()
        out_time += out_step
   
        # print out contact force and torque
        # frc = mixer.GetAppliedForce()
        # trq = mixer.GetAppliedTorque()
        # print(sys.GetChTime())
        # print("force: ", frc)
        # print("torque: ", trq)



