#------------------------------------------------------------------------------
# Name:        pychrono example
# Purpose:
#
# Author:      Lijing Yang
#
# Created:     6/10/2020
# Copyright:   (c) ProjectChrono 2019
#------------------------------------------------------------------------------


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
            body.AddAsset(sphere)

            texture = chrono.ChTexture()
            texture.SetTextureFilename(chrono.GetChronoDataFile("bluwhite.png"))
            body.AddAsset(texture)

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
            body.AddAsset(box)

            texture = chrono.ChTexture()
            texture.SetTextureFilename(chrono.GetChronoDataFile("pinkwhite.png"))
            body.AddAsset(texture)

            sys.AddBody(body)

def AddContainerWall(body, mat, size, pos, visible=True):
    hsize = chrono.ChVectorD(0.5 * size.x, 0.5 * size.y, 0.5 * size.z)
    body.GetCollisionModel().AddBox(mat, hsize.x, hsize.y, hsize.z, pos)
    if visible:
        box = chrono.ChBoxShape()
        box.GetBoxGeometry().Pos = pos
        box.GetBoxGeometry().Size = hsize
        body.AddAsset(box)

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

    texture = chrono.ChTexture()
    texture.SetTextureFilename(chrono.GetChronoDataFile("concrete.jpg"))
    fixedBody.AddAsset(texture)

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
    rotatingBody.AddAsset(box)

    rotatingBody.AddAsset(texture)

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
#  Create the simulation system and add items
#

mysystem  = chrono.ChSystemSMC()

# Simulation and rendering time-step
time_step = 1e-4
out_step  = 1.0 / 20

# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(mysystem, 'PyChrono example: Collisions between objects', chronoirr.dimension2du(1024,768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
myapplication.AddTypicalCamera(chronoirr.vector3df(0, 18, -20))
myapplication.AddTypicalLights()

# Add fixed and moving bodies
mixer = AddContainer(mysystem)
AddFallingItems(mysystem)

# ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
# If you need a finer control on which item really needs a visualization proxy in
# Irrlicht, just use application.AssetBind(myitem) on a per-item basis.

myapplication.AssetBindAll()

# ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

myapplication.AssetUpdateAll()

# ---------------------------------------------------------------------
#
#  Run the simulation
#

# Simulation loop
time     = 0.0
out_time = 0.0

while(myapplication.GetDevice().run()):
    mysystem.DoStepDynamics(time_step)
    time = mysystem.GetChTime()
    if (time >= out_time):
        myapplication.BeginScene()
        myapplication.DrawAll()
        myapplication.EndScene()
        out_time += out_step
   
        # print out contact force and torque
        # frc = mixer.GetAppliedForce()
        # trq = mixer.GetAppliedTorque()
        # print(mysystem.GetChTime())
        # print("force: ", frc)
        # print("torque: ", trq)



