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
    sph_mat = chrono.ChMaterialSurfaceNSC()
    sph_mat.SetFriction(0.2)
    box_mat = chrono.ChMaterialSurfaceNSC()
    cyl_mat = chrono.ChMaterialSurfaceNSC()

    # Create falling rigid bodies (spheres and boxes etc.)
    for bi in range(29):
        msphereBody = chrono.ChBodyEasySphere(1.1,      # radius size
                                              1000,     # density
                                              True,     # visualization?
                                              True,     # collision?
                                              sph_mat)  # contact material
        msphereBody.SetPos(chrono.ChVectorD(-5 + chrono.ChRandom() * 10, 4 + bi * 0.05, -5 + chrono.ChRandom() * 10))
        sys.Add(msphereBody)

        mtexture = chrono.ChTexture()
        mtexture.SetTextureFilename(chrono.GetChronoDataFile("bluwhite.png"))
        msphereBody.AddAsset(mtexture)

        mboxBody = chrono.ChBodyEasyBox(1.5, 1.5, 1.5, # x,y,z size
                                        100,           # density
                                        True,          # visualization?
                                        True,          # collision?
                                        box_mat)       # contact material
        mboxBody.SetPos(chrono.ChVectorD(-5 + chrono.ChRandom() * 10, 4 + bi * 0.05, -5 + chrono.ChRandom() * 10))

        sys.Add(mboxBody)

        mtexturebox = chrono.ChTexture()
        mtexturebox.SetTextureFilename(chrono.GetChronoDataFile("cubetexture_bluwhite.png"))
        mboxBody.AddAsset(mtexturebox)

        mcylBody = chrono.ChBodyEasyCylinder(0.75, 0.5, # radius, height
                                             100,       # density
                                             True,      # visualization?
                                             True,      # collision?
                                             cyl_mat)   # contact material
        mcylBody.SetPos(chrono.ChVectorD(-5 + chrono.ChRandom() * 10, 4 + bi * 0.05, -5 + chrono.ChRandom() * 10))

        sys.Add(mcylBody)

        # optional, attach a texture for better visualization
        mtexturecyl = chrono.ChTexture()
        mtexturecyl.SetTextureFilename(chrono.GetChronoDataFile("pinkwhite.png"))
        mcylBody.AddAsset(mtexturecyl)

def AddContainer(sys):
    # Contact material for container
    ground_mat = chrono.ChMaterialSurfaceNSC()

    # Create the five walls of the rectangular container, using fixed rigid bodies of 'box' type
    floorBody = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True, ground_mat)
    floorBody.SetPos(chrono.ChVectorD(0, -5, 0))
    floorBody.SetBodyFixed(True)
    sys.Add(floorBody)

    wallBody1 = chrono.ChBodyEasyBox(1, 10, 20.99, 1000, True, True, ground_mat)
    wallBody1.SetPos(chrono.ChVectorD(-10, 0, 0))
    wallBody1.SetBodyFixed(True)
    sys.Add(wallBody1)

    wallBody2 = chrono.ChBodyEasyBox(1, 10, 20.99, 1000, True, True, ground_mat)
    wallBody2.SetPos(chrono.ChVectorD(10, 0, 0))
    wallBody2.SetBodyFixed(True)
    sys.Add(wallBody2)

    wallBody3 = chrono.ChBodyEasyBox(20.99, 10, 1, 1000, False, True, ground_mat)
    wallBody3.SetPos(chrono.ChVectorD(0, 0, -10))
    wallBody3.SetBodyFixed(True)
    sys.Add(wallBody3)

    wallBody4 = chrono.ChBodyEasyBox(20.99, 10, 1, 1000, True, True, ground_mat)
    wallBody4.SetPos(chrono.ChVectorD(0, 0, 10))
    wallBody4.SetBodyFixed(True)
    sys.Add(wallBody4)

    # optional, attach  textures for better visualization
    mtexturewall = chrono.ChTexture()
    mtexturewall.SetTextureFilename(chrono.GetChronoDataFile("concrete.jpg"))
    wallBody1.AddAsset(mtexturewall)  # note: most assets can be shared
    wallBody2.AddAsset(mtexturewall)
    wallBody3.AddAsset(mtexturewall)
    wallBody4.AddAsset(mtexturewall)
    floorBody.AddAsset(mtexturewall)

    # Add the rotating mixer
    mixer_mat = chrono.ChMaterialSurfaceNSC()
    mixer_mat.SetFriction(0.4)

    rotatingBody = chrono.ChBodyEasyBox(10, 5, 1,  # x,y,z size
                                        4000,      # density
                                        True,      # visualization?
                                        True,      # collision?
                                        mixer_mat) # contact material
    rotatingBody.SetPos(chrono.ChVectorD(0, -1.6, 0))
    sys.Add(rotatingBody)

    # .. a motor between mixer and truss
    my_motor = chrono.ChLinkMotorRotationSpeed()
    my_motor.Initialize(rotatingBody,
                        floorBody, 
                        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), 
                            chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X)))
    mfun = chrono.ChFunction_Const(chrono.CH_C_PI / 4.0)  # speed 45 deg/s 
    my_motor.SetSpeedFunction(mfun)
    sys.AddLink(my_motor)

    # NOTE: Instead of creating five separate 'box' bodies to make
    # the walls of the container, you could have used a single body
    # made of five box shapes, which build a single collision description,
    # as in the alternative approach:

    """
    # create a plain ChBody (no colliding shape nor visualization mesh is used yet)
    mrigidBody = chrono.ChBody()

    # set as fixed body, and turn collision ON, otherwise no collide by default
    mrigidBody.SetBodyFixed(True)
    mrigidBody.SetCollide(True)

    # Clear model. The colliding shape description MUST be between  ClearModel() .. BuildModel() pair.
    mrigidBody.GetCollisionModel().ClearModel()
    # Describe the (invisible) colliding shape by adding five boxes (the walls and floor)
    mrigidBody.GetCollisionModel().AddBox(ground_mat, 20, 1, 20, chrono.ChVectorD(0, -10, 0))
    mrigidBody.GetCollisionModel().AddBox(ground_mat, 1, 40, 20, chrono.ChVectorD(-11, 0, 0))
    mrigidBody.GetCollisionModel().AddBox(ground_mat, 1, 40, 20, chrono.ChVectorD(11, 0, 0))
    mrigidBody.GetCollisionModel().AddBox(ground_mat, 20, 40, 1, chrono.ChVectorD(0, 0, -11))
    mrigidBody.GetCollisionModel().AddBox(ground_mat, 20, 40, 1, chrono.ChVectorD(0, 0, 11))
    # Complete the description of collision shape.
    mrigidBody.GetCollisionModel().BuildModel()

    # Attach some visualization shapes if needed:
    vshape = chrono.ChBoxShape()
    vshape.GetBoxGeometry().SetLengths(chrono.ChVectorD(20, 1, 20))
    vshape.GetBoxGeometry().Pos = chrono.ChVectorD(0, -5, 0)
    this.AddAsset(vshape)
    # etc. for other 4 box shapes..
    """

    return rotatingBody

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

mysystem      = chrono.ChSystemNSC()

# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(mysystem, 'PyChrono example: Collisions between objects', chronoirr.dimension2du(1024,768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
myapplication.AddTypicalCamera(chronoirr.vector3df(0, 14 , -20))
myapplication.AddTypicalLights()

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

# Modify some setting of the physical syustem for the simulation, if you want
mysystem.SetSolverType(chrono.ChSolver.Type_PSOR)
mysystem.SetSolverMaxIterations(20)

# ---------------------------------------------------------------------
#
#  Run the simulation
#

myapplication.SetTimestep(0.02)
myapplication.SetTryRealtime(True)

while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()
   
    # print out contact force and torque
    # frc = mixer.GetAppliedForce()
    # trq = mixer.GetAppliedTorque()
    # print(mysystem.GetChTime())
    # print("force: ", frc)
    # print("torque: ", trq)
    # c_frc = mixer.GetContactForce()
    # c_trq = mixer.GetContactTorque()
    # print(mysystem.GetChTime())
    # print("contact force: ", c_frc)
    # print("contact torque: ", c_trq)




