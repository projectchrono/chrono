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
        msphereBody.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
        sys.Add(msphereBody)

        mboxBody = chrono.ChBodyEasyBox(1.5, 1.5, 1.5, # x,y,z size
                                        100,           # density
                                        True,          # visualization?
                                        True,          # collision?
                                        box_mat)       # contact material
        mboxBody.SetPos(chrono.ChVectorD(-5 + chrono.ChRandom() * 10, 4 + bi * 0.05, -5 + chrono.ChRandom() * 10))
        mboxBody.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/cubetexture_bluewhite.png"))
        sys.Add(mboxBody)

        mcylBody = chrono.ChBodyEasyCylinder(0.75, 0.5, # radius, height
                                             100,       # density
                                             True,      # visualization?
                                             True,      # collision?
                                             cyl_mat)   # contact material
        mcylBody.SetPos(chrono.ChVectorD(-5 + chrono.ChRandom() * 10, 4 + bi * 0.05, -5 + chrono.ChRandom() * 10))
        mcylBody.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/pinkwhite.png"))
        sys.Add(mcylBody)

def AddContainer(sys):
    # Contact material for container
    ground_mat = chrono.ChMaterialSurfaceNSC()

    # Visualization material for container
    ground_vis_mat = chrono.ChVisualMaterial()
    ground_vis_mat.SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))

    # Create the five walls of the rectangular container, using fixed rigid bodies of 'box' type
    floorBody = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True, ground_mat)
    floorBody.SetPos(chrono.ChVectorD(0, -5, 0))
    floorBody.SetBodyFixed(True)
    floorBody.GetVisualShape(0).SetMaterial(0, ground_vis_mat)
    sys.Add(floorBody)

    wallBody1 = chrono.ChBodyEasyBox(1, 10, 20.99, 1000, True, True, ground_mat)
    wallBody1.SetPos(chrono.ChVectorD(-10, 0, 0))
    wallBody1.SetBodyFixed(True)
    wallBody1.GetVisualShape(0).SetMaterial(0, ground_vis_mat)
    sys.Add(wallBody1)

    wallBody2 = chrono.ChBodyEasyBox(1, 10, 20.99, 1000, True, True, ground_mat)
    wallBody2.SetPos(chrono.ChVectorD(10, 0, 0))
    wallBody2.SetBodyFixed(True)
    wallBody2.GetVisualShape(0).SetMaterial(0, ground_vis_mat)
    sys.Add(wallBody2)

    wallBody3 = chrono.ChBodyEasyBox(20.99, 10, 1, 1000, False, True, ground_mat)
    wallBody3.SetPos(chrono.ChVectorD(0, 0, -10))
    wallBody3.SetBodyFixed(True)
    sys.Add(wallBody3)

    wallBody4 = chrono.ChBodyEasyBox(20.99, 10, 1, 1000, True, True, ground_mat)
    wallBody4.SetPos(chrono.ChVectorD(0, 0, 10))
    wallBody4.SetBodyFixed(True)
    wallBody4.GetVisualShape(0).SetMaterial(0, ground_vis_mat)
    sys.Add(wallBody4)

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

    return rotatingBody

# ---------------------------------------------------------------------

#  Create the simulation sys and add items
sys = chrono.ChSystemNSC()

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
vis.AddCamera(chrono.ChVectorD(0, 14 , -20))
vis.AddTypicalLights()

# Modify some setting of the physical syustem for the simulation, if you want
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.SetSolverMaxIterations(20)

#  Run the simulation
while vis.Run():
    vis.BeginScene() 
    vis.DrawAll()
    vis.EndScene()
    sys.DoStepDynamics(0.02)
   
    # print out contact force and torque
    # frc = mixer.GetAppliedForce()
    # trq = mixer.GetAppliedTorque()
    # print(sys.GetChTime())
    # print("force: ", frc)
    # print("torque: ", trq)
    # c_frc = mixer.GetContactForce()
    # c_trq = mixer.GetContactTorque()
    # print(sys.GetChTime())
    # print("contact force: ", c_frc)
    # print("contact torque: ", c_trq)




