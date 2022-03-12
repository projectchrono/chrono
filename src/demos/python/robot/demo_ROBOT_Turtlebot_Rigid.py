# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2021 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Jason Zhou
# =============================================================================
#
# Demo to show a Turtlebot Robot operated on Rigid Terrain
#
# =============================================================================

import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
try:
    from pychrono import irrlicht as chronoirr
except:
    print('Could not import ChronoIrrlicht')

# Chreate Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body
ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -1))
ground.SetBodyFixed(True)
system.Add(ground)

texture = chrono.ChTexture()
texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
ground.AddAsset(texture)

# Create Turtlebot Robot
robot = turtlebot.TurtleBot(system, chrono.ChVectorD(
    0, 0, -0.45), chrono.ChQuaternionD(1, 0, 0, 0))
robot.Initialize()

# Create run-time visualization
application = chronoirr.ChIrrApp(system, "Turtlebot Robot - Rigid terrain",
                                 chronoirr.dimension2du(1280, 720), chronoirr.VerticalDir_Z)
application.AddLogo(
    chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
application.AddSkyBox()
application.AddCamera(chronoirr.vector3df(0, 0.5, 0.5), chronoirr.vector3df(0, 0, 0))
application.AddTypicalLights()
application.AddLightWithShadow(chronoirr.vector3df(1.5, -2.5, 5.5), chronoirr.vector3df(0, 0, 0), 3, 4, 10, 40, 512)

application.AssetBindAll()
application.AssetUpdateAll()
application.AddShadowAll()

time_step = 2e-3
application.SetTimestep(time_step)

# Simulation loop
time = 0
while (application.GetDevice().run()):
    # since enum is not available in python wrapper
    # WheelID enum has to be explicitly defined:
    # Left Active Drive Wheel (LD) = 0
    # Right Active Drive Wheel (RD) = 1

    # at time = 1 s, start left turn
    if abs(time - 1.0) < 1e-4:
        robot.SetMotorSpeed(-0, 0)
        robot.SetMotorSpeed(-math.pi, 1)
    # at time = 2 s, start right turn
    if (abs(time - 2.0) < 1e-4):
        robot.SetMotorSpeed(-math.pi, 0)
        robot.SetMotorSpeed(-0, 1)

    # increment time counter
    time = time + time_step

    application.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
    application.DrawAll()
    application.DoStep()
    application.EndScene()
