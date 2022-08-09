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
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
robot = turtlebot.TurtleBot(system, chrono.ChVectorD(
    0, 0, -0.45), chrono.ChQuaternionD(1, 0, 0, 0))
robot.Initialize()

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1.5, 0.2), chrono.ChVectorD(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)

####vis.EnableShadows()

time_step = 2e-3

# Simulation loop
time = 0
while (vis.Run()) :
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

    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)
