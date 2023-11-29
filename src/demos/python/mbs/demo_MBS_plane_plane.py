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

print ("Example: demonstration of a plane-plane joint")

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('relative/path/to/data/directory/')

# ---------------------------------------------------------------------
#
#  Create the simulation sys and add items
#

sys      = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, 0, 0))

# Create the ground body
ground = chrono.ChBodyEasyBox(3, 2, 0.1, 10, True, False)
ground.SetBodyFixed(True)
sys.Add(ground)

# Create the sliding body
# Give an initial angular velocity
body = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 10, True, False)
body.SetBodyFixed(False)
sys.Add(body)
body.SetPos(chrono.ChVectorD(-1.25, -0.75, 0.1))
body.SetWvel_loc(chrono.ChVectorD(0.1, 0.1, 0.1))

body.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.4, 0.1))

# Create the plane-plane constraint
# Constrain the sliding body to move and rotate in the x-y plane
# (i.e. the plane whose normal is the z-axis of the specified coord sys)
plane_plane = chrono.ChLinkLockPlanePlane()
plane_plane.Initialize(ground, 
                       body, 
                       chrono.ChCoordsysD(chrono.ChVectorD(-1.25, -0.75, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.AddLink(plane_plane)

# Create a linear spring (with default sprint & damping coefficients)
spring = chrono.ChLinkTSDA()
spring.SetSpringCoefficient(100)
spring.SetDampingCoefficient(5)
spring.Initialize(ground,
                  body,
                  True,
                  chrono.ChVectorD(0, 0, 2),
                  chrono.ChVectorD(0, 0, 0))
spring.SetRestLength(1.9)
sys.AddLink(spring)

spring.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the sys
#

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('ChLinkLockPlanePlane demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 0 ,3))
vis.AddTypicalLights()


# ---------------------------------------------------------------------
#
#  Run the simulation
#

while vis.Run():
    vis.BeginScene() 
    vis.Render()
    chronoirr.drawAllCOGs(vis, 2)
    vis.EndScene()
    sys.DoStepDynamics(5e-3)





