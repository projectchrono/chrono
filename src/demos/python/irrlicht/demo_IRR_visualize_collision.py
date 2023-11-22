# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2022 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Radu Serban
# =============================================================================
#
# Test the collision drawer callback.  This only tests that the callback can be
# used; there is no actual visualization of the collision shapes but simply
# printing out of the end points of the lines that would be used to render them.
# To visualize the collision shapes, one can use the same feature implemented in
# the underlying Irrlicht visualization (hit the 'i' key and select the check
# box 'Draw Collsion Shapes').
# The global reference frame has Y up.
#
# =============================================================================

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# -----------------------------------------------------------------------------
# Callback class for collision shape visualization
# -----------------------------------------------------------------------------
class DebugDrawer(chrono.VisualizationCallback):
    def __init__(self) : 
        super().__init__()

    def DrawLine(self, pA, pB, color):
        print("   pA = ", pA.x, pA.y, pA.z)
        print("   pB = ", pB.x, pB.y, pB.z)

# -----------------------------------------------------------------------------

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('relative/path/to/data/directory/')

print( "Copyright (c) 2022 projectchrono.org")

# Create sys, contact material, and bodies
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

mat = chrono.ChMaterialSurfaceNSC()

ground = chrono.ChBodyEasyBox(10, 3, 10, 100, True, True, mat)
ground.SetBodyFixed(True);
ground.SetPos(chrono.ChVectorD(0, 0, 0))
sys.AddBody(ground)

cyl = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.5, 1.0, 100, True, True, mat)
cyl.SetPos(chrono.ChVectorD(0, 3, 0))
sys.AddBody(cyl)

box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 100, True, True, mat)
box.SetPos(chrono.ChVectorD(0.2, 2, 0))
sys.AddBody(box)

sphere = chrono.ChBodyEasySphere(0.25, 100.0, True, True, mat)
sphere.SetPos(chrono.ChVectorD(-0.2, 2, 0.75))
sys.AddBody(sphere)

ellipse = chrono.ChBodyEasyEllipsoid(chrono.ChVectorD(0.2, 0.4, 0.6), 100, True, True, mat)
ellipse.SetPos(chrono.ChVectorD(0.2, 2, -1.0))
sys.AddBody(ellipse)

mesh = chrono.ChBodyEasyMesh(chrono.GetChronoDataFile("models/cube.obj"), 100, True, True, True, mat, 0.05)
mesh.SetPos(chrono.ChVectorD(2.0, 3.5, -2.0))
sys.AddBody(mesh)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Collision visualization demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 8 , 6))
vis.AddTypicalLights()

# Create collision shape drawer
drawer = DebugDrawer()
sys.GetCollisionSystem().RegisterVisualizationCallback(drawer)

# Specify what information is visualized
mode = chrono.ChCollisionSystem.VIS_Shapes

use_zbuffer = True

#  Run the simulation
while vis.Run():
    vis.BeginScene() 
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

    print(sys.GetChTime(), "  ", sys.GetNcontacts())
    sys.GetCollisionSystem().Visualize(chrono.ChCollisionSystem.VIS_Shapes)
