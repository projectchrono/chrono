# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2024 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Author: Josh Diyn
# =============================================================================

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

print("Example: Simulate a cylinder joined to a cube using a bushing joint and loaded with a hanging weighted body")

# Create chrono system
sys = chrono.ChSystemNSC()

# Create a fixed cube to act as the 'bushing' body
cube = chrono.ChBodyEasyBox(0.5,0.5,0.5,100,True)
cube.SetFixed(True)
cube.SetPos(chrono.ChVector3d(0, 0, 0))
cube.GetVisualShape(0).SetColor(chrono.ChColor(0, 0, 0))
sys.Add(cube)

# Create a cylinder which will be connected to the cube via a bushing
cylinder = chrono.ChBodyEasyCylinder(2, 0.2, 1.0, 200, True)
cylinder.SetFixed(False)
cylinder.SetPos(chrono.ChVector3d(0, 0, 0))
sys.Add(cylinder)

# Create a nonfixed weight
weight = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 1000, True)
weight.SetFixed(False)
weight.SetPos(chrono.ChVector3d(0, -1, 0.5))
weight.SetMass(50)
weight.GetVisualShape(0).SetColor(chrono.ChColor(0.1, 0.9, 0.1))
sys.Add(weight)

#create a linkage joint
link = chrono.ChLinkDistance()
point_on_weight = chrono.ChVector3d(0, 0, 0)
point_on_cylinder = chrono.ChVector3d(0, 0, 0.5)

# Note: Using absolute points, not local to bodies
link.Initialize(weight, cylinder, True, point_on_weight, point_on_cylinder)
sys.Add(link)

# Create a visual line of the distance link
line_visual = chrono.ChVisualShapeLine()
line_visual.SetColor(chrono.ChColor(0.1, 0.9, 0.1))

# Get endpoints and set the visual shape
pos1 = link.GetEndPoint1Abs()
pos2 = link.GetEndPoint2Abs()
line_visual.SetLineGeometry(chrono.ChLineSegment(pos1, pos2))
# attach the line to the stationary/fixed cube
cube.AddVisualShape(line_visual)

# Create the bushing joint
bushing = chrono.ChLinkBushing()
# testing ChMatrix66d SWIG translation
kFactor = chrono.ChMatrix66d()
rFactor = chrono.ChMatrix66d()
# Set stiffness and damping
translationalStiffness = 50000
translationalDamping = 1000
rotationalStiffness = 3000
rotationalDamping = 20

# insert into matrices
for i in range(3):
    kFactor.SetItem(i, i, translationalStiffness)
    rFactor.SetItem(i, i, translationalDamping)
for i in range(3, 6):
    kFactor.SetItem(i, i, rotationalStiffness)
    rFactor.SetItem(i, i, rotationalDamping)

quat = chrono.ChQuaterniond()
quat.SetFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))  # No rotation
mframe = chrono.ChFramed(chrono.ChVector3d(0, 0.5, 0), quat)
bushing.Initialize(cube, cylinder, mframe, kFactor, rFactor)
sys.Add(bushing)

# create the irrlicht window
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Cylinder through Cube with Bushing Joint')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.5, 0.5, 1.5))
vis.AddTypicalLights()


# Run the sim
while vis.Run():
    vis.BeginScene()
    
    # basic updating of the link visual
    pos1 = link.GetEndPoint1Abs()
    pos2 = link.GetEndPoint2Abs()
    line_visual.SetLineGeometry(chrono.ChLineSegment(pos1, pos2))
    
    vis.Render()
    
    vis.EndScene()
    sys.DoStepDynamics(5e-3)
