# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
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
# Demonstration of using rotation limits on a revolute joint. Note that this
# capability is only available for ChLinkLockRevolute.  It is not available
# for ChLinkRevolute (which uses a different formulation).
#
# Recall that Irrlicht uses a left-hand frame, so everything is rendered with
# left and right flipped.
#
# =============================================================================

import pychrono as chrono
import pychrono.irrlicht as irr
import math as m

print("Copyright (c) 2017 projectchrono.org")

sys = chrono.ChSystemNSC()

# Create ground body
ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Visualization for revolute joint
cyl_rev = chrono.ChVisualShapeCylinder(0.04, 0.4)
ground.AddVisualShape(cyl_rev)

# Create a pendulum body
pend = chrono.ChBody()
sys.AddBody(pend)
pend.SetFixed(False)
pend.EnableCollision(False)
pend.SetMass(1)
pend.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))

# Initial position of the pendulum (horizontal, pointing towards positive X).
pend.SetPos(chrono.ChVector3d(1.5, 0, 0))

# Attach visualization assets.
cyl_p = chrono.ChVisualShapeCylinder(0.2, 2.92)
cyl_p.SetColor(chrono.ChColor(0.6, 0, 0))
pend.AddVisualShape(cyl_p, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Create a revolute joint to connect pendulum to ground
rev = chrono.ChLinkLockRevolute()
sys.AddLink(rev)

# Add limits to the Z rotation of the revolute joint
min_angle = 0
max_angle = 0.75 * m.pi
rev.LimitRz().SetActive(True)
rev.LimitRz().SetMin(min_angle)
rev.LimitRz().SetMax(max_angle)

# Initialize the joint specifying a coordinate sys (expressed in the absolute frame).
rev.Initialize(ground, pend, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))

# Create the Irrlicht application
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Limits on LinkLockRevolute demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_chrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-2, 1.5, 5))
vis.AddTypicalLights()

# Points for drawing line segments
p0 = chrono.ChVector3d(0, 0, 0)
p1 = chrono.ChVector3d(m.cos(min_angle), -m.sin(min_angle), 0)
p2 = chrono.ChVector3d(m.cos(max_angle), -m.sin(max_angle), 0)

# Simulation loop
while vis.Run():
    vis.BeginScene() 
    vis.Render()
    irr.drawSegment(vis, p0, p0 + p1 * 4);
    irr.drawSegment(vis, p0, p0 + p2 * 4);
    vis.EndScene()
    sys.DoStepDynamics(1e-3)
