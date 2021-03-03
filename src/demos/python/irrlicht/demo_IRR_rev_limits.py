# =============================================================================
# PROJECT CHRONO - http:#projectchrono.org
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

system = chrono.ChSystemNSC()

# Create ground body
ground = chrono.ChBody()
system.AddBody(ground)
ground.SetIdentifier(-1)
ground.SetBodyFixed(True)
ground.SetCollide(False)

# Visualization for revolute joint
cyl_rev = chrono.ChCylinderShape()
cyl_rev.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0.2)
cyl_rev.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, -0.2)
cyl_rev.GetCylinderGeometry().rad = 0.04
ground.AddAsset(cyl_rev)

# Create a pendulum body
pend = chrono.ChBody()
system.AddBody(pend)
pend.SetIdentifier(1)
pend.SetBodyFixed(False)
pend.SetCollide(False)
pend.SetMass(1)
pend.SetInertiaXX(chrono.ChVectorD(0.2, 1, 1))

# Initial position of the pendulum (horizontal, pointing towards positive X).
pend.SetPos(chrono.ChVectorD(1.5, 0, 0))

# Attach visualization assets.
cyl_p = chrono.ChCylinderShape()
cyl_p.GetCylinderGeometry().p1 = chrono.ChVectorD(-1.46, 0, 0)
cyl_p.GetCylinderGeometry().p2 = chrono.ChVectorD(1.46, 0, 0)
cyl_p.GetCylinderGeometry().rad = 0.2
pend.AddAsset(cyl_p)

col_p = chrono.ChColorAsset()
col_p.SetColor(chrono.ChColor(0.6, 0, 0))
pend.AddAsset(col_p)

# Create a revolute joint to connect pendulum to ground
rev = chrono.ChLinkLockRevolute()
system.AddLink(rev)

# Add limits to the Z rotation of the revolute joint
min_angle = 0
max_angle = 0.75 * m.pi
rev.GetLimit_Rz().SetActive(True)
rev.GetLimit_Rz().SetMin(min_angle)
rev.GetLimit_Rz().SetMax(max_angle)

# Initialize the joint specifying a coordinate system (expressed in the absolute frame).
rev.Initialize(ground, pend, chrono.ChCoordsysD(chrono.VNULL, chrono.QUNIT))

# Create the Irrlicht application
application = irr.ChIrrApp(system, "Limits on LinkLockRevolute demo", irr.dimension2du(800, 600))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(irr.vector3df(-2, 1.5, 5))
application.AssetBindAll()
application.AssetUpdateAll()

# Points for drawing line segments
p0 = chrono.ChVectorD(0, 0, 0)
p1 = chrono.ChVectorD(m.cos(min_angle), -m.sin(min_angle), 0)
p2 = chrono.ChVectorD(m.cos(max_angle), -m.sin(max_angle), 0)

# Simulation loop
application.SetTimestep(0.01)
application.SetTryRealtime(True)

while (application.GetDevice().run()) :
    application.BeginScene()
    application.DrawAll()
    irr.ChIrrTools.drawSegment(application.GetVideoDriver(), p0, p0 + p1 * 4, irr.SColor(255, 255, 150, 0), True);
    irr.ChIrrTools.drawSegment(application.GetVideoDriver(), p0, p0 + p2 * 4, irr.SColor(255, 255, 150, 0), True);
    application.DoStep()
    application.EndScene()