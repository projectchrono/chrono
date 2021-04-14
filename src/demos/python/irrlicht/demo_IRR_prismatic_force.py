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
# Authors: Jay Taves (Py), Radu Serban (C++)
# =============================================================================
#
# Demonstration of actuating a translational joint with a ChLinkForce.
# The model is built with gravity acting in the negative Y direction.
#
# Recall that Irrlicht uses a left-hand frame, so everything is rendered with
# left and right flipped.
#
# =============================================================================

import pychrono as chrono
import pychrono.irrlicht as irr
import math as m

try:
    import numpy as np
    from numpy import linalg as LA
except ImportError:
    print("You need NumPy to run this demo!")

print("Copyright (c) 2017 projectchrono.org")

system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))

# Create the ground body
ground = chrono.ChBody()
system.AddBody(ground)
ground.SetIdentifier(-1)
ground.SetBodyFixed(True)
ground.SetCollide(False)

rail1 = chrono.ChBoxShape()
rail1.GetBoxGeometry().SetLengths(chrono.ChVectorD(8, 0.1, 0.1))
rail1.GetBoxGeometry().Pos = chrono.ChVectorD(0, 0, -1)
ground.AddAsset(rail1)

rail2 = chrono.ChBoxShape()
rail2.GetBoxGeometry().SetLengths(chrono.ChVectorD(8, 0.1, 0.1))
rail2.GetBoxGeometry().Pos = chrono.ChVectorD(0, 0, 1)
ground.AddAsset(rail2)

col = chrono.ChColorAsset()
col.SetColor(chrono.ChColor(0.6, 0.6, 0.6))
ground.AddAsset(col)

# Create the slider bodies
slider1 = chrono.ChBody()
system.AddBody(slider1)
slider1.SetIdentifier(1)
slider1.SetBodyFixed(False)
slider1.SetCollide(False)
slider1.SetMass(1)
slider1.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
slider1.SetPos(chrono.ChVectorD(-4, 0, -1))

cyl1 = chrono.ChCylinderShape()
cyl1.GetCylinderGeometry().p1 = chrono.ChVectorD(-0.2, 0, 0)
cyl1.GetCylinderGeometry().p2 = chrono.ChVectorD(0.2, 0, 0)
cyl1.GetCylinderGeometry().rad = 0.2
slider1.AddAsset(cyl1)

col1 = chrono.ChColorAsset()
col1.SetColor(chrono.ChColor(0.6, 0, 0))
slider1.AddAsset(col1)

slider2 = chrono.ChBody()
system.AddBody(slider2)
slider2.SetIdentifier(1)
slider2.SetBodyFixed(False)
slider2.SetCollide(False)
slider2.SetMass(1)
slider2.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
slider2.SetPos(chrono.ChVectorD(-4, 0, 1))

cyl2 = chrono.ChCylinderShape()
cyl2.GetCylinderGeometry().p1 = chrono.ChVectorD(-0.2, 0, 0)
cyl2.GetCylinderGeometry().p2 = chrono.ChVectorD(0.2, 0, 0)
cyl2.GetCylinderGeometry().rad = 0.2
slider2.AddAsset(cyl2)

col2 = chrono.ChColorAsset()
col2.SetColor(chrono.ChColor(0, 0, 0.6))
slider2.AddAsset(col2)

# Create prismatic joints between ground a sliders
prismatic1 = chrono.ChLinkLockPrismatic()
prismatic1.Initialize(slider1, ground, chrono.ChCoordsysD(
    chrono.ChVectorD(0, 0, -1), chrono.Q_from_AngY(chrono.CH_C_PI_2)))
system.AddLink(prismatic1)

prismatic2 = chrono.ChLinkLockPrismatic()
prismatic2.Initialize(slider2, ground, chrono.ChCoordsysD(
    chrono.ChVectorD(0, 0, 1), chrono.Q_from_AngY(chrono.CH_C_PI_2)))
system.AddLink(prismatic2)

# Sine function parameters
freq = 1
ampl = 4
omg = 2 * chrono.CH_C_PI * freq
mod = chrono.ChFunction_Sine(0, freq, ampl)

# Actuate first slider using a link force
prismatic1.GetForce_Z().SetActive(True)
prismatic1.GetForce_Z().SetF(1)
prismatic1.GetForce_Z().SetModulationF(mod)

# Actuate second slider using a body force
frc2 = chrono.ChForce()
frc2.SetF_x(mod)
slider2.AddForce(frc2)

# Create the Irrlicht application
application = irr.ChIrrApp(system, "Actuated prismatic joint", irr.dimension2du(800, 600))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(irr.vector3df(-1, 1.5, -6))

application.AssetBindAll()
application.AssetUpdateAll()

application.SetTimestep(1e-3)

x0 = slider1.GetPos().x

while application.GetDevice().run():
    time = system.GetChTime()

    # Output slider x position/velocity and analytical solution
    # x = slider1.GetPos().x
    # x_d = slider1.GetPos_dt().x
    # xa = x0 + (ampl / omg) * (time - m.sin(omg * time) / omg)
    # xa_d = (ampl / omg) * (1 - m.cos(omg * time))
    # print('{0:f}   {1:f} {2:f}   {3:f} {4:f}'.format(time, x, x_d, xa, xa_d))

    application.BeginScene()
    application.DrawAll()
    irr.drawAllLinkframes(system, application.GetVideoDriver(), 1)
    application.DoStep()
    application.EndScene()
