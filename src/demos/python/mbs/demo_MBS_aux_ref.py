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
# To demonstrate the use of ChBodyAuxRef, this simple example constructs two
# identical pendulums, one modeled as a ChBody, the other as a ChBodyAuxRef.
# The sys is modeled in a (right-hand) frame with Y up.
#
# The two pendulums have length 2 and are pinned to ground through revolute
# joints with the rotation axis along the Z axis. The absolute locations of
# the revolute joints are at (0, 0, 1) and (0, 0, -1), respectively.
#
# The ChBody pendulum is defined with respect to a centroidal frame (as assumed
# by ChBody) located at the geometric center of the pendulum, with the X axis
# along the length of the pendulum.
# The ChBodyAuxRef pendulum is defined with respect to a local frame, parallel
# to its centroidal frame but located at the pin location.  In other words, the
# center of mass of the pendulum is at (1, 0, 0), relative to the body frame.
#
# The two pendulums move under the action of gravity, acting along the negative
# global Y direction.
#
# The ChBody pendulum is colored red. The ChBodyAuxRef pendulum is blue.
#
# Recall that Irrlicht uses a left-hand frame, so everything is rendered with
# left and right flipped.
# =============================================================================

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()

sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Create the ground body with two visualization cylinders

ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

cyl_1 = chrono.ChVisualShapeCylinder(0.2, 0.4)
ground.AddVisualShape(cyl_1, chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))

cyl_2 = chrono.ChVisualShapeCylinder(0.2, 0.4)
ground.AddVisualShape(cyl_2, chrono.ChFramed(chrono.ChVector3d(0, 0, -1)))


# Create a pendulum modeled using ChBody

pend_1 =  chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)
pend_1.SetMass(1)
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))

# Attach a visualization asset. Note that the cylinder is defined with
# respect to the centroidal reference frame (which is the body reference
# frame for a ChBody)
cyl_1 = chrono.ChVisualShapeCylinder(0.2, 2)
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))
pend_1.AddVisualShape(cyl_1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))


# Specify the intial position of the pendulum (horizontal, pointing towards
# positive X). In this case, we set the absolute position of its center of 
# mass
pend_1.SetPos(chrono.ChVector3d(1, 0, 1))

# Create a revolute joint to connect pendulum to ground. We specify the link
# coordinate frame in the absolute frame.
rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_1)

# Create a pendulum modeled using ChBodyAuxRef
pend_2 = chrono.ChBodyAuxRef()
sys.Add(pend_2)
pend_2.SetFixed(False)
pend_2.EnableCollision(False)
pend_2.SetMass(1)
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))
# NOTE: the inertia tensor must still be expressed in the centroidal frame!

# Attach a visualizationn asset. Note that now the cylinder is defined with
# respect to the body reference frame.
cyl_2 = chrono.ChVisualShapeCylinder(0.2, 2)
cyl_2.SetColor(chrono.ChColor(0, 0, 0.6))
pend_2.AddVisualShape(cyl_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0), chrono.QuatFromAngleY(chrono.CH_PI_2)))


# In this case, we must specify the centroidal frame, relative to the body
# reference frame
pend_2.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(1, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))

# Specify the initial position of the pendulum (horizontal, pointing towards
# positive X).  Here, we want to specify the position of the body reference
# frame (relative to the absolute frame). Recall that the body reference
# frame is located at the pin.
pend_2.SetFrameRefToAbs(chrono.ChFramed(chrono.ChVector3d(0, 0, -1)))


# Note: Beware of using the method SetPos() to specify the initial position
# (as we did for the first pendulum)!  SetPos() specifies the position of the
# centroidal frame.  So one could use it (instead of SetFrameRefToAbs):
#   pend_2->SetPos(ChVector<>(1, 0, -1));
# However, this defeats the goal of specifying the body through the desired
# body reference frame.
# Alternatively, one could use SetPos() and pass it the position of the body
# reference frame; i.e.
#   pend_2->SetPos(ChVector<>(0, 0, -1));
# provided we do this BEFORE the call to SetFrameCOMToRef().
#
# This also applies to SetRot().
#
# Create a revolute joint to connect pendulum to ground. We specify the link
# coordinate frame in the absolute frame.
rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(ground, pend_2, chrono.ChFramed(chrono.ChVector3d(0, 0, -2), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_2)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('ChBodyAuxRef demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_chrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# Simulation Loop
log_info = True

while vis.Run():
    vis.BeginScene() 
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

    if log_info and sys.GetChTime() > 1:
        # Note that GetPos() and similar functions will return information for
        # the centroidal frame for both pendulums:
        pos_1 = pend_1.GetPos()
        pos_2 = pend_2.GetPos()
        print("t = ", sys.GetChTime())
        print("     ", pos_1.x, "  ", pos_1.y)
        print("     ", pos_2.x, "  ", pos_2.y)
        frame_2 = pend_2.GetFrameRefToAbs()
        pos_2 = frame_2.GetPos()


        #OK, what about velocities? Here again, GetPosDt() returns the linear
        # velocity of the body COG (expressed in the global frame) for both
        # pendulums
        lin_vel_1 = pend_1.GetPosDt()
        lin_vel_2 = pend_2.GetPosDt()

        print("     ", lin_vel_1.x, "  ", lin_vel_1.y)
        print("     ", lin_vel_2.y, "  ", lin_vel_2.y)

        # To obtain the absolute linear velocity of the body reference frame, 
        # we use again GetPosDt(), but this time for the reference frame,
        # using GetFrameRefToAbs() similarly for what we did for positions:
        lin_vel_2 = pend_2.GetFrameRefToAbs().GetPosDt()
        print("    ", lin_vel_2.x, "  ", lin_vel_2.y)

        log_info = False
