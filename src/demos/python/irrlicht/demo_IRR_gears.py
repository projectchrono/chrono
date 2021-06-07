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
# Authors: Han Wang
# =============================================================================
#
# Demonstration of the gear constraint (ChLinkGear) as a method to impose a
# transmission ratio between two shafts as they were connected by gears,
# without the need of performing collision detection between gear teeth
# geometries (which would be inefficient)
#
# =============================================================================

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m

print("Copyright (c) 2017 projectchrono.org")

# Create a Chrono::Engine physical system
mphysicalSystem = chrono.ChSystemNSC()

# Create the Irrlicht visualization (open the Irrlicht device,
# bind a simple user interface, etc, etc.)
application = chronoirr.ChIrrApp(mphysicalSystem, "Gears annd pulleys", chronoirr.dimension2du(800, 600))


# Easy shortcuts to add camera, lights, logo, and sky in Irrlicht scene:
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(12, 15, -20))

# Contact material shared among all bodies
mat = chrono.ChMaterialSurfaceNSC()

# Create all rigid bodies.

radA = 2
radB = 4

# ...the truss
mbody_truss = chrono.ChBodyEasyBox(20, 10, 2, 1000, True, False, mat)
mphysicalSystem.Add(mbody_truss)
mbody_truss.SetBodyFixed(True)
mbody_truss.SetPos(chrono.ChVectorD(0, 0, 3))

# ...a texture asset that will be shared among the four wheels
cylinder_texture = chrono.ChTexture(chrono.GetChronoDataFile("textures/pinkwhite.png"))

# ...the rotating bar support for the two epicycloidal wheels
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
mphysicalSystem.Add(mbody_train)
mbody_train.SetPos(chrono.ChVectorD(3, 0, 0))

# ...which must rotate respect to truss along Z axis, in 0,0,0
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train, 
                           chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
mphysicalSystem.AddLink(link_revoluteTT)

# ...the first gear
mbody_gearA = chrono.ChBodyEasyCylinder(radA, 0.5, 1000, True, False, mat)
mphysicalSystem.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVectorD(0, 0, -1))
mbody_gearA.SetRot(chrono.Q_from_AngX(m.pi / 2))
mbody_gearA.AddAsset(cylinder_texture)

# for aesthetic reasons, also add a thin cylinder only as a visualization
mshaft_shape = chrono.ChCylinderShape()
mshaft_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, -3, 0)
mshaft_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 10, 0)
mshaft_shape.GetCylinderGeometry().rad = radA * 0.4
mbody_gearA.AddAsset(mshaft_shape)

# ...impose rotation speed between the first gear and the fixed truss
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss, 
                        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunction_Const(6))
mphysicalSystem.AddLink(link_motor)


# ...the second gear
interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(radB, 0.4, 1000, True, False, mat)
mphysicalSystem.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVectorD(interaxis12, 0, -1))
mbody_gearB.SetRot(chrono.Q_from_AngX(m.pi / 2))
mbody_gearB.AddAsset(cylinder_texture)

# ...the second gear is fixed to the rotating bar
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train, 
                        chrono.ChCoordsysD(chrono.ChVectorD(interaxis12, 0, 0), chrono.QUNIT))
mphysicalSystem.AddLink(link_revolute)

# ...the gear constraint between the two wheels A and B.
#    As transmission ratio (=speed of wheel B / speed of wheel A) to enter in  Set_tau(), we
#    could use whatever positive value we want: the ChLinkGear will compute the two radii of the
#    wheels for its 'hidden' computations, given the distance between the two axes. However, since
#    we already build two '3D cylinders' bodies -just for visualization reasons!- with radA and radB,
#    we must enter Set_tau(radA/radB).
#    Also, note that the initial position of the constraint has no importance (simply use CSYSNORM),
#    but we must set where the two axes are placed in the local coordinates of the two wheels, so
#    we use Set_local_shaft1() and pass some local ChFrame. Note that, since the Z axis of that frame
#    will be considered the axis of the wheel, we must rotate the frame 90� with Q_from_AngAxis(), because
#    we created the wheel with ChBodyEasyCylinder() which created a cylinder with Y as axis
link_gearAB = chrono.ChLinkGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.CSYSNORM)
link_gearAB.Set_local_shaft1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))
link_gearAB.Set_local_shaft2(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))
link_gearAB.Set_tau(radA / radB)
link_gearAB.Set_checkphase(True)
mphysicalSystem.AddLink(link_gearAB)

# ...the gear constraint between the second wheel B and a large wheel C with inner teeth, that
#    does not necessarily need to be created as a new body because it is the 'fixed' part of the
#    epicycloidal reducer, so, as wheel C, we will simply use the ground object 'mbody_truss'.
radC = 2 * radB + radA
link_gearBC = chrono.ChLinkGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.CSYSNORM)
link_gearBC.Set_local_shaft1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))
link_gearBC.Set_local_shaft2(chrono.ChFrameD(chrono.ChVectorD(0, 0, -4), chrono.QUNIT))
link_gearBC.Set_tau(radB / radC)
link_gearBC.Set_epicyclic(True) # <-- this means: use a wheel with internal teeth!
mphysicalSystem.AddLink(link_gearBC)

# ...the bevel gear at the side,
radD = 5
mbody_gearD = chrono.ChBodyEasyCylinder(radD, 0.8, 1000, True, False, mat)
mphysicalSystem.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVectorD(-10, 0, -9))
mbody_gearD.SetRot(chrono.Q_from_AngZ(m.pi / 2))
mbody_gearD.AddAsset(cylinder_texture)

# ...it is fixed to the truss using a revolute joint with horizontal axis (must rotate
#    the default ChLink creation coordys 90 degrees on the Y vertical, since the revolute
#    axis is the Z axis)
link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_gearD, mbody_truss, 
                          chrono.ChCoordsysD(chrono.ChVectorD(-10, 0, -9), 
                          chrono.Q_from_AngY(m.pi / 2)))
mphysicalSystem.AddLink(link_revoluteD)

# ... Let's make a 1:1 gear between wheel A and wheel D as a bevel gear: Chrono does not require
#     special info for this case -the position of the two shafts and the transmission ratio are enough-
link_gearAD = chrono.ChLinkGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.CSYSNORM)
link_gearAD.Set_local_shaft1(chrono.ChFrameD(chrono.ChVectorD(0, -7, 0), chrono.Q_from_AngX(-m.pi / 2)))
link_gearAD.Set_local_shaft2(chrono.ChFrameD(chrono.ChVectorD(0, -7, 0), chrono.Q_from_AngX(-m.pi / 2)))
link_gearAD.Set_tau(1)
mphysicalSystem.AddLink(link_gearAD)

# ...the pulley at the side,
radE = 2;
mbody_pulleyE = chrono.ChBodyEasyCylinder(radE, 0.8, 1000, True, False, mat)
mphysicalSystem.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVectorD(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.Q_from_AngZ(m.pi / 2))
mbody_pulleyE.AddAsset(cylinder_texture)

# ... it is fixed to the truss using a revolute joint with horizontal axis (must rotate
#     default ChLink creation coordys 90� on the Y vertical, since the revolute axis is the Z axis).
link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss,
                          chrono.ChCoordsysD(chrono.ChVectorD(-10, -11, -9), chrono.Q_from_AngY(m.pi / 2)))
mphysicalSystem.AddLink(link_revoluteE)

# ... Let's make a synchro belt constraint between pulley D and pulley E. The user must be
#     sure that the two shafts are parallel in absolute space. Also, interaxial distance should not change.
link_pulleyDE = chrono.ChLinkPulley()
link_pulleyDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.CSYSNORM)
link_pulleyDE.Set_local_shaft1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))
link_pulleyDE.Set_local_shaft2(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))
link_pulleyDE.Set_r1(radD);
link_pulleyDE.Set_r2(radE);
link_pulleyDE.Set_checkphase(True); # synchro belts don't tolerate slipping: this avoids it as numerical errors accumulate.
mphysicalSystem.AddLink(link_pulleyDE);

# Complete construction of Irrlicht assets
application.AssetBindAll();
application.AssetUpdateAll();

# Set intergator type
mphysicalSystem.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# Simulation loop
application.SetTimestep(0.001)
application.SetTryRealtime(True)

while (application.GetDevice().run()):
    application.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192));

    application.DrawAll();

    # Draw some segments for a simplified representation of pulley
    chronoirr.drawSegment(application.GetVideoDriver(),
                          link_pulleyDE.Get_belt_up1(),
                          link_pulleyDE.Get_belt_up2(),
                          chronoirr.SColor(255, 0, 255, 0), True);
    chronoirr.drawSegment(application.GetVideoDriver(),
                          link_pulleyDE.Get_belt_low1(),
                          link_pulleyDE.Get_belt_low2(),
                          chronoirr.SColor(255, 0, 255, 0), True);

    # Advance simulation for one step
    application.DoStep();

    application.EndScene();

