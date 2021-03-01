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
# Simple example demonstrating the use of ChLinkRotSpringCB.
#
# Recall that Irrlicht uses a left-hand frame, so everything is rendered with
# left and right flipped.
#
# =============================================================================

import pychrono as chrono
import pychrono.irrlicht as irr
import math as m

# =============================================================================

spring_coef = 40
damping_coef = 2
rest_angle = m.pi / 6

# =============================================================================

# Functor class implementing the torque for a ChLinkRotSpringCB link.
class MySpringTorque(chrono.TorqueFunctor):
    def __init__(self):
        super(MySpringTorque, self).__init__()
    
    def __call__(self,    #
                 time,    # current time
                 angle,   # relative angle of rotation
                 vel,     # relative angular speed
                 link):   # back-pointer to associated link
                              
        torque = -spring_coef * (angle - rest_angle) - damping_coef * vel
        return torque

# =============================================================================

print("Copyright (c) 2017 projectchrono.org")

system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))

# Revolute joint frame 
rev_rot = chrono.Q_from_AngX(m.pi / 6.0)
rev_dir = rev_rot.GetZaxis()
rev_pos = chrono.ChVectorD(+1, 0, 0)

# Create ground body
ground = chrono.ChBody()
system.AddBody(ground)
ground.SetIdentifier(-1)
ground.SetBodyFixed(True)
ground.SetCollide(False)

# Visualization for revolute joint
cyl_rev = chrono.ChCylinderShape()
cyl_rev.GetCylinderGeometry().p1 = rev_pos + rev_dir * 0.2
cyl_rev.GetCylinderGeometry().p2 = rev_pos - rev_dir * 0.2
cyl_rev.GetCylinderGeometry().rad = 0.1
ground.AddAsset(cyl_rev)

# Offset from joint to body COM
offset = chrono.ChVectorD(1.5, 0, 0)

# Consistent initial velocities
omega = 5.0
ang_vel = rev_dir * omega
lin_vel = ang_vel % offset

# Create pendulum body
body = chrono.ChBody()
system.AddBody(body)
body.SetPos(rev_pos + offset)
body.SetPos_dt(lin_vel)
body.SetWvel_par(ang_vel)
body.SetIdentifier(1)
body.SetBodyFixed(False)
body.SetCollide(False)
body.SetMass(1)
body.SetInertiaXX(chrono.ChVectorD(1, 1, 1))

# Attach visualization assets
sph = chrono.ChSphereShape()
sph.GetSphereGeometry().rad = 0.3
body.AddAsset(sph)
cyl = chrono.ChCylinderShape()
cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(-1.5, 0, 0)
cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, 0)
cyl.GetCylinderGeometry().rad = 0.1
body.AddAsset(cyl)
col = chrono.ChColorAsset()
col.SetColor(chrono.ChColor(0.7, 0.8, 0.8))
body.AddAsset(col)

# Create revolute joint between body and ground
rev = chrono.ChLinkLockRevolute()
rev.Initialize(body, ground, chrono.ChCoordsysD(rev_pos, rev_rot))
system.AddLink(rev)

# Create the rotational spring between body and ground
torque = MySpringTorque()
spring = chrono.ChLinkRotSpringCB()
spring.Initialize(body, ground, chrono.ChCoordsysD(rev_pos, rev_rot))
spring.RegisterTorqueFunctor(torque)
system.AddLink(spring);

# Create the Irrlicht application
application = irr.ChIrrApp(system, "ChLinkRotSpringCB demo", irr.dimension2du(800, 600))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(irr.vector3df(3, 1, 3))
application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop
application.SetTimestep(0.001)

frame = 0
while (application.GetDevice().run()) :
    application.BeginScene()
    application.DrawAll()
    irr.ChIrrTools.drawAllCOGs(system, application.GetVideoDriver(), 1.0)
    irr.ChIrrTools.drawAllLinkframes(system, application.GetVideoDriver(), 1.5)
    application.DoStep()

    if (frame % 50 == 0) :
        print('{:.6}'.format(str(system.GetChTime())))
        print('Body position      ', body.GetPos())
        print('Body lin. vel      ', body.GetPos_dt())
        print('Body abs. ang. vel ', body.GetWvel_par())
        print('Body loc. ang. vel ', body.GetWvel_loc())
        print('Rot. spring-damper ', spring.GetRotSpringAngle(), '  ', spring.GetRotSpringTorque())
        print('---------------')


    frame += 1

    application.EndScene()