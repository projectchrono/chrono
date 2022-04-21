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
# Simple example demonstrating the use of ChLinkRSDA.
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

# Functor class implementing the torque for a ChLinkRSDA link.
class MySpringTorque(chrono.TorqueFunctor):
    def __init__(self):
        super(MySpringTorque, self).__init__()
    
    def evaluate(self,    #
                 time,    # current time
                 angle,   # relative angle of rotation
                 vel,     # relative angular speed
                 link):   # associated link
                              
        torque = -spring_coef * (angle - rest_angle) - damping_coef * vel
        return torque

# =============================================================================

print("Copyright (c) 2017 projectchrono.org")

sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, 0, 0))

# Revolute joint frame 
rev_rot = chrono.Q_from_AngX(m.pi / 6.0)
rev_dir = rev_rot.GetZaxis()
rev_pos = chrono.ChVectorD(+1, 0, 0)

# Create ground body
ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetIdentifier(-1)
ground.SetBodyFixed(True)
ground.SetCollide(False)

# Visualization for revolute joint
cyl_rev = chrono.ChCylinderShape()
cyl_rev.GetCylinderGeometry().p1 = rev_pos + rev_dir * 0.2
cyl_rev.GetCylinderGeometry().p2 = rev_pos - rev_dir * 0.2
cyl_rev.GetCylinderGeometry().rad = 0.1
ground.AddVisualShape(cyl_rev)

# Offset from joint to body COM
offset = chrono.ChVectorD(1.5, 0, 0)

# Consistent initial velocities
omega = 5.0
ang_vel = rev_dir * omega
lin_vel = ang_vel % offset

# Create pendulum body
body = chrono.ChBody()
sys.AddBody(body)
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
body.AddVisualShape(sph)
cyl = chrono.ChCylinderShape()
cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(-1.5, 0, 0)
cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, 0)
cyl.GetCylinderGeometry().rad = 0.1
cyl.SetColor(chrono.ChColor(0.7, 0.8, 0.8))
body.AddVisualShape(cyl)

# Create revolute joint between body and ground
rev = chrono.ChLinkLockRevolute()
rev.Initialize(body, ground, chrono.ChCoordsysD(rev_pos, rev_rot))
sys.AddLink(rev)

# Create the rotational spring between body and ground
torque = MySpringTorque()
spring = chrono.ChLinkRSDA()
spring.Initialize(body, ground, chrono.ChCoordsysD(rev_pos, rev_rot))
spring.RegisterTorqueFunctor(torque)
rsda = chrono.ChRotSpringShape(0.5, 40)
rsda.SetColor(chrono.ChColor(0, 0, 0))
spring.AddVisualShape(rsda)
sys.AddLink(spring)

# Create the Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
sys.SetVisualSystem(vis)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('ChLinkRSDA demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 1, 3))
vis.AddTypicalLights()

# Simulation loop
frame = 0
while vis.Run():
    vis.BeginScene() 
    vis.DrawAll()
    irr.drawAllCOGs(sys, vis.GetVideoDriver(), 1.0)
    irr.drawAllLinkframes(sys, vis.GetVideoDriver(), 1.5)
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

    if (frame % 50 == 0) :
        print('{:.6}'.format(str(sys.GetChTime())))
        print('Body position      ', body.GetPos())
        print('Body lin. vel      ', body.GetPos_dt())
        print('Body abs. ang. vel ', body.GetWvel_par())
        print('Body loc. ang. vel ', body.GetWvel_loc())
        print('Rot. spring-damper ', spring.GetAngle(), '  ', spring.GetTorque())
        print('---------------')


    frame += 1
