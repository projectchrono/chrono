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
                 rangle,  # rest angle
                 angle,   # relative angle of rotation
                 vel,     # relative angular speed
                 link):   # associated link                       
        torque = -spring_coef * (angle - rest_angle) - damping_coef * vel
        return torque

# =============================================================================

print("Copyright (c) 2017 projectchrono.org")

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Revolute joint frame 
rev_rot = chrono.QuatFromAngleX(m.pi / 6.0)
rev_dir = rev_rot.GetAxisZ()
rev_pos = chrono.ChVector3d(+1, 0, 0)

# Create ground body
ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Visualization for revolute joint
seg = chrono.ChLineSegment(rev_pos + rev_dir * 0.2, rev_pos - rev_dir * 0.2)
cyl_rev = chrono.ChVisualShapeCylinder(0.1, seg.GetLength())
ground.AddVisualShape(cyl_rev, seg.GetFrame())

# Offset from joint to body COM
offset = chrono.ChVector3d(1.5, 0, 0)

# Consistent initial velocities
omega = 5.0
ang_vel = rev_dir * omega
lin_vel = ang_vel % offset

# Create pendulum body
body = chrono.ChBody()
sys.AddBody(body)
body.SetPos(rev_pos + offset)
body.SetPosDt(lin_vel)
body.SetAngVelParent(ang_vel)
body.SetFixed(False)
body.EnableCollision(False)
body.SetMass(1)
body.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Attach visualization assets
sph = chrono.ChVisualShapeSphere(0.3)
body.AddVisualShape(sph)
cyl = chrono.ChVisualShapeCylinder(0.1, 1.5)
cyl.SetColor(chrono.ChColor(0.7, 0.8, 0.8))
body.AddVisualShape(cyl, chrono.ChFramed(chrono.ChVector3d(-0.75,0,0), chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Create revolute joint between body and ground
rev = chrono.ChLinkLockRevolute()
rev.Initialize(body, ground, chrono.ChFramed(rev_pos, rev_rot))
sys.AddLink(rev)

# Create the rotational spring between body and ground
torque = MySpringTorque()
spring = chrono.ChLinkRSDA()
spring.Initialize(body, ground, chrono.ChFramed(rev_pos, rev_rot))
spring.RegisterTorqueFunctor(torque)
rsda = chrono.ChVisualShapeRotSpring(0.5, 40)
rsda.SetColor(chrono.ChColor(0, 0, 0))
spring.AddVisualShape(rsda)
sys.AddLink(spring)

# Create the Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('ChLinkRSDA demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 1, 3))
vis.AddTypicalLights()

# Simulation loop
frame = 0
while vis.Run():
    vis.BeginScene() 
    vis.Render()
    irr.drawAllCOGs(vis, 1.0)
    irr.drawAllLinkframes(vis, 1.5)
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

    if (frame % 50 == 0) :
        print('{:.6}'.format(str(sys.GetChTime())))
        print('Body position      ', body.GetPos())
        print('Body lin. vel      ', body.GetPosDt())
        print('Body abs. ang. vel ', body.GetAngVelParent())
        print('Body loc. ang. vel ', body.GetAngVelLocal())
        print('Rot. spring-damper ', spring.GetAngle(), '  ', spring.GetTorque())
        print('---------------')


    frame += 1
