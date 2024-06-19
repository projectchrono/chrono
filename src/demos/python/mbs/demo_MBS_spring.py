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
# Authors: Simone Benatti
# =============================================================================
#
# Simple example demonstrating the use of ChLinkTSDA.
#
# Two bodies, connected with identical (but modeled differently) spring-dampers
# are created side by side.
#
# Recall that Irrlicht uses a left-hand frame, so everything is rendered with
# left and right flipped.
#
# =============================================================================

import pychrono as chrono
import pychrono.irrlicht as chronoirr

# =============================================================================

rest_length = 1.5
spring_coef = 50
damping_coef = 1

# =============================================================================

# Functor class implementing the force for a ChLinkTSDA link.
# In this simple demonstration, we just reimplement the default linear spring-damper.
class MySpringForce(chrono.ForceFunctor):
    def __init__(self):
        super(MySpringForce, self).__init__()
    
    def evaluate(self,         #
                 time,         # current time
                 rest_length,  # undeformed length
                 length,       # current length
                 vel,          # current velocity (positive when extending)
                 link):        # associated link
        force = -spring_coef * (length - rest_length) - damping_coef * vel
        return force

# =============================================================================

print("Copyright (c) 2017 projectchrono.org")

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Create the ground body with two visualization spheres
# -----------------------------------------------------

ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))

sph_2 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))

# Create a body suspended through a ChLinkTSDA (default linear)
# -------------------------------------------------------------

body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Attach a visualization asset.
box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# Create the spring between body_1 and ground. The spring end points are
# specified in the body relative frames.
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)

# Attach a visualization asset.
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Create a body suspended through a ChLinkTSDA (custom force functor)
# -------------------------------------------------------------------

body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(1, -3, 0))
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Attach a visualization asset.
box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0, 0.6))
body_2.AddVisualShape(box_2)

# Create the spring between body_2 and ground. The spring end points are
# specified in the body relative frames.
force = MySpringForce()

spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0))
spring_2.SetRestLength(rest_length)
spring_2.RegisterForceFunctor(force)
sys.AddLink(spring_2)

# Attach a visualization asset.
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

# Create the Irrlicht application
# -------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('ChLinkTSDA demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 6))
vis.AddTypicalLights()

# Simulation loop
frame = 0

while vis.Run() :
    vis.BeginScene() 
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

    if (frame % 50 == 0) :
        print( '{:.6}'.format(str(sys.GetChTime())) + " \n" + '{:.6}'.format(str(spring_1.GetLength())) + 
                 "  " + '{:.6}'.format(str(spring_1.GetVelocity())) + "  "
                 + '{:.6}'.format(str(spring_1.GetForce())))

        print('{:.6}'.format(str(spring_2.GetLength())) + "  " +
                  '{:.6}'.format(str(spring_2.GetVelocity())) + "  " + '{:.6}'.format(str(spring_2.GetForce())) )
    

    frame += 1
