# =============================================================================
# PROJECT CHRONO - http:#projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http:#projectchrono.org/license-chrono.txt.
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
    
    def __call__(             self,         #
                              time,         # current time
                              rest_length,  # undeformed length
                              length,       # current length
                              vel,          # current velocity (positive when extending)
                              link):         # back-pointer to associated link
                              
        force = -spring_coef * (length - rest_length) - damping_coef * vel
        return force
    

# =============================================================================

print("Copyright (c) 2017 projectchrono.org")

system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))

# Create the ground body with two visualization spheres
# -----------------------------------------------------

ground = chrono.ChBody()
system.AddBody(ground)
ground.SetIdentifier(-1)
ground.SetBodyFixed(True)
ground.SetCollide(False)
sph_1 = chrono.ChSphereShape()
sph_1.GetSphereGeometry().rad = 0.1
sph_1.Pos = chrono.ChVectorD(-1, 0, 0)
ground.AddAsset(sph_1)

sph_2 = chrono.ChSphereShape()
sph_2.GetSphereGeometry().rad = 0.1
sph_2.Pos = chrono.ChVectorD(1, 0, 0)
ground.AddAsset(sph_2)

# Create a body suspended through a ChLinkTSDA (default linear)
# -------------------------------------------------------------

body_1 = chrono.ChBody()
system.AddBody(body_1)
body_1.SetPos(chrono.ChVectorD(-1, -3, 0))
body_1.SetIdentifier(1)
body_1.SetBodyFixed(False)
body_1.SetCollide(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVectorD(1, 1, 1))

# Attach a visualization asset.
box_1 = chrono.ChBoxShape()
box_1.GetBoxGeometry().SetLengths(chrono.ChVectorD(1, 1, 1))
body_1.AddAsset(box_1)
col_1 = chrono.ChColorAsset()
col_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddAsset(col_1)

# Create the spring between body_1 and ground. The spring end points are
# specified in the body relative frames.
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(-1, 0, 0), False, rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
system.AddLink(spring_1)

# Attach a visualization asset.
spring_1.AddAsset(col_1)
spring_1.AddAsset(chrono.ChPointPointSpring(0.05, 80, 15))

# Create a body suspended through a ChLinkTSDA (custom force functor)
# -------------------------------------------------------------------

body_2 = chrono.ChBody()
system.AddBody(body_2)
body_2.SetPos(chrono.ChVectorD(1, -3, 0))
body_2.SetIdentifier(1)
body_2.SetBodyFixed(False)
body_2.SetCollide(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVectorD(1, 1, 1))

# Attach a visualization asset.
box_2 = chrono.ChBoxShape()
box_2.GetBoxGeometry().SetLengths(chrono.ChVectorD(1, 1, 1))
body_2.AddAsset(box_1)
col_2 = chrono.ChColorAsset()
col_2.SetColor(chrono.ChColor(0, 0, 0.6))
body_2.AddAsset(col_2)

# Create the spring between body_2 and ground. The spring end points are
# specified in the body relative frames.
force = MySpringForce()

spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(1, 0, 0), False, rest_length)
spring_2.RegisterForceFunctor(force)
system.AddLink(spring_2)

# Attach a visualization asset.
spring_2.AddAsset(col_2)
spring_2.AddAsset(chrono.ChPointPointSpring(0.05, 80, 15))

# Create the Irrlicht application
# -------------------------------

application = chronoirr.ChIrrApp(system, "ChLinkTSDA demo", chronoirr.dimension2du(800, 600), False, True)
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0, 0, 6))

application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop
frame = 0

application.SetTimestep(0.001)


while (application.GetDevice().run()) :
    application.BeginScene()

    application.DrawAll()

    application.DoStep()

    if (frame % 50 == 0) :
        print( '{:.6}'.format(str(system.GetChTime())) + " \n" + '{:.6}'.format(str(spring_1.GetLength())) + 
                 "  " + '{:.6}'.format(str(spring_1.GetVelocity())) + "  "
                 + '{:.6}'.format(str(spring_1.GetForce())))

        print('{:.6}'.format(str(spring_2.GetLength())) + "  " +
                  '{:.6}'.format(str(spring_2.GetVelocity())) + "  " + '{:.6}'.format(str(spring_2.GetForce())) )
    

    frame += 1

    application.EndScene()