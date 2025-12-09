# =============================================================================
# PROJECT CHRONO - http:#projectchrono.org
#
# Copyright (c) 2025 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http:#projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Radu Serban
# =============================================================================
#
# Python demo for 2D collision shapes (Generva wheel)
#
# =============================================================================

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# ------------------------------------------------------------------------------

# Set output root directory
chrono.SetChronoOutputPath("../DEMO_OUTPUT/")

# ------------------------------------------------------------------------------

# Create system
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create a (shared) contact contact_material
contact_mat = chrono.ChContactMaterialNSC()

# Create the ground body
ground = chrono.ChBody()
ground.SetFixed(True)
sys.Add(ground)

# ------------------------------------------------------------------------------

# Geneva wheel geometry data
num_stations = 5
R = 1
Ri = 0.5
wi = 0.1
Li = 0.8
geneva_center = chrono.ChVector3d(0, 0, 0)

beta = 2 * math.pi / num_stations  # angle width of station
gamma = 2 * (0.5 * math.pi - beta / 2)
B = R * math.tan(beta / 2)
crank_center = chrono.ChVector3d(B, R, 0) + geneva_center

# Create the rotating Geneva wheel body
geneva_wheel = chrono.ChBody()
geneva_wheel.SetPos(geneva_center)
geneva_wheel.SetAngVelLocal(chrono.ChVector3d(0, 0, -0.08))
sys.Add(geneva_wheel)

# Create a ChLinePath geometry that represents the 2D shape of the Geneva wheel.
# It can be made of ChLineArc or ChLineSegment sub-lines. These must be added in clockwise
# order, and the end of sub-item i must be coincident with beginning of sub-line i+1.
wheel_path = chrono.ChLinePath()

for i in range(0, num_stations):
    alpha = -i * beta  # phase of current station
    p1 = chrono.ChVector3d(-B + Ri, R, 0)
    p2 = chrono.ChVector3d(-wi / 2, R, 0)
    p3 = chrono.ChVector3d(-wi / 2, R - Li, 0)
    p4 = chrono.ChVector3d(wi / 2, R - Li, 0)
    p5 = chrono.ChVector3d(wi / 2, R, 0)
    p6 = chrono.ChVector3d(B - Ri, R, 0)
    p7 = chrono.ChVector3d(B, R, 0)
    mm = chrono.ChMatrix33d(alpha, chrono.VECT_Z)
    p1 = mm * p1
    p2 = mm * p2
    p3 = mm * p3
    p4 = mm * p4
    p5 = mm * p5
    p6 = mm * p6
    p7 = mm * p7
    seg1 = chrono.ChLineSegment(p1, p2)
    seg2 = chrono.ChLineSegment(p2, p3)
    seg3 = chrono.ChLineSegment(p3, p4)
    seg4 = chrono.ChLineSegment(p4, p5)
    seg5 = chrono.ChLineSegment(p5, p6)
    wheel_path.AddSubLine(seg1)
    wheel_path.AddSubLine(seg2)
    wheel_path.AddSubLine(seg3)
    wheel_path.AddSubLine(seg4)
    wheel_path.AddSubLine(seg5)
    a1 = alpha + math.pi
    a2 = alpha + math.pi + gamma
    arc0 = chrono.ChLineArc(chrono.ChCoordsysd(p7), Ri, a1, a2, True)  # ccw arc because concave
    wheel_path.AddSubLine(arc0)

# Add the collision shape to the body
geneva_wheel.EnableCollision(True)
genevawheel_coll = chrono.ChCollisionShapePath2D(contact_mat, wheel_path)
geneva_wheel.AddCollisionShape(genevawheel_coll, chrono.ChFramed())
geneva_wheel.GetCollisionModel().SetSafeMargin(0.02)

# Create a ChVisualShapeLine, a visualization asset for lines.
wheel_asset = chrono.ChVisualShapeLine()
wheel_asset.SetLineGeometry(wheel_path)
geneva_wheel.AddVisualShape(wheel_asset)

# ------------------------------------------------------------------------------

# Revolute constraint
revolute = chrono.ChLinkLockRevolute()
revolute.Initialize(geneva_wheel, ground, chrono.ChFramed(geneva_center))
sys.Add(revolute)

# Create the crank
crank = chrono.ChBody()
crank.SetPos(crank_center)
sys.Add(crank)

# Create a ChLinePath geometry, and insert sub-paths in clockwise order
crankpin_path = chrono.ChLinePath()
pin = chrono.ChLineArc(chrono.ChCoordsysd(chrono.ChVector3d(-B, 0, 0)), wi / 2 - 0.005, 2 * math.pi, 0)
crankpin_path.AddSubLine(pin)

crankstopper_path = chrono.ChLinePath()
stopper_arc = chrono.ChLineArc(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0)), Ri - 0.005, math.pi - gamma / 2, -math.pi + gamma / 2)
stopper_ve1 = chrono.ChLineSegment(stopper_arc.GetEndB(), chrono.ChVector3d(0, 0, 0))
stopper_ve2 = chrono.ChLineSegment(chrono.ChVector3d(0, 0, 0), stopper_arc.GetEndA())
crankstopper_path.AddSubLine(stopper_arc)
crankstopper_path.AddSubLine(stopper_ve1)
crankstopper_path.AddSubLine(stopper_ve2)

# Add the collision shape to the body
crank.EnableCollision(True)
crankpin_coll = chrono.ChCollisionShapePath2D(contact_mat, crankpin_path)
crankstopper_coll = chrono.ChCollisionShapePath2D(contact_mat, crankstopper_path)
crank.AddCollisionShape(crankpin_coll, chrono.ChFramed())
crank.AddCollisionShape(crankstopper_coll, chrono.ChFramed())
crank.GetCollisionModel().SetSafeMargin(0.02)

# Create a ChVisualShapeLine, a visualization asset for lines
crankasset = chrono.ChVisualShapeLine()
crankasset.SetLineGeometry(crankpin_path)
crank.AddVisualShape(crankasset)

crankasset2 = chrono.ChVisualShapeLine()
crankasset2.SetLineGeometry(crankstopper_path)
crank.AddVisualShape(crankasset2)

# ------------------------------------------------------------------------------

# Motor between crank and truss
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, ground, chrono.ChFramed(crank_center))
motor.SetSpeedFunction(chrono.ChFunctionConst(math.pi / 8.0))
sys.AddLink(motor)

# ------------------------------------------------------------------------------

# Create the VSG visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("2D collision")
vis.SetBackgroundColor(chrono.ChColor(0.1, 0.2, 0.3))
vis.Initialize()
vis.AddLogo()
vis.AddCamera(chrono.ChVector3d(0, 0, -4))
vis.AddTypicalLights()

# ------------------------------------------------------------------------------

# Simulation loop
while vis.Run() :
    vis.BeginScene() 
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)
