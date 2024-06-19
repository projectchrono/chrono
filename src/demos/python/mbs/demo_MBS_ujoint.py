# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2022 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

print ("Example: demonstration of a universal joint")

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('relative/path/to/data/directory/')

# ---------------------------------------------------------------------
#
#  Create the simulation sys and add items
#

sys      = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Set the half-length of the two shafts
hl = 2
 
# Set the bend angle between the two shafts (positive rotation
# about the global X axis)
angle = math.pi / 6.
cosa  = math.cos(angle)
sina  = math.sin(angle)
rot   = chrono.QuatFromAngleX(angle) 

# Create the ground body
# ----------------------
ground = chrono.ChBody()
ground.SetFixed(True)
ground.EnableCollision(False)
sys.Add(ground)

# attach visualization assets to represent the revolute and cylindrical
# joints that connect the two shafts to ground
cyl_1 = chrono.ChVisualShapeCylinder(0.3, 0.4)
ground.AddVisualShape(cyl_1, chrono.ChFramed(chrono.ChVector3d(0, 0, -hl), chrono.QUNIT))

seg = chrono.ChLineSegment(chrono.ChVector3d(0, -(hl - 0.2) * sina, (hl - 0.2) * cosa),
                           chrono.ChVector3d(0, -(hl + 0.2) * sina, (hl + 0.2) * cosa))
cyl_2 = chrono.ChVisualShapeCylinder(0.3, seg.GetLength())
ground.AddVisualShape(cyl_2, seg.GetFrame())


# Create the first shaft body
# ---------------------------
shaft_1 = chrono.ChBody()
sys.AddBody(shaft_1)
shaft_1.SetFixed(False)
shaft_1.EnableCollision(False)
shaft_1.SetMass(1)
shaft_1.SetInertiaXX(chrono.ChVector3d(1, 1, 0.2))
shaft_1.SetPos(chrono.ChVector3d(0, 0, -hl))
shaft_1.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

# Add visualization assets to represent the shaft (a box) and the arm of the
# universal joint's cross associated with this shaft (a cylinder)
box_1 = chrono.ChVisualShapeBox(0.3, 0.3, 1.8 * hl)
shaft_1.AddVisualShape(box_1)

cyl_2 = chrono.ChVisualShapeCylinder(0.05, 0.4)
cyl_2.SetColor(chrono.ChColor(0.9, 0.4, 0.1))
shaft_1.AddVisualShape(cyl_2, chrono.ChFramed(chrono.ChVector3d(0, 0, hl), chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Create the second shaft body
# ----------------------------

# The second shaft is identical to the first one, but initialized at an angle
# equal to the specified bend angle.

shaft_2 = chrono.ChBody()
sys.AddBody(shaft_2)
shaft_2.SetFixed(False)
shaft_2.EnableCollision(False)
shaft_2.SetMass(1)
shaft_2.SetInertiaXX(chrono.ChVector3d(1, 1, 0.2))
shaft_2.SetPos(chrono.ChVector3d(0, -hl * sina, hl * cosa))
shaft_2.SetRot(rot)

# Add visualization assets to represent the shaft (a box) and the arm of the
# universal joint's cross associated with this shaft (a cylinder)
box_1 = chrono.ChVisualShapeBox(0.3, 0.3, 1.8 * hl)
shaft_2.AddVisualShape(box_1)

cyl_2 = chrono.ChVisualShapeCylinder(0.05, 0.4)
cyl_2.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
shaft_2.AddVisualShape(cyl_2, chrono.ChFramed(chrono.ChVector3d(0, 0, -hl), chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Connect the first shaft to ground
# ---------------------------------

# Use a rotational motor to impose both the revolute joint constraints, as well
# as constant angular velocity. Here, we drive the motor angle with a ramp function.
# Alternatively, we could use a ChLinkMotorAngularSpeed with constant speed.
# The joint is located at the origin of the first shaft.
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(ground,
                 shaft_1,
                 chrono.ChFramed(chrono.ChVector3d(0, 0, -hl), chrono.ChQuaterniond(1, 0, 0, 0)))
motor.SetAngleFunction(chrono.ChFunctionRamp(0, 1))
sys.AddLink(motor)

# Connect the second shaft to ground through a cylindrical joint
# --------------------------------------------------------------

# Use a cylindrical joint so that we do not have redundant constraints
# (note that, technically Chrono could deal with a revolute joint here).
# the joint is located at the origin of the second shaft.

cyljoint = chrono.ChLinkLockCylindrical()
sys.AddLink(cyljoint)
cyljoint.Initialize(ground, 
                    shaft_2,
                    chrono.ChFramed(chrono.ChVector3d(0, -hl * sina, hl * cosa), rot))

# Connect the two shafts through a universal joint
# ------------------------------------------------

# The joint is located at the global origin.  Its kinematic constraints will
# enforce orthogonality of the associated cross.

ujoint = chrono.ChLinkUniversal()
sys.AddLink(ujoint)
ujoint.Initialize(shaft_1,
                  shaft_2,
                  chrono.ChFramed(chrono.ChVector3d(0, 0, 0), rot))


# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Universal joint demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 1, -1.5))
vis.AddTypicalLights()


#  Run the simulation
frame = 0

while vis.Run():
    vis.BeginScene() 
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)
    frame += 1

    if frame % 20 == 0:
        omega_1 = shaft_1.GetAngVelLocal().z
        omega_2 = shaft_2.GetAngVelLocal().z
        print('{:.4}'.format(str(sys.GetChTime())), '{:.6}'.format(str(omega_1)), '{:.6}'.format(str(omega_2)))






