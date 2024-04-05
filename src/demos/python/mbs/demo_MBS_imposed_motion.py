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
# Authors: Jay Taves (Py), Alessandro Tasora (C++)
# =============================================================================
#
# Demo code about how to impose 3D position and/or 3D rotation to a rigid body
# with functions of time
#
# Recall that Irrlicht uses a left-hand frame, so everything is rendered with
# left and right flipped.
#
# =============================================================================

import pychrono as chrono
import pychrono.irrlicht as irr
import math as m

print("Copyright (c) 2017 projectchrono.org")

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

sys = chrono.ChSystemNSC()

# Create a floor
mfloor = chrono.ChBodyEasyBox(3, 0.2, 3, 1000, False, False)
mfloor.SetFixed(True)
sys.Add(mfloor)

#
# EXAMPLE 1
#
# In this example we impose position and rotation of a body shape respect to absolute reference,
# using the following methods:
#
# position:  use a triplet of X,Y,Z ChFunction objects
# rotation:  use a fixed axis of rotation and an angle(time) function defined via a ChFunction

# Create the object to move
mmoved_1 = chrono.ChBodyEasyMesh(chrono.GetChronoDataFile(
    "models/support.obj"), 1000, True, True, False)
sys.Add(mmoved_1)
mmoved_1.SetPos(chrono.ChVector3d(-0.5, 0, 0))

# Create a position function p(t) from three x,y,z distinct ChFunction objects,
# in this case two sine functions on y and z, whereas x remains constant 0 by default.
f_xyz = chrono.ChFunctionPositionXYZFunctions()
f_xyz.SetFunctionY(chrono.ChFunctionSine(0.5, 0.5))
f_xyz.SetFunctionZ(chrono.ChFunctionSine(0.5, 0.5))

# Create a rotation function q(t) from a angle(time) rotation with fixed axis:
f_rot_axis = chrono.ChFunctionRotationAxis()
f_rot_axis.SetFunctionAngle(chrono.ChFunctionSine(chrono.CH_PI, 0.15))
f_rot_axis.SetAxis(chrono.ChVector3d(1, 1, 1).GetNormalized())

# Create the constraint to impose motion and rotation.
# Note that the constraint acts by imposing the motion and rotation between
# frame1 of Body1 (the impose_1 body in this case) and frame2 of Body2 (the floor
# fixed reference in this case, but it could also be another moving object).
# We initially set both frame1 and frame2 in the same absolute position, btw in the
# main reference of the moved mesh, by setting it during the Initialize() method.
impose_1 = chrono.ChLinkMotionImposed()
sys.Add(impose_1)
impose_1.Initialize(mmoved_1, mfloor, chrono.ChFramed(mmoved_1.GetPos()))
impose_1.SetPositionFunction(f_xyz)
impose_1.SetRotationFunction(f_rot_axis)

#
# EXAMPLE 2
#
#
# In this example we impose position and rotation of a shape respect to absolute reference,
# using the following methods:
#
# position:  use a Bspline
# rotation:  use a quaternion spline.

mmoved_2 = chrono.ChBodyEasyMesh(chrono.GetChronoDataFile(
    "models/support.obj"), 1000, True, True, False)
sys.Add(mmoved_2)
mmoved_2.SetPos(chrono.ChVector3d(0.5, 0, 0))

v1 = chrono.ChVector3d(0, 0, 0)
v2 = chrono.ChVector3d(0.3, 0, 0)
v3 = chrono.ChVector3d(0.5, 0.2, 0)
v4 = chrono.ChVector3d(0.6, 0.3, 0)
v5 = chrono.ChVector3d(0.5, 0.5, 0.1)
v6 = chrono.ChVector3d(0, 0.5, 0.1)
splinepoints = chrono.vector_ChVector3d([v1, v2, v3, v4, v5, v6])
mspline = chrono.ChLineBSpline(3, splinepoints)
mspline.SetClosed(True)

f_line = chrono.ChFunctionPositionLine()
f_line.SetLine(mspline)
f_line.SetSpaceFunction(chrono.ChFunctionRamp(0, 0.2))

# Create a spline rotation interpolation
q1 = chrono.ChQuaterniond(1, 0, 0, 0)
q2 = chrono.ChQuaterniond(0, 0, 1, 0)
q3 = chrono.QuatFromAngleZ(1.2)
q4 = chrono.QuatFromAngleZ(2.2)
q5 = chrono.QuatFromAngleZ(-1.2)
q6 = chrono.ChQuaterniond(0, 1, 0, 0)
spinerots = chrono.vector_ChQuaterniond([q1, q2, q3, q4, q5, q6])
f_rotspline = chrono.ChFunctionRotationBSpline(1, [q1, q2, q3, q4, q5, q6])
f_rotspline.SetClosed(True)
f_rotspline.SetSpaceFunction(chrono.ChFunctionRamp(0, 0.2))

impose_2 = chrono.ChLinkMotionImposed()
sys.Add(impose_2)
impose_2.Initialize(mmoved_2, mfloor, chrono.ChFramed(mmoved_2.GetPos()))
impose_2.SetPositionFunction(f_line)
impose_2.SetRotationFunction(f_rotspline)

# Visualize the position spline
mglyphasset = chrono.ChVisualShapeLine()
mglyphasset.SetLineGeometry(mspline)
impose_2.AddVisualShape(mglyphasset)

mmoved_2.SetPos(f_line.GetPos(0) >> impose_2.GetFrame2Rel()
                >> impose_2.GetBody2().GetCoordsys())

#
# EXAMPLE 3
#
#
# In this example we impose position and rotation of a shape respect to absolute reference,
# using the following methods:
#
# position:  use a continuous setpoint (FOH first order hold), ie. a sin(t) set in simulation loop
# rotation:  use a continuous setpoint (FOH first order hold).

mmoved_3 = chrono.ChBodyEasyMesh(chrono.GetChronoDataFile(
    "models/support.obj"), 1000, True, True, False)
sys.Add(mmoved_3)
mmoved_3.SetPos(chrono.ChVector3d(1.5, 0, 0))

f_pos_setpoint = chrono.ChFunctionPositionSetpoint()
f_rot_setpoint = chrono.ChFunctionRotationSetpoint()

impose_3 = chrono.ChLinkMotionImposed()
sys.Add(impose_3)
impose_3.Initialize(mmoved_3, mfloor, chrono.ChFramed(mmoved_3.GetPos()))
impose_3.SetPositionFunction(f_pos_setpoint)
impose_3.SetRotationFunction(f_rot_setpoint)

#
# EXAMPLE 4
#
#
# In this example we impose position and rotation of a shape respect to absolute reference,
# using the following methods:
#
# position:  [nothing]
# rotation:  use three angle functions of time, ie. three ChFunction objects

mmoved_4 = chrono.ChBodyEasyMesh(chrono.GetChronoDataFile(
    "models/support.obj"), 1000, True, True, False)
sys.Add(mmoved_4)
mmoved_4.SetPos(chrono.ChVector3d(2.5, 0, 0))

f_abc_angles = chrono.ChFunctionRotationABCFunctions()
f_abc_angles.SetRotationRepresentation(chrono.RotRepresentation_CARDAN_ANGLES_XYZ)
f_abc_angles.SetFunctionAngleA(chrono.ChFunctionSine(0.3, 2))
f_abc_angles.SetFunctionAngleB(chrono.ChFunctionRamp(0, 0.2))

impose_4 = chrono.ChLinkMotionImposed()
sys.Add(impose_4)
impose_4.Initialize(mmoved_4, mfloor, chrono.ChFramed(mmoved_4.GetPos()))
impose_4.SetRotationFunction(f_abc_angles)

#
# EXAMPLE 5
#
#
# In this example we impose rotation of a shape respect to absolute reference,
# using the following methods:
#
# rotation:  use a SQUAD (smooth interpolation of quaternion rotations as in Shoemake 1987 paper).

mmoved_5 = chrono.ChBodyEasyMesh(chrono.GetChronoDataFile(
    "models/support.obj"), 1000, True, True, False)
sys.Add(mmoved_5)
mmoved_5.SetPos(chrono.ChVector3d(1, 1, 0))

q1 = chrono.ChQuaterniond(1, 0, 0, 0)
q2 = chrono.ChQuaterniond(0, 0, 1, 0)
q3 = chrono.QuatFromAngleZ(1.2)
q4 = chrono.QuatFromAngleZ(2.2)
q5 = chrono.QuatFromAngleZ(-1.2)
q6 = chrono.ChQuaterniond(0, 1, 0, 0)

f_squad = chrono.ChFunctionRotationSQUAD([q1, q2, q3, q4, q5, q6])
f_squad.SetClosed(True)
f_squad.SetSpaceFunction(chrono.ChFunctionRamp(0, 0.2))

# Create the constraint to impose motion and rotation
impose_5 = chrono.ChLinkMotionImposed()
sys.Add(impose_5)
impose_5.Initialize(mmoved_5, mfloor, chrono.ChFramed(mmoved_5.GetPos()))
impose_5.SetRotationFunction(f_squad)

# Create the Irrlicht application
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Imposing rotation and position to bodies')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, -3))
vis.AddTypicalLights()

# Simulation loop
frame = 0

while vis.Run():
    vis.BeginScene() 
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

    if (frame % 10 == 0):
        t = sys.GetChTime()
        f_pos_setpoint.SetSetpoint(
            chrono.ChVector3d(0.2 * m.cos(t * 12), 0.2 * m.sin(t * 12), 0), t)

    frame = frame + 1