# =============================================================================
# PROJECT CHRONO - http:#projectchrono.org
#
# Copyright (c) 2024 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http:#projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Dario Fusai
# =============================================================================
#
# Demo of industrial robot models and kinematics.
#
# =============================================================================

import pychrono as chrono
import pychrono.robot as chronorob
try:
   from pychrono import irrlicht as chronoirr
except:
   print('Could not import ChronoIrrlicht')

# Use analytical Inverse Kinematics (true) or Chrono Imposed motion (false)
USE_ANALYTICAL_IK = True

# Physical system ---------------------------------------------------------
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Floor
floor = chrono.ChBody()
floor.SetFixed(True)
sys.Add(floor)

# Robot model -------------------------------------------------------------
# Arm lengths
H = 0.180
L1 = 0.250
L2 = 0.317
L3 = 0.258
lengths = [H, L1, L2, L3]

# Create industrial 6 dofs articulated robot from given arm lengths
robot = chronorob.IndustrialRobot6dof(sys, lengths, chrono.ChFramed())
# robot.Add1dShapes()  # add segment-like shapes
robot.Add3dShapes(0.05)  # add 3d silhouette shapes

# Target keyframes, in operation space
motion_cycle_time = 5
key_0 = chrono.ChCoordsysd(robot.GetMarkerTCP().GetAbsCoordsys())
key_1 = chrono.ChCoordsysd(key_0.pos + chrono.ChVector3d(-0.1, 0.2, 0), chrono.QuatFromAngleZ(30 * chrono.CH_DEG_TO_RAD) * key_0.rot)
key_2 = chrono.ChCoordsysd(key_1.pos + chrono.ChVector3d(0.2, -0.1, 0.2), chrono.QuatFromAngleX(-70 * chrono.CH_DEG_TO_RAD) * key_0.rot)
keys = [key_0, key_1, key_2, key_0]

positions = []
rotations = []
for i in range(len(keys)):
   positions.append(keys[i].pos)
   rotations.append(keys[i].rot)

# Position motion law
posline = chrono.ChLineBezier(chrono.ChBezierCurve(positions))
pos_spacefun = chrono.ChFunctionRamp(0, 1. / motion_cycle_time)

posfun = chrono.ChFunctionPositionLine()
posfun.SetLine(posline)
posfun.SetSpaceFunction(pos_spacefun)

# Rotation motion law
rotfun = chrono.ChFunctionRotationBSpline(1, rotations)
rot_spacefun = chrono.ChFunctionRamp(0, 1.0 / motion_cycle_time)
rotfun.SetSpaceFunction(rot_spacefun)

trajectory_vis = chrono.ChVisualShapeLine()
trajectory_vis.SetLineGeometry(posline)
trajectory_vis.SetColor(chrono.ChColor(1, 1, 0))
floor.AddVisualShape(trajectory_vis)

# Kinematics --------------------------------------------------------------
# If analytical solution of inverse kinematics is enabled, use appropriate class to retrieve
# angles that must be provided to motors, at run time, to follow trajectory

markerlist = robot.GetMarkerlist()
robot_joint_coords = []
for i in range(len(markerlist)):
    robot_joint_coords.append(markerlist[i].GetAbsCoordsys())

# Angles theta2 & theta3 that would be needed to move 3dof sub-robot to a vertical configuration
vert_angs = [0, chrono.CH_PI_2]

# Class for analytical (inverse) kinematics of 6 dofs robots with spherical wrist.
# Useful to impose a trajectory to a robot end effector
kin_6dof_sph = chronorob.IndustrialKinematics6dofSpherical(robot_joint_coords, vert_angs)

# Imposed motion ----------------------------------------------------------
# If analytical solution of inverse kinematics is *not* enabled, constrain the end-effector to
# follow the trajectory through an imposed motion. IK is automatically obtained by
# solution of links

if not USE_ANALYTICAL_IK:
    # Disable robot direct actuation
    robot.DisableMotors(True)

    # Impose trajectory on robot end-effector
    imposed_motion = chrono.ChLinkMotionImposed()
    imposed_motion.Initialize(robot.GetEndEffector(), robot.GetBase(), False, robot.GetMarkerTCP().GetAbsFrame(), robot.GetBase().GetFrameCOMToAbs())
    imposed_motion.SetPositionFunction(posfun)
    imposed_motion.SetRotationFunction(rotfun)
    sys.Add(imposed_motion)

# Irrlicht ----------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Industrial Robot")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 1))
vis.AddTypicalLights()

vis.EnableAbsCoordsysDrawing(True)

# Simulation loop ---------------------------------------------------------
timestep = 0.01
realtime_timer = chrono.ChRealtimeStepTimer()

solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(200)
solver.SetTolerance(1e-5)
sys.SetSolver(solver)

while vis.Run():
    # Updates
    t = sys.GetChTime()
    if t > motion_cycle_time:
        sys.SetChTime(0)
    targetcoord = chrono.ChCoordsysd(posfun.GetPos(t), rotfun.GetQuat(t)) # update target

    # Graphics
    vis.BeginScene()
    vis.Render()
    chronoirr.drawCoordsys(vis, targetcoord, 0.1) # draw target
    for ii in keys:
        chronoirr.drawCoordsys(vis, ii, 0.05) # draw key frames
    vis.EndScene()

    # Impose following of trajectory through analytical IK, at run-time
    if USE_ANALYTICAL_IK:       
        angles_ik = kin_6dof_sph.GetIK(targetcoord)
        robot.SetSetpoints(angles_ik, t)

    # Advance simulation
    sys.DoStepDynamics(timestep)
    realtime_timer.Spin(timestep)