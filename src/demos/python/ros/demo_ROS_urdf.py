# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2026 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Aaron Young, Patrick Chen
# =============================================================================
#
# ROS + ChParserURDF, in Python. A RoboSimian is parsed from URDF, actuated
# through a walking cycle, and its model + transforms are published for RViz2:
# the RobotModel handler publishes the URDF string on /robot_description, and the
# TF handler publishes the full link tree. Both take a pychrono.parsers
# ChParserURDF directly. Runs headless (view in RViz2).
#
#   ros2 topic echo /robot_description --once
#   ros2 run tf2_tools view_frames
#
# =============================================================================

import pychrono as ch
import pychrono.ros as chros
import pychrono.parsers as parsers


def create_terrain(system, ground, length, width, height, offset):
    ground_mat = ch.ChContactMaterial_DefaultMaterial(system.GetContactMethod())
    ground_mat.SetFriction(0.8)
    ground_mat.SetRestitution(0.0)
    ch.CastToChContactMaterialSMC(ground_mat).SetYoungModulus(1e7)

    ground.SetFixed(True)
    ground.SetPos(ch.ChVector3d(offset, 0, height - 0.1))
    ground.EnableCollision(True)

    ct_shape = ch.ChCollisionShapeBox(ground_mat, length, width, 0.2)
    ground.AddCollisionShape(ct_shape)

    box = ch.ChVisualShapeBox(length, width, 0.2)
    box.SetTexture(ch.GetChronoDataFile("textures/checker2.png"), length, width)
    ground.AddVisualShape(box)
    system.GetCollisionSystem().BindItem(ground)


def main():
    # Create the Chrono system.
    system = ch.ChSystemSMC()
    system.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)
    system.SetSolverType(ch.ChSolver.Type_BARZILAIBORWEIN)
    system.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.8))

    ground_body = ch.ChBody()
    ground_body.SetName("floor")
    system.Add(ground_body)

    # Parse the RoboSimian URDF and build the Chrono model.
    robot_urdf = ch.GetChronoDataFile("robot/robosimian/rs.urdf")
    robot = parsers.ChParserURDF(robot_urdf)
    robot.SetRootInitPose(ch.ChFramed(ch.ChVector3d(0, 0, 1.5), ch.QUNIT))
    robot.SetAllJointsActuationType(parsers.ChParserURDF.ActuationType_POSITION)
    robot.SetJointActuationType("limb1_joint8", parsers.ChParserURDF.ActuationType_SPEED)
    robot.SetJointActuationType("limb2_joint8", parsers.ChParserURDF.ActuationType_SPEED)
    robot.SetJointActuationType("limb3_joint8", parsers.ChParserURDF.ActuationType_SPEED)
    robot.SetJointActuationType("limb4_joint8", parsers.ChParserURDF.ActuationType_SPEED)
    robot.SetBodyMeshCollisionType("sled", parsers.ChParserURDF.MeshCollisionType_CONVEX_HULL)
    robot.EnableCollisionVisualization()
    robot.PopulateSystem(system)

    limb1_wheel = robot.GetChBody("limb1_link8")
    sled = robot.GetChBody("sled")
    limb2_wheel = robot.GetChBody("limb2_link8")
    limb3_wheel = robot.GetChBody("limb3_link8")
    limb4_wheel = robot.GetChBody("limb4_link8")

    for body in (sled, limb1_wheel, limb2_wheel, limb3_wheel, limb4_wheel):
        body.EnableCollision(True)

    mat = ch.ChContactMaterialData()
    mat.mu = 0.8
    mat.cr = 0.0
    mat.Y = 1e7
    cmat = mat.CreateMaterial(system.GetContactMethod())
    for body in (sled, limb1_wheel, limb2_wheel, limb3_wheel, limb4_wheel):
        body.GetCollisionModel().SetAllShapesMaterial(cmat)

    robot.GetRootChBody().SetFixed(True)

    # Set up the walking-cycle actuation.
    motors = []
    with open(ch.GetChronoDataFile("robot/robosimian/actuation/motor_names.txt"), "r") as f:
        for line in f:
            motor = robot.GetChMotor(line.strip())
            motor.SetMotorFunction(ch.ChFunctionSetpoint())
            motors.append(motor)

    cycle_filename = ch.GetChronoDataFile("robot/robosimian/actuation/walking_cycle.txt")
    actuator = parsers.ChRobotActuation(len(motors), "", cycle_filename, "", True)
    duration_pose = 1.0          # assume initial pose
    duration_settle_robot = 0.5  # settle on the ground
    actuator.SetTimeOffsets(duration_pose, duration_settle_robot)
    actuator.SetVerbose(True)

    # ROS: clock + tf (per-limb transforms and the full URDF tree) + robot model.
    ros_manager = chros.ChROSManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    tf_handler = chros.ChROSTFHandler(100)
    tf_handler.AddTransform(ground_body, ground_body.GetName(), robot.GetChBody("torso"), "torso")
    tf_handler.AddTransform(robot.GetChBody("torso"), "torso", robot.GetChBody("limb1_link1"), "limb1_link1")
    tf_handler.AddTransform(robot.GetChBody("torso"), "torso", robot.GetChBody("limb2_link1"), "limb2_link1")
    tf_handler.AddTransform(robot.GetChBody("torso"), "torso", robot.GetChBody("limb3_link1"), "limb3_link1")
    tf_handler.AddTransform(robot.GetChBody("torso"), "torso", robot.GetChBody("limb4_link1"), "limb4_link1")
    tf_handler.AddURDF(robot)
    ros_manager.RegisterHandler(tf_handler)

    # Publish the URDF string on /robot_description (latched, for RViz2).
    ros_manager.RegisterHandler(chros.ChROSRobotModelHandler(robot))

    ros_manager.Initialize()

    # Simulation loop (headless; view /robot_description + /tf in RViz2).
    time = 0
    time_step = 5e-4
    time_end = 1000

    terrain_created = False
    realtime_timer = ch.ChRealtimeStepTimer()
    while time < time_end:
        time = system.GetChTime()

        if not terrain_created and time > duration_pose:
            z = limb1_wheel.GetPos().z - 0.15
            create_terrain(system, ground_body, 8, 2, z, 2)
            robot.GetRootChBody().SetFixed(False)  # release the robot
            terrain_created = True

        actuator.Update(time)
        actuations = actuator.GetActuation()
        for i, motor in enumerate(motors):
            ch.CastToChFunctionSetpoint(motor.GetMotorFunction()).SetSetpoint(-actuations[i], time)

        if not ros_manager.Update(time, time_step):
            break

        system.DoStepDynamics(time_step)
        realtime_timer.Spin(time_step)


if __name__ == "__main__":
    main()
