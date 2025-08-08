# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2023 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Aaron Young
# =============================================================================
#
# Demo showing the integration of ROS with the ChParserURDF in python
#
# =============================================================================

import pychrono as ch
import pychrono.irrlicht as irr
import pychrono.ros as chros
import pychrono.parsers as parsers


def create_terrain(
    system: ch.ChSystem,
    ground: ch.ChBody,
    length: int,
    width: int,
    height: int,
    offset: int,
):
    ground_mat = ch.ChContactMaterialSMC()
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
    # Create Chrono system
    system = ch.ChSystemSMC()
    system.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)
    system.SetSolverType(ch.ChSolver.Type_BARZILAIBORWEIN)
    system.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.8))

    ground_body = ch.ChBody()
    ground_body.SetName("floor")
    system.Add(ground_body)

    # Create the robosimian
    robot_urdf = ch.GetChronoDataFile("robot/robosimian/rs.urdf")
    robot = parsers.ChParserURDF(robot_urdf)

    # Set root body pose
    robot.SetRootInitPose(ch.ChFramed(ch.ChVector3d(0, 0, 1.5), ch.QUNIT))

    # Make all eligible joints as actuated (POSITION type) and overwrite wheel
    # motors with SPEED actuation.
    robot.SetAllJointsActuationType(parsers.ChParserURDF.ActuationType_POSITION)
    robot.SetJointActuationType("limb1_joint8", parsers.ChParserURDF.ActuationType_SPEED)
    robot.SetJointActuationType("limb2_joint8", parsers.ChParserURDF.ActuationType_SPEED)
    robot.SetJointActuationType("limb3_joint8", parsers.ChParserURDF.ActuationType_SPEED)
    robot.SetJointActuationType("limb4_joint8", parsers.ChParserURDF.ActuationType_SPEED)

    # Use convex hull for the sled collision shape
    robot.SetBodyMeshCollisionType("sled", parsers.ChParserURDF.MeshCollisionType_CONVEX_HULL)

    # Visualize collision shapes
    robot.EnableCollisionVisualization()

    # Create the Chrono model
    robot.PopulateSystem(system)

    # Get the selected bodies of the robot
    torso = robot.GetChBody("torso")
    sled = robot.GetChBody("sled")
    limb1_wheel = robot.GetChBody("limb1_link8")
    limb2_wheel = robot.GetChBody("limb2_link8")
    limb3_wheel = robot.GetChBody("limb3_link8")
    limb4_wheel = robot.GetChBody("limb4_link8")

    # Enable collision and set contact material for selected bodies of the robot
    sled.EnableCollision(True)
    limb1_wheel.EnableCollision(True)
    limb2_wheel.EnableCollision(True)
    limb3_wheel.EnableCollision(True)
    limb4_wheel.EnableCollision(True)

    # Update the contact material properties
    mat = ch.ChContactMaterialData()
    mat.mu = 0.8
    mat.cr = 0.0
    mat.Y = 1e7
    cmat = mat.CreateMaterial(system.GetContactMethod())
    sled.GetCollisionModel().SetAllShapesMaterial(cmat)
    limb1_wheel.GetCollisionModel().SetAllShapesMaterial(cmat)
    limb2_wheel.GetCollisionModel().SetAllShapesMaterial(cmat)
    limb3_wheel.GetCollisionModel().SetAllShapesMaterial(cmat)
    limb4_wheel.GetCollisionModel().SetAllShapesMaterial(cmat)

    # Fix root body
    robot.GetRootChBody().SetFixed(True)

    motors = []
    with open(ch.GetChronoDataFile("robot/robosimian/actuation/motor_names.txt"), "r") as f:
        for line in f:
            motor_name = line.strip()
            motor = robot.GetChMotor(motor_name)
            motor.SetMotorFunction(ch.ChFunctionSetpoint())
            motors.append(motor)

    # Create the robot motor actuation object
    cycle_filename = ch.GetChronoDataFile("robot/robosimian/actuation/walking_cycle.txt")
    actuator = parsers.ChRobotActuation(len(motors), "", cycle_filename, "", True)

    duration_pose = 1.0 # time interval to assume initial pose
    duration_settle_robot = 0.5 # time interval to settle the robot on the ground
    actuator.SetTimeOffsets(duration_pose, duration_settle_robot)
    actuator.SetVerbose(True)

    # ------------

    # Create ROS manager
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    # Create the TF handler that will publish the transform of the robot
    tf_handler = chros.ChROSTFHandler(100)
    tf_handler.AddTransform(ground_body, ground_body.GetName(), robot.GetChBody("torso"), "torso")
    tf_handler.AddTransform(robot.GetChBody("torso"), "torso", robot.GetChBody("limb1_link1"), "limb1_link1")
    tf_handler.AddTransform(robot.GetChBody("torso"), "torso", robot.GetChBody("limb2_link1"), "limb2_link1")
    tf_handler.AddTransform(robot.GetChBody("torso"), "torso", robot.GetChBody("limb3_link1"), "limb3_link1")
    tf_handler.AddTransform(robot.GetChBody("torso"), "torso", robot.GetChBody("limb4_link1"), "limb4_link1")
    tf_handler.AddURDF(robot)
    ros_manager.RegisterHandler(tf_handler)

    # Create a robot model handler that will publish the URDF file as a string for rviz to load
    # The QoS of this publisher is set to transient local, meaning we can publish once and late subscribers will still
    # receive the message. We do this by setting the update rate to infinity.
    robot_model_handler = chros.ChROSRobotModelHandler(robot)
    ros_manager.RegisterHandler(robot_model_handler)

    # Finally, initialize the ROS manager
    ros_manager.Initialize()

    # ------------

    # Create the irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("ROS RoboSimian URDF Demo")
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddTypicalLights()

    camera_lookat = robot.GetChBody("torso").GetPos()
    camera_loc = camera_lookat + ch.ChVector3d(3, 3, 0)
    vis.AddCamera(camera_loc, camera_lookat)

    # Simulation loop
    time = 0
    time_step = 5e-4
    time_end = 1000

    step_number = 0
    render_step_size = 1.0 / 60  # FPS = 60
    render_steps = int(render_step_size / time_step)

    terrain_created = False

    # Simulation loop
    realtime_timer = ch.ChRealtimeStepTimer()
    while time < time_end:
        if not vis.Run():
            break

        time = system.GetChTime()

        if not terrain_created and time > duration_pose:
            # set robot height
            z = limb1_wheel.GetPos().z - 0.15

            # create the terrain
            create_terrain(system, ground_body, 8, 2, z, 2)
            vis.BindItem(ground_body)

            # release the robot
            robot.GetRootChBody().SetFixed(False)

            terrain_created = True

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        step_number += 1

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
