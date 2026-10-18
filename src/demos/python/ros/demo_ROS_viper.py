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
# Viper rover + ROS, in Python. Subscribes to motor commands
# (chrono_ros_interfaces/msg/ViperDCMotorControl) and publishes the rover state.
# ChROSViperDCMotorControlHandler is the same C++ built-in used by the C++ demo,
# here taking a pychrono.robot ViperDCMotorControl driver.
#
#   ros2 topic echo /chrono_ros_node/output/rover/state/pose
#
# Opens a VSG window by default; pass --headless to run without one.
#
# =============================================================================

import sys

import pychrono as ch
import pychrono.robot as robot
import pychrono.ros as chros


def main():
    system = ch.ChSystemNSC()
    system.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))
    system.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)
    ch.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    ch.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

    # Ground.
    ground_mat = ch.ChContactMaterialNSC()
    ground = ch.ChBodyEasyBox(30, 30, 1, 1000, True, True, ground_mat)
    ground.SetPos(ch.ChVector3d(0, 0, -0.5))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(ch.GetChronoDataFile("textures/concrete.jpg"), 60, 45)
    system.Add(ground)

    # Viper rover with a DC-motor-control driver (commanded from ROS).
    driver = robot.ViperDCMotorControl()
    rover = robot.Viper(system, robot.ViperWheelType_RealWheel)
    rover.SetDriver(driver)
    rover.Initialize(ch.ChFramed(ch.ChVector3d(0, 0, 0.5), ch.ChQuaterniond(1, 0, 0, 0)))

    # ROS: clock + Viper motor-control subscriber + rover state publisher.
    ros_manager = chros.ChROSManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSViperDCMotorControlHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, rover.GetChassis().GetBody(), "~/output/rover/state"))
    ros_manager.Initialize()

    # Optional run-time visualization (pass --headless to skip the window). VSG
    # needs a Vulkan ICD: a GPU driver, or the lavapipe software fallback
    # (mesa-vulkan-drivers). Imported lazily so --headless needs no Vulkan at all.
    vis = None
    if "--headless" not in sys.argv:
        import pychrono.vsg3d as vsg
        vis = vsg.ChVisualSystemVSG()
        vis.AttachSystem(system)
        vis.AddCamera(ch.ChVector3d(3, 3, 1))
        vis.SetWindowTitle("Viper Rover on Rigid Terrain")
        vis.Initialize()

    # Simulation loop.
    time = 0
    time_step = 1e-3
    time_end = 30

    while time < time_end:
        if vis is not None:
            if not vis.Run():
                break
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        time = system.GetChTime()

        rover.Update()
        if not ros_manager.Update(time, time_step):
            break

        system.DoStepDynamics(time_step)


if __name__ == "__main__":
    main()
