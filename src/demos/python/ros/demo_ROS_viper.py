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
# Demo showing the integration of ROS with the Viper rover model in python
#
# =============================================================================

import pychrono as ch
import pychrono.robot as robot
import pychrono.ros as chros

def main():
    # Create Chrono system
    system = ch.ChSystemNSC()
    system.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))
    ch.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    ch.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

    # Create ground body
    ground_mat = ch.ChContactMaterialNSC()
    ground = ch.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
    ground.SetPos(ch.ChVector3d(0, 0, -1))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(ch.GetChronoDataFile("textures/concrete.jpg"))
    system.Add(ground)

    # Create Viper rover
    driver = robot.ViperDCMotorControl()
    rover = robot.Viper(system)
    rover.SetDriver(driver)
    rover.Initialize(ch.ChFramed(ch.ChVector3d(0, -0.2, 0), ch.ChQuaterniond(1, 0, 0, 0)))

    # Create ROS manager
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSViperDCMotorControlHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, rover.GetChassis().GetBody(), "~/output/viper/state"))
    ros_manager.Initialize()

    # Simulation loop
    time = 0
    time_step = 1e-3
    time_end = 30

    while time < time_end:
        time = system.GetChTime()

        rover.Update()
        if not ros_manager.Update(time, time_step):
            break

        system.DoStepDynamics(time_step)

if __name__ == "__main__":
    main()