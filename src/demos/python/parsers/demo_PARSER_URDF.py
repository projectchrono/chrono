# =============================================================================
# PROJECT CHRONO - http:#projectchrono.org
#
# Copyright (c) 2023 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http:#projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Radu Serban, Aaron Young
# =============================================================================
#
# Demo for the URDF -> Chrono parser in python
#
# =============================================================================

import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.parsers as parsers

def main():
    # Create a Chrono system
    system = chrono.ChSystemSMC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    # Create a ground body
    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground_box = chrono.ChVisualShapeBox(3, 2, 0.1)
    ground_box.SetTexture(chrono.GetChronoDataFile("textures/checker2.png"))
    ground.AddVisualShape(ground_box)
    system.Add(ground)

    # Create the parser instance
    filename = "robot/r2d2/r2d2.urdf"
    parser = parsers.ChParserURDF(chrono.GetChronoDataFile(filename))

    # Set root body pose
    parser.SetRootInitPose(chrono.ChFramed(chrono.ChVector3d(0, 0, 1.5), chrono.QUNIT))

    # Make all eligible joints as actuated
    parser.SetAllJointsActuationType(parsers.ChParserURDF.ActuationType_POSITION);

    # Example: change contact material properties for a body
    ####mat = ChContactMaterialData()
    ####mat.kn = 2.5e6
    ####parser.SetBodyContactMaterial("head", mat)  ## hardcoded for R2D2 model

    # Display raw XML string
    print("\nURDF input\n")
    print(parser.GetXMLstring())

    # Report parsed elements
    parser.PrintModelBodyTree()
    parser.PrintModelBodies()
    parser.PrintModelJoints()

    # Create the Chrono model
    parser.PopulateSystem(system)

    # Get location of the root body
    root_loc = parser.GetRootChBody().GetPos()

    # Fix root body
    parser.GetRootChBody().SetFixed(True)

    # Example: Change actuation function for a particular joint
    sfun = chrono.ChFunctionSine(0, 0.2, 1.0)
    parser.SetMotorFunction("head_swivel", sfun)  # hardcoded for R2D2 model

    # Create the irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("URDF Parser Demo")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(root_loc + chrono.ChVector3d(3, 3, 0), root_loc)
    vis.AddTypicalLights()

    # Simulation loop
    step_size = 1e-3
    real_timer = chrono.ChRealtimeStepTimer()

    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        system.DoStepDynamics(step_size)
        real_timer.Spin(step_size)

if __name__ == "__main__":
    main()