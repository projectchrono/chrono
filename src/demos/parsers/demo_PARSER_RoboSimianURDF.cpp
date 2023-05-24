// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demo for the URDF -> Chrono parser
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_parsers/ChParserURDF.h"
#include "chrono_parsers/ChRobotActuation.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::parsers;

// -----------------------------------------------------------------------------

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Actuation input files

// WALK
////std::string start_filename = "";
////std::string cycle_filename = "robot/robosimian/actuation/walking_cycle.txt";
////std::string stop_filename = "";

// SCULL
std::string start_filename = "robot/robosimian/actuation/sculling_start.txt";
std::string cycle_filename = "robot/robosimian/actuation/sculling_cycle2.txt";
std::string stop_filename = "robot/robosimian/actuation/sculling_stop.txt";

// INCHWORK
////std::string start_filename = "robot/robosimian/actuation/inchworming_start.txt";
////std::string cycle_filename = "robot/robosimian/actuation/inchworming_cycle.txt";
////std::string stop_filename = "robot/robosimian/actuation/inchworming_stop.txt";

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Create a Chrono system
    ChSystemSMC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.8));

    // Create a "floor" body
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetBodyFixed(true);
    auto floor_box = chrono_types::make_shared<ChBoxShape>(6, 2, 0.1);
    floor_box->SetTexture(GetChronoDataFile("textures/checker2.png"));
    floor->AddVisualShape(floor_box);
    sys.AddBody(floor);

    // Create parser instance
    ChParserURDF parser(GetChronoDataFile("robot/robosimian/rs.urdf"));

    // Set root body pose
    parser.SetRootInitPose(ChFrame<>(ChVector<>(0, 0, 1.5), QUNIT));

    // Make all eligible joints as actuated
    parser.SetAllJointsActuated(ChParserURDF::ActuationType::POSITION);

    // Example: change contact material properties for a body
    ////ChContactMaterialData mat;
    ////mat.kn = 2.5e6;
    ////parser.SetBodyContactMaterial("head", mat);  // hardcoded for R2D2 model

    // Report parsed elements
    parser.PrintModelBodies();
    parser.PrintModelJoints();

    // Create the Chrono model
    parser.PopulateSystem(sys);

    // Get location of the root body
    auto root_loc = parser.GetRootChBody()->GetPos();

    // Fix root body
    parser.GetRootChBody()->SetBodyFixed(true);

    // Read the list of actuated motors, cache the motor links, and set their actuation function
    int num_motors = 32;
    std::ifstream ifs(GetChronoDataFile("robot/robosimian/actuation/motor_names.txt"));
    std::vector<std::shared_ptr<ChLinkMotor>> motors(num_motors);
    std::vector<std::shared_ptr<ChFunction_Setpoint>> motor_functions(num_motors);
    for (int i = 0; i < num_motors; i++) {
        std::string name;
        ifs >> name;
        motors[i] = parser.GetChMotor(name);
        motor_functions[i] = chrono_types::make_shared<ChFunction_Setpoint>();
        motors[i]->SetMotorFunction(motor_functions[i]);
    }

    // Create a robot motor actuation object
    ChRobotActuation actuator(32,                                 // number motors
                              GetChronoDataFile(start_filename),  // start input file
                              GetChronoDataFile(cycle_filename),  // cycle input file
                              GetChronoDataFile(stop_filename),   // stop input file
                              true                                // repeat cycle
    );
    actuator.SetTimeOffsets(1.0, 0.5);
    actuator.SetVerbose(true);

    // Create the visualization window
    std::shared_ptr<ChVisualSystem> vis;
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->SetWindowSize(1200, 800);
            vis_irr->SetWindowTitle("NSC callbacks");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(root_loc + ChVector<>(3, 3, 0), root_loc);
            vis_irr->AddTypicalLights();

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
            vis_vsg->SetWindowTitle("NSC callbacks");
            vis_vsg->AddCamera(root_loc + ChVector<>(3, 3, 0), root_loc);
            vis_vsg->SetWindowSize(ChVector2<int>(1200, 800));
            vis_vsg->SetWindowPosition(ChVector2<int>(400, 100));
            vis_vsg->SetClearColor(ChColor(0.455f, 0.525f, 0.640f));
            vis_vsg->SetUseSkyBox(false);
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);
            vis_vsg->SetWireFrameMode(false);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop
    double step_size = 5e-4;
    ChRealtimeStepTimer real_timer;

    while (vis->Run()) {
        double time = sys.GetChTime();

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Update actuator and get current actuations
        actuator.Update(time);
        const auto& actuations = actuator.GetActuation();

        ////std::cout << time << std::endl;
        ////int k = 0;
        ////for (int i = 0; i < 4; i++) {
        ////    for (int j = 0; j < 8; j++) {
        ////        std::cout << actuations[k++] << " ";
        ////    }
        ////    std::cout << std::endl;
        ////}
        ////std::cout << std::endl;

        // Apply motor actuations
        for (int i = 0; i < num_motors; i++)
            motor_functions[i]->SetSetpoint(-actuations[i], time);

        // Advance system dynamics
        sys.DoStepDynamics(step_size);
        real_timer.Spin(step_size);
    }

    return 0;
}
