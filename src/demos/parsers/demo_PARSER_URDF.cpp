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
#include "chrono/assets/ChBoxShape.h"

#include "chrono_parsers/ChParserURDF.h"

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

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::string filename = (argc > 1) ? std::string(argv[1]) : "robot/r2d2/r2d2.urdf";
    ////std::string filename = (argc > 1) ? std::string(argv[1]) : "robot/robosimian/rs.urdf";
    ////std::string filename = (argc > 1) ? std::string(argv[1]) : "robot/viper/viper.urdf";

    // Create a Chrono system
    ChSystemSMC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.8));

    // Create a "floor" body
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetBodyFixed(true);
    auto floor_box = chrono_types::make_shared<ChBoxShape>(3, 2, 0.1);
    floor_box->SetTexture(GetChronoDataFile("textures/checker2.png"));
    floor->AddVisualShape(floor_box);
    sys.AddBody(floor);

    // Create parser instance
    ChParserURDF parser(GetChronoDataFile(filename));

    // Set root body pose
    parser.SetRootInitPose(ChFrame<>(ChVector<>(0, 0, 1.5), QUNIT));

    // Make all eligible joints as actuated
    parser.SetAllJointsActuationType(ChParserURDF::ActuationType::POSITION);

    // Example: change contact material properties for a body
    ////ChContactMaterialData mat;
    ////mat.kn = 2.5e6;
    ////parser.SetBodyContactMaterial("head", mat);  // hardcoded for R2D2 model

    // Display raw XML string
    std::cout << "\nURDF input\n" << std::endl;
    std::cout << parser.GetXMLstring() << std::endl;

    // Report parsed elements
    parser.PrintModelBodyTree();
    parser.PrintModelBodies();
    parser.PrintModelJoints();

    // Create the Chrono model
    parser.PopulateSystem(sys);

    // Optional custom processing
    std::cout << "\nCustom processing example - scan elements \"link\"\n" << std::endl;
    class MyCustomProcessor : public ChParserURDF::CustomProcessor {
        virtual void Process(const tinyxml2::XMLElement& element, ChSystem& system) override {
            std::cout << "Process element: " << element.Name() << std::endl;
            if (element.FirstChildElement()) {
                std::cout << "   First child name: " << element.FirstChildElement()->Name() << std::endl;
            }
        }
    };
    parser.CustomProcess("link", chrono_types::make_shared<MyCustomProcessor>());
    std::cout << std::endl;

    // Get location of the root body
    auto root_loc = parser.GetRootChBody()->GetPos();

    // Fix root body
    parser.GetRootChBody()->SetBodyFixed(true);

    // Example: Change actuation function for a particular joint
    auto sfun = chrono_types::make_shared<ChFunction_Sine>(0, 0.2, 1.0);
    parser.SetMotorFunction("head_swivel", sfun);  // hardcoded for R2D2 model

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
            vis_irr->SetWindowTitle("URDF parser demo");
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
            vis_vsg->SetWindowTitle("URDF parser demo");
            vis_vsg->AddCamera(root_loc + ChVector<>(3, 3, 0), root_loc);
            vis_vsg->SetWindowSize(ChVector2<int>(1200, 800));
            vis_vsg->SetWindowPosition(ChVector2<int>(500, 100));
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
    double step_size = 1e-3;
    ChRealtimeStepTimer real_timer;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(step_size);
        real_timer.Spin(step_size);
    }

    return 0;
}
