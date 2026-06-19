// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Patrick Chen
// =============================================================================
//
// Demo: integrating ROS with the Viper rover. Subscribes to motor commands
// (chrono_ros_interfaces/msg/ViperDCMotorControl) and publishes the rover state.
//
//   ros2 topic echo /chrono_ros_node/output/rover/state/pose
//   ros2 topic pub  /chrono_ros_node/input/driver_inputs ...
//
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/robot/viper/ChROSViperDCMotorControlHandler.h"

using namespace chrono;
using namespace chrono::viper;
using namespace chrono::ros;

// Run-time visualization system (IRRLICHT or VSG).
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl << std::endl;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    auto ground_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(30, 30, 1, 1000, true, true, ground_mat);
    ground->SetPos(ChVector3d(0, 0, -0.5));
    ground->SetFixed(true);
    ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 60, 45);
    sys.Add(ground);

    // Viper rover with a DC-motor-control driver (commanded from ROS).
    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    Viper viper(&sys, ViperWheelType::RealWheel);
    viper.SetDriver(driver);
    viper.Initialize(ChFrame<>(ChVector3d(0, 0, 0.5), QUNIT));

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Viper Rover on Rigid Terrain");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(3, 3, 1));
            vis_irr->AddTypicalLights();
            vis_irr->EnableContactDrawing(ContactsDrawMode::CONTACT_DISTANCES);
            vis_irr->EnableShadows();
            vis = vis_irr;
#endif
            break;
        }
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->AddCamera(ChVector3d(3, 3, 1));
            vis_vsg->SetWindowTitle("Viper Rover on Rigid Terrain");
            vis_vsg->Initialize();
            vis = vis_vsg;
#endif
            break;
        }
        default:
            throw std::runtime_error("Failed to initialize a visualization method.");
    }

    // ------------ ROS: clock + Viper motor-control subscriber + rover state.
    auto ros_manager = chrono_types::make_shared<ChROSManager>();
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSClockHandler>());

    ros_manager->RegisterHandler(
        chrono_types::make_shared<ChROSViperDCMotorControlHandler>(25, driver, "~/input/driver_inputs"));

    ros_manager->RegisterHandler(
        chrono_types::make_shared<ChROSBodyHandler>(25, viper.GetChassis()->GetBody(), "~/output/rover/state"));

    ros_manager->Initialize();

    // ------------ Simulation loop

    double time_step = 1e-3;
    double time_end = 30;

#if !defined(CHRONO_IRRLICHT) && !defined(CHRONO_VSG)
    double time = 0;
    while (time < time_end) {
        time = sys.GetChTime();
#else
    while (vis->Run()) {
        double time = sys.GetChTime();
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
#endif
        viper.Update();
        if (!ros_manager->Update(time, time_step))
            break;

        sys.DoStepDynamics(time_step);
    }

    return 0;
}
