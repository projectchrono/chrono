// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jason Zhou, Radu Serban
// =============================================================================
//
// Demo to show Curiosity rovering on rigid terrain
// The rover will cross an obstacle to show Rocker-bogie suspension system
//
// =============================================================================

#include "chrono_models/robot/curiosity/Curiosity.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChTexture.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/input_output/ChWriterCSV.h"
#include "chrono/assets/ChVisualShapeBox.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

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
using namespace chrono::curiosity;

// -----------------------------------------------------------------------------

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Specify rover chassis type
// The options are Scarecrow and FullRover
CuriosityChassisType chassis_type = CuriosityChassisType::FullRover;

// Specify rover wheel type
// The options are RealWheel, SimpleWheel, and CylWheel
CuriosityWheelType wheel_type = CuriosityWheelType::RealWheel;

// Simulation time step
double time_step = 1e-3;

// -----------------------------------------------------------------------------

void CreateTerrain(ChSystem& sys);

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;
    // Create a Chrono physical system
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    // Create terrain and obstacles
    CreateTerrain(sys);

    // Create a Curiosity rover and the asociated driver
    ////auto driver = chrono_types::make_shared<CuriositySpeedDriver>(1.0, 5.0);
    auto driver = chrono_types::make_shared<CuriosityDCMotorControl>();

    Curiosity rover(&sys, chassis_type, wheel_type);
    rover.SetDriver(driver);

    ////rover.SetChassisVisualization(false);
    ////rover.SetWheelVisualization(false);
    ////rover.SetSuspensionVisualization(false);

    rover.Initialize(ChFrame<>(ChVector3d(0, 0, 0.2), QUNIT));

    ////rover.FixChassis(true);
    ////rover.FixSuspension(true);

    std::cout << "Curiosity total mass: " << rover.GetRoverMass() << std::endl;
    std::cout << "  chassis:            " << rover.GetChassis()->GetBody()->GetMass() << std::endl;
    std::cout << "  wheel:              " << rover.GetWheel(CuriosityWheelID::C_LF)->GetBody()->GetMass() << std::endl;
    std::cout << std::endl;

    // Create the run-time visualization interface
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
            vis_irr->SetWindowTitle("Curiosity Rover on Rigid Terrain");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(3, 3, 1));
            vis_irr->AddTypicalLights();
            vis_irr->AddLightWithShadow(ChVector3d(2.5, 7.0, 0.0), ChVector3d(0, 0, 0), 50, 4, 25, 130, 512,
                                        ChColor(0.8f, 0.8f, 0.8f));
            vis_irr->EnableContactDrawing(ContactsDrawMode::CONTACT_DISTANCES);
            vis_irr->EnableShadows();

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowTitle("Curiosity Rover on Rigid Terrain");
            vis_vsg->AddCamera(ChVector3d(5, 5, 1));
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        ////auto time = rover.GetSystem()->GetChTime();
        ////if (time < 1)
        ////    driver->SetSteering(0);
        ////else
        ////    driver->SetSteering((time - 1) * 0.2);

        // Update Curiosity controls
        rover.Update();

        // Read rover chassis velocity
        ////std::cout <<"Rover speed: " << rover.GetChassisVel() << std::endl;

        // Read rover chassis acceleration
        ////std::cout << "Rover acceleration: "<< rover.GetChassisAcc() << std::endl;

        sys.DoStepDynamics(time_step);
    }

    return 0;
}

void CreateTerrain(ChSystem& sys) {
    // Create the ground and obstacles
    auto ground_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(30, 30, 1, 1000, true, true, ground_mat);
    ground->SetPos(ChVector3d(0, 0, -0.5));
    ground->SetFixed(true);
    ground->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"), 60, 45);
    sys.Add(ground);

    // Create the first step of the stair-shaped obstacle
    auto mbox_1 = chrono_types::make_shared<ChBodyEasyBox>(2.4, 1.4, 0.1, 1000, true, true, ground_mat);
    mbox_1->SetPos(ChVector3d(3, 1, 0.05));
    mbox_1->SetFixed(true);
    mbox_1->EnableCollision(true);
    sys.Add(mbox_1);

    // Create the second step of the stair-shaped obstacle
    auto mbox_2 = chrono_types::make_shared<ChBodyEasyBox>(1.6, 1.2, 0.2, 1000, true, true, ground_mat);
    mbox_2->SetPos(ChVector3d(3, 1, 0.1));
    mbox_2->SetFixed(true);
    mbox_2->EnableCollision(true);
    sys.Add(mbox_2);

    // Create the third step of the stair-shaped obstacle
    auto mbox_3 = chrono_types::make_shared<ChBodyEasyBox>(0.8, 1.0, 0.3, 1000, true, true, ground_mat);
    mbox_3->SetPos(ChVector3d(3, 1, 0.15));
    mbox_3->SetFixed(true);
    mbox_3->EnableCollision(true);
    sys.Add(mbox_3);
}