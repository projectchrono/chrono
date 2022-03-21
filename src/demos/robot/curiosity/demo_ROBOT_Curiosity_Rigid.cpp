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
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChBoxShape.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::curiosity;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;

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
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    // Create a ChronoENGINE physical system
    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&sys, L"Curiosity Rover on Rigid Terrain", core::dimension2d<u32>(1280, 720), VerticalDir::Z);
    application.AddLogo();
    application.AddSkyBox();
    application.AddTypicalLights();
    application.AddCamera(core::vector3df(3, 3, 1));
    application.AddLightWithShadow(core::vector3df(2.5f, 7.0f, 0.0f), core::vector3df(0, 0, 0), 50, 4, 25, 130, 512,
                                   video::SColorf(0.8f, 0.8f, 0.8f));
    application.SetContactsDrawMode(IrrContactsDrawMode::CONTACT_DISTANCES);
    application.SetTimestep(time_step);

    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

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

    rover.Initialize(ChFrame<>(ChVector<double>(0, 0, 0.2), QUNIT));

    ////rover.FixChassis(true);
    ////rover.FixSuspension(true);

    std::cout << "Curiosity total mass: " << rover.GetRoverMass() << std::endl;
    std::cout << "  chassis:            " << rover.GetChassis()->GetBody()->GetMass() << std::endl;
    std::cout << "  wheel:              " << rover.GetWheel(CuriosityWheelID::C_LF)->GetBody()->GetMass() << std::endl;
    std::cout << std::endl;

    // Complete visual asset construction
    application.AssetBindAll();
    application.AssetUpdateAll();
    application.AddShadowAll();

    // Simulation loop
    while (application.GetDevice()->run()) {
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

        application.BeginScene(true, true, SColor(255, 140, 161, 192));
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }

    return 0;
}

void CreateTerrain(ChSystem& sys) {
    // Create the ground and obstacles
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(30, 30, 1, 1000, true, true, ground_mat);

    ground->SetPos(ChVector<>(0, 0, -0.5));
    ground->SetBodyFixed(true);
    sys.Add(ground);

    auto texture = chrono_types::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("textures/concrete.jpg"));
    texture->SetTextureScale(60, 45);
    ground->AddAsset(texture);

    // Create the first step of the stair-shaped obstacle
    auto mbox_1 = chrono_types::make_shared<ChBodyEasyBox>(2.4, 1.4, 0.1, 1000, true, true, ground_mat);
    mbox_1->SetPos(ChVector<>(3, 1, 0.05));
    mbox_1->SetBodyFixed(true);
    mbox_1->SetCollide(true);
    sys.Add(mbox_1);

    // Create the second step of the stair-shaped obstacle
    auto mbox_2 = chrono_types::make_shared<ChBodyEasyBox>(1.6, 1.2, 0.2, 1000, true, true, ground_mat);
    mbox_2->SetPos(ChVector<>(3, 1, 0.1));
    mbox_2->SetBodyFixed(true);
    mbox_2->SetCollide(true);
    sys.Add(mbox_2);

    // Create the third step of the stair-shaped obstacle
    auto mbox_3 = chrono_types::make_shared<ChBodyEasyBox>(0.8, 1.0, 0.3, 1000, true, true, ground_mat);
    mbox_3->SetPos(ChVector<>(3, 1, 0.15));
    mbox_3->SetBodyFixed(true);
    mbox_3->SetCollide(true);
    sys.Add(mbox_3);
}