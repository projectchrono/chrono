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
// Authors: Jason Zhou
// =============================================================================
//
// Demo to show Curiosity rovering on rigid terrain
// The rover will cross an obstacle to show Rocker-bogie suspension system
//
// =============================================================================

#include "chrono_models/robot/curiosity/Curiosity.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkDistance.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::curiosity;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;

// Use custom material for the Curiosity Wheel
bool use_custom_mat = false;

// Specify rover chassis type
// The options are Chassis_Type::Scarecrow and Chassis_Type::FullRover
Chassis_Type chassis_type = Chassis_Type::FullRover;

// Specify rover wheel type
// The options are Wheel_Type::RealWheel, Wheel_Type::SimpleWheel, and Wheel_Type::CylWheel
Wheel_Type wheel_type = Wheel_Type::RealWheel;

// Helper function to create custom wheel material for the rover wheel body
std::shared_ptr<ChMaterialSurface> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.2f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChMaterialSurface>();
    }
}

// Simulation time step
double time_step = 2e-3;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;
    mphysicalSystem.Set_G_acc(ChVector<>(0, -9.81, 0));

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Curiosity Rover on Rigid Terrain", core::dimension2d<u32>(1280, 720),
                         VerticalDir::Y);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 2, -2));
    application.AddLightWithShadow(core::vector3df(2.5f, 7.0f, 0.0f), core::vector3df(0, 0, 0), 50, 4, 25, 130, 512,
                                   video::SColorf(0.8f, 0.8f, 1.0f));
    application.SetContactsDrawMode(IrrContactsDrawMode::CONTACT_DISTANCES);

    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    // Create a floor
    auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto mfloor = chrono_types::make_shared<ChBodyEasyBox>(20, 1, 20, 1000, true, true, floor_mat);

    mfloor->SetPos(ChVector<>(0, -1, 0));
    mfloor->SetBodyFixed(true);
    mphysicalSystem.Add(mfloor);

    auto masset_texture = chrono_types::make_shared<ChTexture>();
    masset_texture->SetTextureFilename(GetChronoDataFile("textures/concrete.jpg"));
    mfloor->AddAsset(masset_texture);

    // Create a Curiosity Rover with default parameters.
    ChVector<double> body_pos(0, -0.2, 0);
    ChQuaternion<> body_rot = Q_from_AngX(-CH_C_PI / 2);

    std::shared_ptr<CuriosityRover> rover;

    if (use_custom_mat == true) {
        // If use the customized wheel material
        rover = chrono_types::make_shared<CuriosityRover>(
            &mphysicalSystem, body_pos, body_rot, CustomWheelMaterial(ChContactMethod::NSC), chassis_type, wheel_type);
        // Set to use linear DC motor model for the drive motors
        rover->SetDCControl(true);
        rover->Initialize();

        // Display the mass of the rover
        std::cout << "total mass:" << rover->GetRoverMass() << std::endl;
    } else {
        // If use default wheel material
        rover = rover =
            chrono_types::make_shared<CuriosityRover>(&mphysicalSystem, body_pos, body_rot, chassis_type, wheel_type);
        // Set to use linear DC motor model for the drive motors
        rover->SetDCControl(true);
        rover->Initialize();

        // Display the mass of the rover
        std::cout << "total mass:" << rover->GetRoverMass() << std::endl;
    }

    // Create the first step of the stair-shaped obstacle
    auto mbox = chrono_types::make_shared<ChBodyEasyBox>(0.6, 0.3, 1, 1000, true, true, floor_mat);

    mbox->SetPos(ChVector<>(3, -0.4, 1));
    mbox->SetBodyFixed(true);
    mbox->SetCollide(true);
    mphysicalSystem.Add(mbox);

    // Create the second step of the stair-shaped obstacle
    auto mbox_2 = chrono_types::make_shared<ChBodyEasyBox>(0.6, 0.6, 1, 1000, true, true, floor_mat);

    mbox_2->SetPos(ChVector<>(3.3, -0.4, 1));
    mbox_2->SetBodyFixed(true);
    mbox_2->SetCollide(true);
    mphysicalSystem.Add(mbox_2);

    // Create the third step of the stair-shaped obstacle
    auto mbox_3 = chrono_types::make_shared<ChBodyEasyBox>(0.6, 0.8, 1, 1000, true, true, floor_mat);

    mbox_3->SetPos(ChVector<>(3.6, -0.4, 1));
    mbox_3->SetBodyFixed(true);
    mbox_3->SetCollide(true);
    mphysicalSystem.Add(mbox_3);

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    // Use shadows in realtime view
    application.AddShadowAll();

    application.SetTimestep(time_step);

    // Simulation loop
    while (application.GetDevice()->run()) {
        rover->Update();

        // Read rover chassis velocity
        // std::cout <<"Rover Chassis Speedo Reading: " << rover -> GetChassisVel() << std::endl;

        // Read rover chassis acceleration
        // std::cout << "Rover Chassis Accelerometer Reading: "<< rover -> GetChassisAcc() << std::endl;

        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }

    return 0;
}