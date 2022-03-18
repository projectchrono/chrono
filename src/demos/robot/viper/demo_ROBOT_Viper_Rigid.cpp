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
// Demo to show Viper Rover operated on Rigid Terrain
// This Demo includes operation to spawn a Viper rover, control wheel speed, and
// control rover steering
//
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"
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
using namespace chrono::viper;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;

// Use custom material for the Viper Wheel
bool use_custom_mat = false;

// Define Viper rover wheel type
ViperWheelType wheel_type = ViperWheelType::RealWheel;

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
double time_step = 1e-3;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the Chrono system with gravity in the negative Z direction
    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Create the Irrlicht visualization
    ChIrrApp application(&sys, L"Viper Rover on Rigid Terrain", core::dimension2d<u32>(1280, 720), VerticalDir::Z,
                         false, false, true);
    application.AddLogo();
    application.AddSkyBox();
    application.AddLight(irr::core::vector3df(30.f, 30.f, 150.f), 290, irr::video::SColorf(0.7f, 0.7f, 0.7f, 1.0f));
    application.AddLight(irr::core::vector3df(-30.f, -30.f, 150.f), 190, irr::video::SColorf(0.7f, 0.8f, 0.8f, 1.0f));
    application.AddCamera(core::vector3df(3, 3, 1));
    application.SetContactsDrawMode(IrrContactsDrawMode::CONTACT_DISTANCES);

    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    // Create the ground.
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(30, 30, 1, 1000, true, true, ground_mat);

    ground->SetPos(ChVector<>(0, 0, -0.5));
    ground->SetBodyFixed(true);
    sys.Add(ground);

    auto texture = chrono_types::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("textures/concrete.jpg"));
    texture->SetTextureScale(60, 45);
    ground->AddAsset(texture);

    // Construct a Viper rover and the asociated driver
    ////auto driver = chrono_types::make_shared<ViperSpeedDriver>(1.0, 5.0);
    auto driver = chrono_types::make_shared<ViperDCMotorControl>();

    Viper viper(&sys, wheel_type);
    viper.SetDriver(driver);
    if (use_custom_mat)
        viper.SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));

    ////viper.SetChassisFixed(true);

    ////viper.SetChassisVisualization(false);
    ////viper.SetSuspensionVisualization(false);

    viper.Initialize(ChFrame<>(ChVector<>(0, 0, 0.5), QUNIT));

    std::cout << "Viper total mass: " << viper.GetRoverMass() << std::endl;
    std::cout << "  chassis:        " << viper.GetChassis()->GetBody()->GetMass() << std::endl;
    std::cout << "  upper arm:      " << viper.GetUpperArm(ViperWheelID::V_LF)->GetBody()->GetMass() << std::endl;
    std::cout << "  lower arm:      " << viper.GetLowerArm(ViperWheelID::V_LF)->GetBody()->GetMass() << std::endl;
    std::cout << "  upright:        " << viper.GetUpright(ViperWheelID::V_LF)->GetBody()->GetMass() << std::endl;
    std::cout << "  wheel:          " << viper.GetWheel(ViperWheelID::V_LF)->GetBody()->GetMass() << std::endl;
    std::cout << std::endl;

    // Complete construction of visual assets
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Use shadows in realtime view
    application.AddShadowAll();

    application.SetTimestep(time_step);

    // Simulation loop
    while (application.GetDevice()->run()) {
        // Set current steering angle
        double time = viper.GetSystem()->GetChTime();
        double max_steering = CH_C_PI / 6;
        double steering = 0;
        if (time > 2 && time < 7)
            steering = max_steering * (time - 2) / 5;
        else if (time > 7 && time < 12)
            steering = max_steering * (12 - time) / 5;
        driver->SetSteering(steering);

        ////double max_lifting = CH_C_PI / 8;
        ////double lifting = 0;
        ////if (time > 1 && time < 2)
        ////    lifting = max_lifting * (time - 1);
        ////else if (time > 2)
        ////    lifting = max_lifting;
        ////driver->SetLifting(lifting);

        // Update Viper controls
        viper.Update();

        // Read rover chassis velocity
        ////std::cout <<"Rover speed: " << viper.GetChassisVel() << std::endl;

        // Read rover chassis acceleration
        ////std::cout << "Rover acceleration: "<< viper.GetChassisAcc() << std::endl;

        application.BeginScene(true, true, SColor(255, 140, 161, 192));
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
