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
Wheel_Type wheel_type = Wheel_Type::RealWheel;

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
                         false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights(irr::core::vector3df(30.f, 30.f, 100.f), irr::core::vector3df(30.f, -30.f, 100.f));
    application.AddTypicalCamera(core::vector3df(0, 2, 2));
    application.AddLightWithShadow(core::vector3df(1.5f, 1.5f, 5.5f), core::vector3df(0, 0, 0), 3, 4, 10, 40, 512,
                                   video::SColorf(0.8f, 0.8f, 1.0f));
    application.SetContactsDrawMode(IrrContactsDrawMode::CONTACT_DISTANCES);

    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    // Create the ground.
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(20, 20, 1, 1000, true, true, ground_mat);

    ground->SetPos(ChVector<>(0, 0, -1));
    ground->SetBodyFixed(true);
    sys.Add(ground);

    auto masset_texture = chrono_types::make_shared<ChTexture>();
    masset_texture->SetTextureFilename(GetChronoDataFile("textures/concrete.jpg"));
    ground->AddAsset(masset_texture);

    // Construct and initialize a Viper rover.
    // The default rotational speed of the Motor is speed w=3.145 rad/sec.
    auto viper = chrono_types::make_shared<ViperRover>(&sys, wheel_type);
    if (use_custom_mat)
        viper->SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));
    ////viper->SetChassisFixed(true);
    ////viper->SetChassisVisualization(false);
    ////viper->SetSuspensionVisualization(false);
    viper->SetDCControl(true);

    ChVector<double> body_pos(0, 0, -0.2);
    viper->Initialize(ChFrame<>(body_pos, QUNIT));

    // Complete construction of visual assets
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Use shadows in realtime view
    application.AddShadowAll();

    application.SetTimestep(time_step);

    // Simulation loop
    double time = 0.0;
    while (application.GetDevice()->run()) {
        // Once the Viper rover turning angle returns back to 0, HOLD the steering
        if ((viper->GetTurnAngle() - 0) < 1e-8 && viper->GetTurnState() == TurnSig::RIGHT) {
            viper->SetTurn(TurnSig::HOLD);
        }

        if (abs(time - 1.0) < 1e-5) {
            viper->SetTurn(TurnSig::LEFT, CH_C_PI / 8);
        } else if (abs(time - 7.0) < 1e-5) {
            viper->SetTurn(TurnSig::RIGHT, CH_C_PI / 8);
        }

        // Note: Viper steering control needs to be updated every simulation loop
        viper->Update();

        // Display turning angle - ranges from -pi/3 to pi/3
        ////std::cout << "turn angle: " << viper->GetTurnAngle() << std::endl;

        // Read rover chassis velocity
        ////std::cout <<"Rover Chassis Speedo Reading: " << viper -> GetChassisVel() << std::endl;

        // Read rover chassis acceleration
        ////std::cout << "Rover Chassis Accelerometer Reading: "<< viper -> GetChassisAcc() << std::endl;

        application.BeginScene(true, true, SColor(255, 140, 161, 192));
        application.DrawAll();
        application.DoStep();
        application.EndScene();

        time = time + time_step;
    }

    return 0;
}
