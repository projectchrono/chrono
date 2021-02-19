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
// Demo to show Curiosity Scarecrow testing rovers being operated on Rigid terrain
// Includes an obstacle to show Rocker-bogie suspension system
// Note: The obstacle has been lowered since Scarecrow has a smaller weight
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

// Use custom material for the Viper Wheel
bool use_custom_mat = false;

// Choose Curiosity rover chassis type
Chassis_Type chassis_type = Chassis_Type::Scarecrow;

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
double time_step = 0.0005;  

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Scarecrow on Rigid Terrain", core::dimension2d<u32>(1280, 720), false);

    // set gravity
    mphysicalSystem.Set_G_acc(ChVector<>(0, -9.81, 0));

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 2, -2));

    application.AddLightWithShadow(core::vector3df(2.5f, 7.0f, 0.0f), core::vector3df(0, 0, 0), 50, 4, 25, 130, 512,
                                   video::SColorf(0.8f, 0.8f, 1.0f));

    application.SetContactsDrawMode(ChIrrTools::eCh_ContactsDrawMode::CONTACT_DISTANCES);

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

    // Create a Viper Rover with default parameters.
    // The default rotational speed of the Motor is speed w=3.145 rad/sec.
    // Note: the Viper Rover uses a Z-up frame, which will need to be translated to Y-up.
    ChVector<double> body_pos(0, -0.2, 0);
    ChQuaternion<> body_rot = Q_from_AngX(-CH_C_PI / 2);

    std::shared_ptr<CuriosityRover> rover; 

    if (use_custom_mat == true) {
        // If use the customized wheel material
        rover = chrono_types::make_shared<CuriosityRover>(&mphysicalSystem, body_pos, body_rot, CustomWheelMaterial(ChContactMethod::NSC),chassis_type);
        rover->Initialize();
        std::cout<<"total mass:"<<rover->GetRoverMass()<<std::endl;
    } else {
        // If use default wheel material
        rover = rover = chrono_types::make_shared<CuriosityRover>(&mphysicalSystem, body_pos, body_rot,chassis_type);
        rover->Initialize();
        std::cout<<"total mass:"<<rover->GetRoverMass()<<std::endl;
    }

    // Create the first step of the stair-shaped obstacle
    auto mbox = chrono_types::make_shared<ChBodyEasyBox>(0.6, 0.15, 1, 1000, true, true, floor_mat);

    mbox->SetPos(ChVector<>(2, -0.45, 1));
    mbox->SetBodyFixed(true);
    mbox->SetCollide(true);
    mphysicalSystem.Add(mbox);


    // Create the second step of the stair-shaped obstacle
    auto mbox_2 = chrono_types::make_shared<ChBodyEasyBox>(0.6, 0.3, 1, 1000, true, true, floor_mat);
 
    mbox_2->SetPos(ChVector<>(2.3, -0.45, 1));
    mbox_2->SetBodyFixed(true);
    mbox_2->SetCollide(true);
    mphysicalSystem.Add(mbox_2);

    // Create the third step of the stair-shaped obstacle
    auto mbox_3 = chrono_types::make_shared<ChBodyEasyBox>(0.6, 0.4, 1, 1000, true, true, floor_mat);
 
    mbox_3->SetPos(ChVector<>(2.6, -0.45, 1));
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

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    while (application.GetDevice()->run()) {

        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        application.DoStep();

        application.EndScene();


    }

    return 0;
}