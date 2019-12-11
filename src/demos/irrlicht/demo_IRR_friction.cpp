// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about
// - using rolling friction (not only sliding and static friction, available in
//   all objects by default)
// - optional sharing of some assets (visualization stuff)
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_irrlicht/ChIrrApp.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;


int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Contacts with rolling friction", core::dimension2d<u32>(800, 600), false,
                         true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 14, -20));


    // Create all the rigid bodies.

    double mradius = 0.5;
    double density = 1000;

    // Create a texture asset. It can be shared between bodies.
    auto textureasset = chrono_types::make_shared<ChTexture>(GetChronoDataFile("bluwhite.png"));

    // Create some spheres that roll horizontally,
    // with increasing rolling friction values
    for (int bi = 0; bi < 10; bi++) {
        double initial_angspeed = 10;
        double initial_linspeed = initial_angspeed * mradius;

        auto msphereBody = chrono_types::make_shared<ChBodyEasySphere>(mradius,           // radius size
                                                              1000,              // density
                                                              true,              // collide enable?
                                                              true);             // visualization?
        // Set some properties
        msphereBody->SetPos(ChVector<>(-7, mradius - 0.5, -5 + bi * mradius * 2.5));
        msphereBody->GetMaterialSurfaceNSC()->SetFriction(0.4f);
        msphereBody->AddAsset(textureasset);  // assets can be shared

        // Set initial speed: rolling in horizontal direction,
        msphereBody->SetWvel_par(ChVector<>(0, 0, -initial_angspeed));
        msphereBody->SetPos_dt(ChVector<>(initial_linspeed, 0, 0));

        // Set a non zero value of rolling friction to have a rolling resisting torque:
        msphereBody->GetMaterialSurfaceNSC()->SetRollingFriction(((float)bi / 10) * 0.05f);

        // Add to the system
        mphysicalSystem.Add(msphereBody);
    }

    // Create some spheres that spin on place, for a 'drilling friction' case,
    // with increasing spinning friction values
    for (int bi = 0; bi < 10; bi++) {
        auto msphereBody = chrono_types::make_shared<ChBodyEasySphere>(mradius,           // radius size
            1000,              // density
            true,              // collide enable?
            true);             // visualization?
        // Set some properties
        msphereBody->SetPos(ChVector<>(-8, 1 + mradius - 0.5, -5 + bi * mradius * 2.5));
        msphereBody->GetMaterialSurfaceNSC()->SetFriction(0.4f);
        msphereBody->AddAsset(textureasset);  // assets can be shared

        // Set initial speed: spinning in vertical direction
        msphereBody->SetWvel_par(ChVector<>(0, 20, 0));

        // Set a non zero value of spinning friction that brakes the spinning on vertical axis
        // of the contact:
        msphereBody->GetMaterialSurfaceNSC()->SetSpinningFriction(((float)bi / 10) * 0.02f);

        // Add to the system
        mphysicalSystem.Add(msphereBody);

        // Notes:
        // - setting nonzero spinning friction and/or setting nonzero rolling friction
        //   affects the speed of the solver (each contact eats 2x of CPU time repsect to the
        //   case of simple sliding/staic contact)
        // - avoid using zero spinning friction with nonzero rolling friction.
    }

    // Create the five walls of the rectangular container, using
    // fixed rigid bodies of 'box' type:

    // floor:

    auto mfloorBody = chrono_types::make_shared<ChBodyEasyBox>(20, 1, 20,  // x,y,z size
                                                      2000,       // density
                                                      true,       // collide enable?
                                                      true);      // visualization?
    mfloorBody->SetPos(ChVector<>(0, -1, 0));
    mfloorBody->SetBodyFixed(true);
    mfloorBody->GetMaterialSurfaceNSC()->SetRollingFriction(
        1);  // the min. of the two coeff. of the two contact surfaces will be used
    mfloorBody->GetMaterialSurfaceNSC()->SetSpinningFriction(
        1);  // the min. of the two coeff. of the two contact surfaces will be used

    mfloorBody->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("blu.png")));

    mphysicalSystem.Add(mfloorBody);

    // four walls:

    auto mwallBody1 = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 20.99,  // x,y,z size
        2000,         // density
        true,         // collide enable?
        true);       // visualization?
    mwallBody1->SetPos(ChVector<>(-10, 0, 0));
    mwallBody1->SetBodyFixed(true);
    mwallBody1->AddAsset(chrono_types::make_shared<ChColorAsset>(0.6f, 0.3f, 0.0f));
    mphysicalSystem.Add(mwallBody1);

    auto mwallBody2 = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 20.99,  // x,y,z size
        2000,         // density
        true,         // collide enable?
        true);       // visualization?
    mwallBody2->SetPos(ChVector<>(10, 0, 0));
    mwallBody2->SetBodyFixed(true);
    mwallBody2->AddAsset(chrono_types::make_shared<ChColorAsset>(0.6f, 0.3f, 0.0f));
    mphysicalSystem.Add(mwallBody2);

    auto mwallBody3 = chrono_types::make_shared<ChBodyEasyBox>(20.99, 2, 1,  // x,y,z size
        2000,         // density
        true,         // collide enable?
        true);       // visualization?
    mwallBody3->SetPos(ChVector<>(0, 0, -10));
    mwallBody3->SetBodyFixed(true);
    mwallBody3->AddAsset(chrono_types::make_shared<ChColorAsset>(0.6f, 0.3f, 0.0f));
    mphysicalSystem.Add(mwallBody3);

    auto mwallBody4 = chrono_types::make_shared<ChBodyEasyBox>(20.99, 2, 1,  // x,y,z size
        2000,         // density
        true,         // collide enable?
        true);       // visualization?
    mwallBody4->SetPos(ChVector<>(0, 0, 10));
    mwallBody4->SetBodyFixed(true);
    mwallBody4->AddAsset(chrono_types::make_shared<ChColorAsset>(0.6f, 0.3f, 0.0f));
    mphysicalSystem.Add(mwallBody4);


    // Use this function for adding a ChIrrNodeAsset to all already created items.
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Modify some setting of the physical system for the simulation
    mphysicalSystem.SetSolverType(ChSolver::Type::APGD);
    mphysicalSystem.SetSolverMaxIterations(100);

    // Simulation loop

    application.SetStepManage(true);
    application.SetTimestep(0.01);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
