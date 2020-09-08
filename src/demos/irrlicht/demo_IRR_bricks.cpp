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
//   Demo code about
//     - collisions and contacts
//     - sharing a ChMaterialSurfaceNSC property between bodies
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/assets/ChTexture.h"

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

// Create a bunch of ChronoENGINE rigid bodies that
// represent bricks in a large wall.

void create_wall_bodies(ChSystemNSC& mphysicalSystem) {
    // Create a material that will be shared among all collision shapes
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0.4f);
    mat->SetCompliance(0.0);
    mat->SetComplianceT(0.0);
    mat->SetDampingF(0.2f);

    // Create bricks
    for (int ai = 0; ai < 1; ai++) {           // N. of walls
        for (int bi = 0; bi < 10; bi++) {      // N. of vert. bricks
            for (int ui = 0; ui < 15; ui++) {  // N. of hor. bricks

                auto mrigidBody = chrono_types::make_shared<ChBodyEasyBox>(3.96, 2, 4,  // x,y,z size
                                                                           100,         // density
                                                                           true,        // visualization?
                                                                           true,        // collision?
                                                                           mat);        // contact material
                mrigidBody->SetPos(ChVector<>(-8 + ui * 4.0 + 2 * (bi % 2), 1.0 + bi * 2.0, ai * 9));

                mphysicalSystem.Add(mrigidBody);

                // optional, attach a texture for better visualization
                auto mtexture = chrono_types::make_shared<ChTexture>();
                mtexture->SetTextureFilename(GetChronoDataFile("cubetexture_borders.png"));
                mrigidBody->AddAsset(mtexture);
            }
        }
    }

    // Create the floor using
    // fixed rigid body of 'box' type:

    auto mrigidFloor = chrono_types::make_shared<ChBodyEasyBox>(250, 4, 250,  // x,y,z size
                                                                1000,         // density
                                                                true,         // visulization?
                                                                true,         // collision?
                                                                mat);         // contact material
    mrigidFloor->SetPos(ChVector<>(0, -2, 0));
    mrigidFloor->SetBodyFixed(true);

    mphysicalSystem.Add(mrigidFloor);

    // Create a ball that will collide with wall
    auto mrigidBall = chrono_types::make_shared<ChBodyEasySphere>(4,     // radius
                                                                  8000,  // density
                                                                  true,  // visualization?
                                                                  true,  // collision?
                                                                  mat);  // contact material
    mrigidBall->SetPos(ChVector<>(0, -2, 0));
    mrigidBall->SetPos(ChVector<>(0, 3, -8));
    mrigidBall->SetPos_dt(ChVector<>(0, 0, 16));  // set initial speed

    mphysicalSystem.Add(mrigidBall);

    // optional, attach a texture for better visualization
    auto mtextureball = chrono_types::make_shared<ChTexture>();
    mtextureball->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
    mrigidBall->AddAsset(mtextureball);
}

// Create a bunch of ChronoENGINE rigid bodies that
// represent bricks in a Jenga tower

void create_jengatower_bodies(ChSystemNSC& mphysicalSystem) {
    // Create a material that will be shared among all collision shapes
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0.4f);
    mat->SetCompliance(0.0);
    mat->SetComplianceT(0.0);
    mat->SetDampingF(0.2f);

    // Create bricks
    for (int bi = 0; bi < 12; bi += 2) {
        auto mrigidBody1 = chrono_types::make_shared<ChBodyEasyBox>(2, 2, 14,  // x,y,z size
                                                                    100,       // density
                                                                    true,      // visualization?
                                                                    true,      // collision?
                                                                    mat);      // contact material
        mrigidBody1->SetPos(ChVector<>(-5, 1.0 + bi * 2.0, 0));
        mphysicalSystem.Add(mrigidBody1);

        auto mrigidBody2 = chrono_types::make_shared<ChBodyEasyBox>(2, 2, 14,  // x,y,z size
                                                                    100,       // density
                                                                    true,      // visualization?
                                                                    true,      // collision?
                                                                    mat);      // contact material
        mrigidBody2->SetPos(ChVector<>(5, 1.0 + bi * 2.0, 0));
        mphysicalSystem.Add(mrigidBody2);

        auto mrigidBody3 = chrono_types::make_shared<ChBodyEasyBox>(14, 2, 2,  // x,y,z size
                                                                    100,       // density
                                                                    true,      // visualization?
                                                                    true,      // collision?
                                                                    mat);      // contact material
        mrigidBody3->SetPos(ChVector<>(0, 3.0 + bi * 2.0, 5));
        mphysicalSystem.Add(mrigidBody3);

        auto mrigidBody4 = chrono_types::make_shared<ChBodyEasyBox>(14, 2, 2,  // x,y,z size
                                                                    100,       // density
                                                                    true,      // visualization?
                                                                    true,      // collision?
                                                                    mat);      // contact material
        mrigidBody4->SetPos(ChVector<>(0, 3.0 + bi * 2.0, -5));
        mphysicalSystem.Add(mrigidBody4);
    }

    // Create the floor using
    // fixed rigid body of 'box' type:
    auto mrigidFloor = chrono_types::make_shared<ChBodyEasyBox>(250, 4, 250,  // x,y,z size
                                                                1000,         // density
                                                                true,         // visualization?
                                                                true,         // collision?
                                                                mat);         // contact material
    mrigidFloor->SetPos(ChVector<>(0, -2, 0));
    mrigidFloor->SetBodyFixed(true);

    mphysicalSystem.Add(mrigidFloor);

    // Create a ball that will collide with tower
    auto mrigidBall = chrono_types::make_shared<ChBodyEasySphere>(4,     // radius
                                                                  1000,  // density
                                                                  true,  // visualization?
                                                                  true,  // collision?
                                                                  mat);  // contact material
    mrigidBall->SetPos(ChVector<>(0, 3, -8));
    mrigidBall->SetPos_dt(ChVector<>(0, 0, 2));  // set initial speed

    mphysicalSystem.Add(mrigidBall);

    // optional, attach a texture for better visualization
    auto mtextureball = chrono_types::make_shared<ChTexture>();
    mtextureball->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
    mrigidBall->AddAsset(mtextureball);
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Bricks test", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice(), core::vector3df(70.f, 120.f, -90.f),
                                    core::vector3df(30.f, 80.f, 60.f), 290, 190);
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(-15, 14, -30), core::vector3df(0, 5, 0));

    //
    // HERE YOU POPULATE THE MECHANICAL SYSTEM OF CHRONO...
    //

    // Create all the rigid bodies.
    create_wall_bodies(mphysicalSystem);
    // create_jengatower_bodies (mphysicalSystem);

    // Use this function for adding a ChIrrNodeAsset to all items
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' into Irrlicht meshes the assets
    // into Irrlicht-visualizable meshes
    application.AssetUpdateAll();

    // Prepare the physical system for the simulation

    auto solver = chrono_types::make_shared<ChSolverPSOR>();
    solver->SetMaxIterations(40);
    solver->EnableWarmStart(true);
    mphysicalSystem.SetSolver(solver);

    // mphysicalSystem.SetUseSleeping(true);
    mphysicalSystem.SetMaxPenetrationRecoverySpeed(1.0);

    // Simulation loop

    application.SetTimestep(0.02);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        ChIrrTools::drawGrid(application.GetVideoDriver(), 5, 5, 20, 20,
                             ChCoordsys<>(ChVector<>(0, 0.04, 0), Q_from_AngAxis(CH_C_PI / 2, VECT_X)),
                             video::SColor(50, 90, 90, 150), true);

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
