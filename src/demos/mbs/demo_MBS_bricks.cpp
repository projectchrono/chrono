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

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Create a bunch of rigid bodies that represent bricks in a large wall.
void create_wall_bodies(ChSystemNSC& sys) {
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
                mrigidBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/cubetexture_borders.png"));
                sys.Add(mrigidBody);
            }
        }
    }

    // Create the floor using fixed rigid body of 'box' type:
    auto mrigidFloor = chrono_types::make_shared<ChBodyEasyBox>(250, 4, 250,  // x,y,z size
                                                                1000,         // density
                                                                true,         // visulization?
                                                                true,         // collision?
                                                                mat);         // contact material
    mrigidFloor->SetPos(ChVector<>(0, -2, 0));
    mrigidFloor->SetBodyFixed(true);

    sys.Add(mrigidFloor);

    // Create a ball that will collide with wall
    auto mrigidBall = chrono_types::make_shared<ChBodyEasySphere>(4,     // radius
                                                                  8000,  // density
                                                                  true,  // visualization?
                                                                  true,  // collision?
                                                                  mat);  // contact material
    mrigidBall->SetPos(ChVector<>(0, -2, 0));
    mrigidBall->SetPos(ChVector<>(0, 3, -8));
    mrigidBall->SetPos_dt(ChVector<>(0, 0, 16));  // set initial speed
    mrigidBall->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
    sys.Add(mrigidBall);
}

// Create a bunch of ChronoENGINE rigid bodies that represent bricks in a Jenga tower
void create_jengatower_bodies(ChSystemNSC& sys) {
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
        sys.Add(mrigidBody1);

        auto mrigidBody2 = chrono_types::make_shared<ChBodyEasyBox>(2, 2, 14,  // x,y,z size
                                                                    100,       // density
                                                                    true,      // visualization?
                                                                    true,      // collision?
                                                                    mat);      // contact material
        mrigidBody2->SetPos(ChVector<>(5, 1.0 + bi * 2.0, 0));
        sys.Add(mrigidBody2);

        auto mrigidBody3 = chrono_types::make_shared<ChBodyEasyBox>(14, 2, 2,  // x,y,z size
                                                                    100,       // density
                                                                    true,      // visualization?
                                                                    true,      // collision?
                                                                    mat);      // contact material
        mrigidBody3->SetPos(ChVector<>(0, 3.0 + bi * 2.0, 5));
        sys.Add(mrigidBody3);

        auto mrigidBody4 = chrono_types::make_shared<ChBodyEasyBox>(14, 2, 2,  // x,y,z size
                                                                    100,       // density
                                                                    true,      // visualization?
                                                                    true,      // collision?
                                                                    mat);      // contact material
        mrigidBody4->SetPos(ChVector<>(0, 3.0 + bi * 2.0, -5));
        sys.Add(mrigidBody4);
    }

    // Create the floor using fixed rigid body of 'box' type:
    auto mrigidFloor = chrono_types::make_shared<ChBodyEasyBox>(250, 4, 250,  // x,y,z size
                                                                1000,         // density
                                                                true,         // visualization?
                                                                true,         // collision?
                                                                mat);         // contact material
    mrigidFloor->SetPos(ChVector<>(0, -2, 0));
    mrigidFloor->SetBodyFixed(true);

    sys.Add(mrigidFloor);

    // Create a ball that will collide with tower
    auto mrigidBall = chrono_types::make_shared<ChBodyEasySphere>(4,     // radius
                                                                  1000,  // density
                                                                  true,  // visualization?
                                                                  true,  // collision?
                                                                  mat);  // contact material
    mrigidBall->SetPos(ChVector<>(0, 3, -8));
    mrigidBall->SetPos_dt(ChVector<>(0, 0, 2));  // set initial speed
    mrigidBall->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
    sys.Add(mrigidBall);
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the physical system
    ChSystemNSC sys;

    // Create all the rigid bodies.
    create_wall_bodies(sys);
    ////create_jengatower_bodies (sys);

    // Create the run-time visualization system
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
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Bricks test");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddLight(ChVector<>(70, 120, -90), 290, ChColor(0.7f, 0.7f, 0.7f));
            vis_irr->AddLight(ChVector<>(30, 80, 60), 190, ChColor(0.7f, 0.8f, 0.8f));
            vis_irr->AddCamera(ChVector<>(-15, 14, -30), ChVector<>(0, 5, 0));

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetWindowSize(ChVector2<int>(800, 600));
            vis_vsg->SetWindowPosition(ChVector2<int>(100, 100));
            vis_vsg->SetWindowTitle("VSG Bricks Demo");
            vis_vsg->SetClearColor(ChColor(0.8f, 0.85f, 0.9f));
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->AddCamera(ChVector<>(-30, 28, -60), ChVector<>(0, 5, 0));
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Prepare the physical system for the simulation
    auto solver = chrono_types::make_shared<ChSolverPSOR>();
    solver->SetMaxIterations(40);
    solver->EnableWarmStart(true);
    sys.SetSolver(solver);

    // sys.SetUseSleeping(true);
    sys.SetMaxPenetrationRecoverySpeed(1.0);

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(0.02);
    }

    return 0;
}
