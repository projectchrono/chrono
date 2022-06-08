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
// Authors: Rainer Gericke (VSG), Alessandro Tasora (Irrlicht Version)
// =============================================================================
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::vsg3d;

// Create a bunch of ChronoENGINE rigid bodies that
// represent bricks in a large wall.

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

    // Create the floor using
    // fixed rigid body of 'box' type:

    auto mrigidFloor = chrono_types::make_shared<ChBodyEasyBox>(250, 4, 250,  // x,y,z size
            1000,         // density
            true,         // visulization?
            true,         // collision?
            mat);         // contact material
    mrigidFloor->SetPos(ChVector<>(0, -2, 0));
    mrigidFloor->SetBodyFixed(true);
    mrigidFloor->GetVisualShape(0)->SetTexture("textures/concrete.jpg");

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

// Create a bunch of ChronoENGINE rigid bodies that
// represent bricks in a Jenga tower

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

    // Create the floor using
    // fixed rigid body of 'box' type:
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
    GetLog() << "Copyright (c) 2022 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC sys;

    // Create all the rigid bodies.
    create_wall_bodies(sys);
    // create_jengatower_bodies (sys);

    // Create the VSG visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    sys.SetVisualSystem(vis);
    vis->SetCameraVertical(vsg3d::CameraVerticalDir::Y);
    vis->SetWindowSize(ChVector2<int>(800, 600));
    vis->SetWindowPosition(ChVector2<int>(100, 300));
    vis->SetWindowTitle("Brick Wall Test");
    vis->SetUseSkyBox(true);
    vis->SetLightIntensity(0.9);
    vis->SetLightDirection(1.5*CH_C_PI_2, CH_C_PI_4);
    vis->Initialize();

    // Prepare the physical system for the simulation
    auto solver = chrono_types::make_shared<ChSolverPSOR>();
    solver->SetMaxIterations(40);
    solver->EnableWarmStart(true);
    sys.SetSolver(solver);

    // sys.SetUseSleeping(true);
    sys.SetMaxPenetrationRecoverySpeed(1.0);

    // Simulation loop
    double timestep = 0.02;
    while(vis->Run()){
        vis->Render();
        sys.DoStepDynamics(timestep);
    }
    GetLog() << "Terminated at simulation time " << sys.GetChTime() << " secs.\n";

    return 0;
}