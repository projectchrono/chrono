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
//     - collisions and contacts
//     - using Irrlicht to display objects.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#ifdef CHRONO_COLLISION
    #include "chrono/collision/ChCollisionSystemChrono.h"
#endif

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;

collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::BULLET;

void AddFallingItems(ChSystemNSC& sys) {
    // Shared contact materials for falling objects
    auto sph_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    sph_mat->SetFriction(0.2f);
    auto box_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto cyl_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    // Create falling rigid bodies (spheres and boxes etc.)
    for (int bi = 0; bi < 29; bi++) {
        auto sphereBody = chrono_types::make_shared<ChBodyEasySphere>(1.1,      // radius size
                                                                      1000,     // density
                                                                      sph_mat,  // contact material
                                                                      collision_type);
        sphereBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        sphereBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
        sys.Add(sphereBody);

        auto boxBody = chrono_types::make_shared<ChBodyEasyBox>(1.5, 1.5, 1.5,  // x,y,z size
                                                                100,            // density
                                                                box_mat,        // contact material
                                                                collision_type);
        boxBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        boxBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/cubetexture_bluewhite.png"));
        sys.Add(boxBody);

        auto cylBody = chrono_types::make_shared<ChBodyEasyCylinder>(0.75, 0.5,  // radius, height
                                                                     100,        // density
                                                                     cyl_mat,    // contact material
                                                                     collision_type);
        cylBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        cylBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/pinkwhite.png"));
        sys.Add(cylBody);
    }
}

std::shared_ptr<ChBody> AddContainer(ChSystemNSC& sys) {
    // Contact and visualization materials for container
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto ground_mat_vis = chrono_types::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());
    ground_mat_vis->SetKdTexture(GetChronoDataFile("textures/concrete.jpg"));

    // Create the five walls of the rectangular container, using fixed rigid bodies of 'box' type
    auto floorBody = chrono_types::make_shared<ChBodyEasyBox>(20, 1, 20, 1000, ground_mat, collision_type);
    floorBody->SetPos(ChVector<>(0, -5, 0));
    floorBody->SetBodyFixed(true);
    floorBody->GetVisualShape(0)->SetMaterial(0, ground_mat_vis);
    sys.Add(floorBody);

    auto wallBody1 = chrono_types::make_shared<ChBodyEasyBox>(1, 10, 20.99, 1000, ground_mat, collision_type);
    wallBody1->SetPos(ChVector<>(-10, 0, 0));
    wallBody1->SetBodyFixed(true);
    wallBody1->GetVisualShape(0)->SetMaterial(0, ground_mat_vis);
    sys.Add(wallBody1);

    auto wallBody2 = chrono_types::make_shared<ChBodyEasyBox>(1, 10, 20.99, 1000, ground_mat, collision_type);
    wallBody2->SetPos(ChVector<>(10, 0, 0));
    wallBody2->SetBodyFixed(true);
    wallBody2->GetVisualShape(0)->SetMaterial(0, ground_mat_vis);
    sys.Add(wallBody2);

    auto wallBody3 = chrono_types::make_shared<ChBodyEasyBox>(20.99, 10, 1, 1000, ground_mat, collision_type);
    wallBody3->SetPos(ChVector<>(0, 0, -10));
    wallBody3->SetBodyFixed(true);
    wallBody3->GetVisualShape(0)->SetMaterial(0, ground_mat_vis);
    sys.Add(wallBody3);

    auto wallBody4 = chrono_types::make_shared<ChBodyEasyBox>(20.99, 10, 1, 1000, ground_mat, collision_type);
    wallBody4->SetPos(ChVector<>(0, 0, 10));
    wallBody4->SetBodyFixed(true);
    wallBody4->GetVisualShape(0)->SetMaterial(0, ground_mat_vis);
    sys.Add(wallBody4);

    // Add the rotating mixer
    auto mixer_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mixer_mat->SetFriction(0.4f);

    auto rotatingBody = chrono_types::make_shared<ChBodyEasyBox>(10, 5, 1,   // x,y,z size
                                                                 4000,       // density
                                                                 mixer_mat,  // contact material
                                                                 collision_type);
    rotatingBody->SetPos(ChVector<>(0, -1.6, 0));
    rotatingBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    sys.Add(rotatingBody);

    // .. a motor between mixer and truss

    auto my_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    my_motor->Initialize(rotatingBody, floorBody, ChFrame<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
    auto mfun = chrono_types::make_shared<ChFunction_Const>(CH_C_PI / 4.0);  // speed 45 deg/s
    my_motor->SetSpeedFunction(mfun);
    sys.AddLink(my_motor);

    /// NOTE: Instead of creating five separate 'box' bodies to make
    /// the walls of the container, you could have used a single body
    /// made of five box shapes, which build a single collision description,
    /// as in the alternative approach:

    /*
    // create a plain ChBody (no colliding shape nor visualization mesh is used yet)
    auto rigidBody = chrono_types::make_shared<ChBody>(collision_type);

    // set as fixed body, and turn collision ON, otherwise no collide by default
    rigidBody->SetBodyFixed(true);
    rigidBody->SetCollide(true);

    // Clear model. The colliding shape description MUST be between  ClearModel() .. BuildModel() pair.
    rigidBody->GetCollisionModel()->ClearModel();
    // Describe the (invisible) colliding shape by adding five boxes (the walls and floor)
    rigidBody->GetCollisionModel()->AddBox(ground_mat, 20, 1, 20, ChVector<>(0, -10, 0));
    rigidBody->GetCollisionModel()->AddBox(ground_mat, 1, 40, 20, ChVector<>(-11, 0, 0));
    rigidBody->GetCollisionModel()->AddBox(ground_mat, 1, 40, 20, ChVector<>(11, 0, 0));
    rigidBody->GetCollisionModel()->AddBox(ground_mat, 20, 40, 1, ChVector<>(0, 0, -11));
    rigidBody->GetCollisionModel()->AddBox(ground_mat, 20, 40, 1, ChVector<>(0, 0, 11));
    // Complete the description of collision shape.
    rigidBody->GetCollisionModel()->BuildModel();
    */

    return rotatingBody;
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the physical system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(collision_type);

    // Settings specific to Chrono multicore collision system
    if (collision_type == collision::ChCollisionSystemType::CHRONO) {
#ifdef CHRONO_COLLISION
        auto collsys = std::static_pointer_cast<collision::ChCollisionSystemChrono>(sys.GetCollisionSystem());
        // Change the default number of broadphase bins
        collsys->SetBroadphaseGridResolution(ChVector<int>(10, 10, 2));
        // Change default narrowphase algorithm
        ////collsys->SetNarrowphaseAlgorithm(collision::ChNarrowphase::Algorithm::MPR);
        collsys->SetEnvelope(0.005);
        // Enable active bounding box
        collsys->EnableActiveBoundingBox(ChVector<>(-10, -10, -20), ChVector<>(+10, +10, +10));
        // Set number of threads used by the collision detection system
        collsys->SetNumThreads(4);
#endif
    }

    // Add fixed and moving bodies
    auto mixer = AddContainer(sys);
    AddFallingItems(sys);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    sys.SetVisualSystem(vis);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("NSC collision demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 14, -20));
    vis->AddTypicalLights();

    // Modify some setting of the physical system for the simulation, if you want
    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.SetSolverMaxIterations(50);
    ////sys.SetUseSleeping(true);

    // Simulation loop
    ChRealtimeStepTimer rt;
    double step_size = 0.003;

    while (vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();
        vis->EndScene();

        sys.DoStepDynamics(step_size);

        ////std::cout << std::endl;
        ////auto frc = mixer->GetAppliedForce();
        ////auto trq = mixer->GetAppliedTorque();
        ////std::cout << sys.GetChTime() << "  force: " << frc << "  torque: " << trq << std::endl;
        ////auto c_frc = mixer->GetContactForce();
        ////auto c_trq = mixer->GetContactTorque();
        ////std::cout << sys.GetChTime() << "  ct force: " << c_frc << "  ct torque: " << c_trq << std::endl;

        rt.Spin(step_size);
    }

    return 0;
}
