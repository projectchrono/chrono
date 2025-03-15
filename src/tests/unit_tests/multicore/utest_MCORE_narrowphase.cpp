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
// Author: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Chrono::Multicore unit test to compare solutions from different narrowphase
// algorithms.
//
// =============================================================================

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChSystemDescriptorMulticore.h"

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "../ut_utils.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;

// -----------------------------------------------------------------------------
// Global problem definitions
// -----------------------------------------------------------------------------

void CreateContainer(ChSystemMulticore* system) {
    auto mat_walls = chrono_types::make_shared<ChContactMaterialNSC>();
    mat_walls->SetFriction(0.3f);

    auto container = chrono_types::make_shared<ChBody>();
    container->SetFixed(true);
    container->EnableCollision(true);
    container->SetMass(10000.0);

    // Attach geometry of the containing bin
    utils::AddBoxContainer(container, mat_walls,                   //
                           ChFrame<>(ChVector3d(0, 0, 1), QUNIT),  //
                           ChVector3d(2, 2, 2), 0.1,               //
                           ChVector3i(2, 2, -1));

    system->AddBody(container);
}

void CreateGranularMaterial(ChSystemMulticore* sys) {
    // Common material
    auto ballMat = chrono_types::make_shared<ChContactMaterialNSC>();
    ballMat->SetFriction(1.0);

    // Create the falling balls
    double mass = 1;
    double radius = 0.15;
    ChVector3d inertia = (2.0 / 5.0) * mass * radius * radius * ChVector3d(1, 1, 1);
    srand(1);

    for (int ix = -2; ix < 3; ix++) {
        for (int iy = -2; iy < 3; iy++) {
            for (int iz = -2; iz < 3; iz++) {
                ChVector3d rnd(rand() % 1000 / 100000.0, rand() % 1000 / 100000.0, rand() % 1000 / 100000.0);
                ChVector3d pos(0.4 * ix, 0.4 * iy, 0.4 * iz + 1);

                auto ball = chrono_types::make_shared<ChBody>();

                ball->SetMass(mass);
                ball->SetInertiaXX(inertia);
                ball->SetPos(pos + rnd);
                ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
                ball->SetFixed(false);
                ball->EnableCollision(true);

                utils::AddSphereGeometry(ball.get(), ballMat, radius);

                sys->AddBody(ball);
            }
        }
    }
}

void SetupSystem(ChSystemMulticoreNSC* msystem) {
    // Solver settings
    int max_iteration_normal = 0;
    int max_iteration_sliding = 25;
    int max_iteration_spinning = 0;
    float contact_recovery_speed = 10e30f;
    double tolerance = 1e-2;

    msystem->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    msystem->GetSettings()->solver.tolerance = tolerance;
    msystem->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    msystem->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    msystem->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    msystem->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    msystem->GetSettings()->solver.alpha = 0;
    msystem->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    msystem->SetMaxPenetrationRecoverySpeed(contact_recovery_speed);
    msystem->ChangeSolverType(SolverType::APGD);
    msystem->GetSettings()->collision.collision_envelope = 0;
    msystem->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    msystem->SetNumThreads(1);

    CreateContainer(msystem);
    CreateGranularMaterial(msystem);
}

// Sync the positions and velocities of the rigid bodies
void Sync(ChSystemMulticore* msystem_A, ChSystemMulticore* msystem_B) {
    for (int i = 0; i < msystem_A->GetBodies().size(); i++) {
        ChVector3d pos = msystem_B->GetBodies().at(i)->GetPos();
        ChVector3d pos_dt = msystem_B->GetBodies().at(i)->GetPosDt();
        msystem_A->GetBodies().at(i)->SetPos(pos);
        msystem_A->GetBodies().at(i)->SetPosDt(pos_dt);
    }
}

bool CompareContacts(ChSystemMulticore* msystem_A, ChSystemMulticore* msystem_B) {
    // Tolerance for test
    double test_tolerance = 1e-5;

    bool passing = true;
    int num_contacts_A = msystem_A->data_manager->cd_data->num_rigid_contacts;
    int num_contacts_B = msystem_B->data_manager->cd_data->num_rigid_contacts;

    if (num_contacts_A != num_contacts_B) {
        std::cout << num_contacts_A << " does not equal " << num_contacts_B << std::endl;
        passing = false;
    }

    // compare depth
    if (passing) {
        for (int i = 0; i < num_contacts_A; i++) {
            real depth_A = msystem_A->data_manager->cd_data->dpth_rigid_rigid[i];
            real depth_B = msystem_B->data_manager->cd_data->dpth_rigid_rigid[i];

            if (fabs(depth_A - depth_B) > test_tolerance) {
                std::cout << depth_A << " does not equal " << depth_B << " " << fabs(depth_A - depth_B) << std::endl;
                passing = false;
            }
        }
    }
    if (passing) {
        for (int i = 0; i < num_contacts_A; i++) {
            real3 norm_A = msystem_A->data_manager->cd_data->norm_rigid_rigid[i];
            real3 norm_B = msystem_B->data_manager->cd_data->norm_rigid_rigid[i];

            real x = norm_A.x;
            real y = norm_B.x;
            if (fabs(x - y) > test_tolerance) {
                std::cout << x << " does not equal " << y << " " << fabs(x - y) << std::endl;
                passing = false;
            }
            x = norm_A.y;
            y = norm_B.y;
            if (fabs(x - y) > test_tolerance) {
                std::cout << x << " does not equal " << y << " " << fabs(x - y) << std::endl;
                passing = false;
            }
            x = norm_A.z;
            y = norm_B.z;
            if (fabs(x - y) > test_tolerance) {
                std::cout << x << " does not equal " << y << " " << fabs(x - y) << std::endl;
                passing = false;
            }
        }
    }
    //  if (!passing) {
    //    std::cout << "MPR:" << std::endl;
    //    for (int i = 0; i < num_contacts_A; i++) {
    //      vec2 id = msystem_A->data_manager->host_data.bids_rigid_rigid[i];
    //      real depth = msystem_A->data_manager->host_data.dpth_rigid_rigid[i];
    //      std::cout << id.x << " " << id.y << " " << depth << std::endl;
    //    }
    //    std::cout << "R:" << std::endl;
    //    for (int i = 0; i < num_contacts_B; i++) {
    //      vec2 id = msystem_B->data_manager->host_data.bids_rigid_rigid[i];
    //      real depth = msystem_B->data_manager->host_data.dpth_rigid_rigid[i];
    //      std::cout << id.x << " " << id.y << " " << depth << std::endl;
    //    }
    //
    //    exit(1);
    //  }

    return true;
}

int main(int argc, char* argv[]) {
    // No animation by default (i.e. when no program arguments)
    bool animate = (argc > 1);

    ChSystemMulticoreNSC* msystem_mpr = new ChSystemMulticoreNSC();
    ChSystemMulticoreNSC* msystem_r = new ChSystemMulticoreNSC();

    msystem_mpr->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);
    msystem_r->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    SetupSystem(msystem_mpr);
    SetupSystem(msystem_r);

    // Edit system settings

    msystem_mpr->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
    msystem_r->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::PRIMS;

    // Initialize counters
    double time = 0;
    double time_step = 1e-3;
    double time_end = 1;

    if (animate) {
#ifdef CHRONO_VSG
        auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
        vis->AttachSystem(msystem_mpr);
        vis->SetWindowTitle("Unit test");
        vis->SetCameraVertical(CameraVerticalDir::Z);
        vis->AddCamera(ChVector3d(6, -6, 1), ChVector3d(0, 0, 0));
        vis->SetWindowSize(1280, 720);
        vis->SetClearColor(ChColor(0.8f, 0.85f, 0.9f));
        vis->SetUseSkyBox(true);
        vis->SetCameraAngleDeg(40.0);
        vis->SetLightIntensity(1.0f);
        vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
        vis->SetShadows(true);
        vis->SetWireFrameMode(true);
        vis->Initialize();

        while (time < time_end) {
            if (vis->Run()) {
                vis->Render();
            }

            Sync(msystem_mpr, msystem_r);
            msystem_mpr->DoStepDynamics(time_step);
            msystem_r->DoStepDynamics(time_step);

            std::cout << "Time: " << time << std::endl;
            CompareContacts(msystem_mpr, msystem_r);

            time += time_step;
        }
#else
        std::cout << "Run-time visualization not available.  Cannot animate mechanism." << std::endl;
        FAIL();
#endif
    } else {
        while (time < time_end) {
            Sync(msystem_mpr, msystem_r);
            msystem_mpr->DoStepDynamics(time_step);
            msystem_r->DoStepDynamics(time_step);

            std::cout << "Time: " << time << std::endl;
            CompareContacts(msystem_mpr, msystem_r);

            time += time_step;
        }
    }

    return 0;
}
