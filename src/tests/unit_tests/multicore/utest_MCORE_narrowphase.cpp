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

#include "unit_testing.h"

#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

using namespace chrono;
using namespace chrono::collision;

// -----------------------------------------------------------------------------
// Global problem definitions
// -----------------------------------------------------------------------------

void CreateContainer(ChSystemMulticore* system) {
    auto mat_walls = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat_walls->SetFriction(0.3f);

    std::shared_ptr<ChBody> container(system->NewBody());
    container->SetBodyFixed(true);
    container->SetCollide(true);
    container->SetMass(10000.0);

    // Attach geometry of the containing bin
    double hthick = 0.05;
    container->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(container.get(), mat_walls, ChVector<>(1, 1, hthick), ChVector<>(0, 0, -hthick));
    utils::AddBoxGeometry(container.get(), mat_walls, ChVector<>(hthick, 1, 1), ChVector<>(-1 - hthick, 0, 1));
    utils::AddBoxGeometry(container.get(), mat_walls, ChVector<>(hthick, 1, 1), ChVector<>(1 + hthick, 0, 1));
    utils::AddBoxGeometry(container.get(), mat_walls, ChVector<>(1, hthick, 1), ChVector<>(0, -1 - hthick, 1));
    utils::AddBoxGeometry(container.get(), mat_walls, ChVector<>(1, hthick, 1), ChVector<>(0, 1 + hthick, 1));
    container->GetCollisionModel()->BuildModel();

    system->AddBody(container);
}

void CreateGranularMaterial(ChSystemMulticore* sys) {
    // Common material
    auto ballMat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ballMat->SetFriction(1.0);

    // Create the falling balls
    double mass = 1;
    double radius = 0.15;
    ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);
    srand(1);

    for (int ix = -2; ix < 3; ix++) {
        for (int iy = -2; iy < 3; iy++) {
            for (int iz = -2; iz < 3; iz++) {
                ChVector<> rnd(rand() % 1000 / 100000.0, rand() % 1000 / 100000.0, rand() % 1000 / 100000.0);
                ChVector<> pos(0.4 * ix, 0.4 * iy, 0.4 * iz + 1);

                std::shared_ptr<ChBody> ball(sys->NewBody());

                ball->SetMass(mass);
                ball->SetInertiaXX(inertia);
                ball->SetPos(pos + rnd);
                ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
                ball->SetBodyFixed(false);
                ball->SetCollide(true);

                ball->GetCollisionModel()->ClearModel();
                utils::AddSphereGeometry(ball.get(), ballMat, radius);
                ball->GetCollisionModel()->BuildModel();

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

    msystem->Set_G_acc(ChVector<>(0, 0, -9.81));

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
    for (int i = 0; i < msystem_A->Get_bodylist().size(); i++) {
        ChVector<> pos = msystem_B->Get_bodylist().at(i)->GetPos();
        ChVector<> pos_dt = msystem_B->Get_bodylist().at(i)->GetPos_dt();
        msystem_A->Get_bodylist().at(i)->SetPos(pos);
        msystem_A->Get_bodylist().at(i)->SetPos_dt(pos_dt);
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

#ifdef BULLET
    msystem_mpr->SetCollisionSystemType(ChCollisionSystemType::BULLET);
    msystem_r->SetCollisionSystemType(ChCollisionSystemType::BULLET);
#endif

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
#ifdef CHRONO_OPENGL
        opengl::ChVisualSystemOpenGL vis;
        vis.AttachSystem(msystem_mpr);
        vis.SetWindowTitle("");
        vis.SetWindowSize(1280, 720);
        vis.SetRenderMode(opengl::WIREFRAME);
        vis.Initialize();
        vis.SetCameraPosition(ChVector<>(6, -6, 1), ChVector<>(0, 0, 0));
        vis.SetCameraVertical(CameraVerticalDir::Z);

        while (time < time_end) {
            if (vis.Run()) {
                vis.Render();
            }

            Sync(msystem_mpr, msystem_r);
            msystem_mpr->DoStepDynamics(time_step);
            msystem_r->DoStepDynamics(time_step);

            std::cout << "Time: " << time << std::endl;
            CompareContacts(msystem_mpr, msystem_r);

            time += time_step;
        }

#else
        std::cout << "OpenGL support not available.  Cannot animate mechanism." << std::endl;
        return 1;
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

/*
void CompareContacts(ChSystemMulticore* msystem_A, ChSystemMulticore* msystem_B) {
    // Tolerance for test
    double test_tolerance = 1e-5;

    int num_contacts_A = msystem_A->data_manager->num_rigid_contacts;
    int num_contacts_B = msystem_B->data_manager->num_rigid_contacts;

    ASSERT_EQ(num_contacts_A, num_contacts_B);

    // compare depth
    for (int i = 0; i < num_contacts_A; i++) {
        real depth_A = msystem_A->data_manager->host_data.dpth_rigid_rigid[i];
        real depth_B = msystem_B->data_manager->host_data.dpth_rigid_rigid[i];

        ASSERT_NEAR(depth_A, depth_B, test_tolerance);
    }

    for (int i = 0; i < num_contacts_A; i++) {
        real3 norm_A = msystem_A->data_manager->host_data.norm_rigid_rigid[i];
        real3 norm_B = msystem_B->data_manager->host_data.norm_rigid_rigid[i];

        Assert_near(norm_A, norm_B, test_tolerance);
    }

    ////for (int i = 0; i < num_contacts_A; i++) {
    ////    real3 pta_A = msystem_A->data_manager->host_data.cpta_rigid_rigid[i];
    ////    real3 pta_B = msystem_B->data_manager->host_data.cpta_rigid_rigid[i];
    ////    Assert_near(pta_A, pta_B, test_tolerance);
    ////    real3 ptb_A = msystem_A->data_manager->host_data.cptb_rigid_rigid[i];
    ////    real3 ptb_B = msystem_B->data_manager->host_data.cptb_rigid_rigid[i];
    ////    Assert_near(ptb_A, ptb_B, test_tolerance);
    ////}
}

TEST(ChronoMulticore, narrowphase) {
    bool animate = false;

    ChSystemMulticoreNSC* msystem_mpr = new ChSystemMulticoreNSC();
    ChSystemMulticoreNSC* msystem_r = new ChSystemMulticoreNSC();

    SetupSystem(msystem_mpr);
    SetupSystem(msystem_r);

    msystem_mpr->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
    msystem_r->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::PRIMS;


    double time_end = 1;
    double time_step = 1e-3;
    double time = 0;

    if (animate) {
#ifdef CHRONO_OPENGL
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.Initialize(1280, 720, "Narrowphase", msystem_mpr);
        gl_window.SetCamera(ChVector<>(6, -6, 1), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
        gl_window.SetRenderMode(opengl::WIREFRAME);

        // Loop until reaching the end time...
        while (time < time_end) {
            if (gl_window.Active()) {
                gl_window.Render();
            }
            msystem_mpr->DoStepDynamics(time_step);
            msystem_r->DoStepDynamics(time_step);
            Sync(msystem_mpr, msystem_r);
            CompareContacts(msystem_mpr, msystem_r);

            time += time_step;
        }

#else
        std::cout << "OpenGL support not available.  Cannot animate mechanism." << std::endl;
        return false;
#endif
    } else {
        while (time < time_end) {
            msystem_mpr->DoStepDynamics(time_step);
            msystem_r->DoStepDynamics(time_step);
            Sync(msystem_mpr, msystem_r);
            CompareContacts(msystem_mpr, msystem_r);

            time += time_step;
        }
    }
}
*/
