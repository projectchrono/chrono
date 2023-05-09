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
// Chrono::Multicore unit test for DVI contact Jacobians
//
// =============================================================================

#include "chrono_multicore/constraints/ChConstraintUtils.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChSystemDescriptorMulticore.h"

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "unit_testing.h"

#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

// Comment the following line to use multicore collision detection
//#define BULLET

using namespace chrono;
using namespace chrono::collision;

void CreateContainer(ChSystemMulticore* system) {
    auto mat_walls = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat_walls->SetFriction(0.3f);

    std::shared_ptr<ChBody> container(system->NewBody());
    container->SetBodyFixed(true);
    container->SetCollide(true);
    container->SetMass(10000.0);

    container->GetCollisionModel()->ClearModel();
    utils::AddBoxContainer(container, mat_walls,                   //
                           ChFrame<>(ChVector<>(0, 0, 1), QUNIT),  //
                           ChVector<>(2, 2, 2), 0.1,               //
                           ChVector<int>(2, 2, -1));
    container->GetCollisionModel()->BuildModel();

    system->AddBody(container);
}

void CreateGranularMaterial(ChSystemMulticore* sys) {
    // Common material
    auto ballMat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ballMat->SetFriction(.5);

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

void SetupSystem(ChSystemMulticoreNSC* sys) {
    sys->Set_G_acc(ChVector<>(0, 0, -9.81));

    // Solver settings
    int max_iteration_normal = 0;
    int max_iteration_sliding = 25;
    int max_iteration_spinning = 0;
    float contact_recovery_speed = 10e30f;
    double tolerance = 1e-2;

    sys->GetSettings()->solver.tolerance = tolerance;
    sys->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    sys->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    sys->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    sys->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    sys->GetSettings()->solver.alpha = 0;
    sys->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
    sys->ChangeSolverType(SolverType::APGD);
    sys->GetSettings()->collision.collision_envelope = 0.00;
    sys->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    sys->SetNumThreads(1);

    CreateContainer(sys);
    CreateGranularMaterial(sys);
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

void CompareContacts(ChMulticoreDataManager* data_manager,
                     const std::vector<real3>& pos_rigid,      // positions at begining of step
                     const std::vector<quaternion>& rot_rigid  // orientations at begining of step
) {
    const auto num_rigid_contacts = data_manager->cd_data->num_rigid_contacts;
    if (num_rigid_contacts == 0) {
        return;
    }

    real3* norm = data_manager->cd_data->norm_rigid_rigid.data();
    real3* ptA = data_manager->cd_data->cpta_rigid_rigid.data();
    real3* ptB = data_manager->cd_data->cptb_rigid_rigid.data();
    chrono::vec2* ids = data_manager->cd_data->bids_rigid_rigid.data();

    uint nnz_normal = 6 * 2 * num_rigid_contacts;
    uint nnz_tangential = 6 * 4 * num_rigid_contacts;
    // uint nnz_spinning = 6 * 3 * num_rigid_contacts;

    ASSERT_EQ((real)data_manager->host_data.D_T.nonZeros(), nnz_normal + nnz_tangential);

    for (uint index = 0; index < num_rigid_contacts; index++) {
        real3 U = norm[index], V, W;
        real3 T3, T4, T5, T6, T7, T8;
        real3 TA, TB, TC;
        real3 TD, TE, TF;

        Orthogonalize(U, V, W);

        chrono::vec2 body_id = ids[index];

        int row = index;

        // The position is subtracted here now instead of performing it in the narrowphase
        Compute_Jacobian(rot_rigid[body_id.x], U, V, W, ptA[index] - pos_rigid[body_id.x], T3, T4, T5);
        Compute_Jacobian(rot_rigid[body_id.y], U, V, W, ptB[index] - pos_rigid[body_id.y], T6, T7, T8);

        int off = data_manager->cd_data->num_rigid_contacts;

        ASSERT_EQ((real)data_manager->host_data.D_T(row * 1 + 0, body_id.x * 6 + 0), -U.x);
        ASSERT_EQ((real)data_manager->host_data.D_T(row * 1 + 0, body_id.x * 6 + 1), -U.y);
        ASSERT_EQ((real)data_manager->host_data.D_T(row * 1 + 0, body_id.x * 6 + 2), -U.z);

        ASSERT_EQ((real)data_manager->host_data.D_T(row * 1 + 0, body_id.x * 6 + 3), T3.x);
        ASSERT_EQ((real)data_manager->host_data.D_T(row * 1 + 0, body_id.x * 6 + 4), T3.y);
        ASSERT_EQ((real)data_manager->host_data.D_T(row * 1 + 0, body_id.x * 6 + 5), T3.z);

        ASSERT_EQ((real)data_manager->host_data.D_T(row * 1 + 0, body_id.y * 6 + 0), U.x);
        ASSERT_EQ((real)data_manager->host_data.D_T(row * 1 + 0, body_id.y * 6 + 1), U.y);
        ASSERT_EQ((real)data_manager->host_data.D_T(row * 1 + 0, body_id.y * 6 + 2), U.z);

        ASSERT_EQ((real)data_manager->host_data.D_T(row * 1 + 0, body_id.y * 6 + 3), -T6.x);
        ASSERT_EQ((real)data_manager->host_data.D_T(row * 1 + 0, body_id.y * 6 + 4), -T6.y);
        ASSERT_EQ((real)data_manager->host_data.D_T(row * 1 + 0, body_id.y * 6 + 5), -T6.z);

        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 0, body_id.x * 6 + 0), -V.x);
        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 0, body_id.x * 6 + 1), -V.y);
        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 0, body_id.x * 6 + 2), -V.z);

        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 0, body_id.x * 6 + 3), T4.x);
        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 0, body_id.x * 6 + 4), T4.y);
        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 0, body_id.x * 6 + 5), T4.z);

        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 1, body_id.x * 6 + 0), -W.x);
        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 1, body_id.x * 6 + 1), -W.y);
        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 1, body_id.x * 6 + 2), -W.z);

        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 1, body_id.x * 6 + 3), T5.x);
        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 1, body_id.x * 6 + 4), T5.y);
        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 1, body_id.x * 6 + 5), T5.z);

        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 0, body_id.y * 6 + 0), V.x);
        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 0, body_id.y * 6 + 1), V.y);
        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 0, body_id.y * 6 + 2), V.z);

        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 0, body_id.y * 6 + 3), -T7.x);
        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 0, body_id.y * 6 + 4), -T7.y);
        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 0, body_id.y * 6 + 5), -T7.z);

        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 1, body_id.y * 6 + 0), W.x);
        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 1, body_id.y * 6 + 1), W.y);
        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 1, body_id.y * 6 + 2), W.z);

        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 1, body_id.y * 6 + 3), -T8.x);
        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 1, body_id.y * 6 + 4), -T8.y);
        ASSERT_EQ((real)data_manager->host_data.D_T(off + row * 2 + 1, body_id.y * 6 + 5), -T8.z);
    }
}

TEST(ChronoMulticore, jacobians) {
    bool animate = false;

    ChSystemMulticoreNSC* sys = new ChSystemMulticoreNSC();
    sys->SetNumThreads(1);

#ifdef BULLET
    sys->SetCollisionSystemType(ChCollisionSystemType::BULLET);
#else
    sys->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
#endif

    SetupSystem(sys);

    // Initialize counters
    double time = 0;
    double time_end = 1.0;
    double time_step = 1e-3;

    sys->DoStepDynamics(time_step);

    if (animate) {
#ifdef CHRONO_OPENGL
        opengl::ChVisualSystemOpenGL vis;
        vis.AttachSystem(sys);
        vis.SetWindowTitle("Jacobians");
        vis.SetWindowSize(1280, 720);
        vis.SetRenderMode(opengl::WIREFRAME);
        vis.Initialize();
        vis.AddCamera(ChVector<>(6, -6, 1), ChVector<>(0, 0, 0));
        vis.SetCameraVertical(CameraVerticalDir::Z);

        // Loop until reaching the end time...
        while (time < time_end) {
            if (vis.Run()) {
                vis.Render();
            }
            auto pos_rigid = sys->data_manager->host_data.pos_rigid;
            auto rot_rigid = sys->data_manager->host_data.rot_rigid;
            sys->DoStepDynamics(time_step);
            CompareContacts(sys->data_manager, pos_rigid, rot_rigid);
            time += time_step;
        }
#else
        std::cout << "OpenGL support not available.  Cannot animate mechanism." << std::endl;
        return;
#endif
    } else {
        while (time < time_end) {
            auto pos_rigid = sys->data_manager->host_data.pos_rigid;
            auto rot_rigid = sys->data_manager->host_data.rot_rigid;
            sys->DoStepDynamics(time_step);
            CompareContacts(sys->data_manager, pos_rigid, rot_rigid);
            time += time_step;
        }
    }
}
