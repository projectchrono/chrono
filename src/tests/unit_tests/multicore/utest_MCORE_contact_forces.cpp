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
// Author: Radu Serban
// =============================================================================
//
// Unit test for calculation of cumulative contact forces on a body.
// The test checks that the cumulative contact force on a container body (fixed
// to ground) is equal to the sum of the weights of several bodies dropped in
// the container.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"

#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChVisualSystemOpenGL.h"
//#define USE_OPENGL
#endif

#include "../ut_utils.h"

using namespace chrono;

class ContactForceTest : public ::testing::TestWithParam<ChContactMethod> {
  protected:
    ContactForceTest();
    ~ContactForceTest() { delete sys; }

    ChSystemMulticore* sys;
    std::shared_ptr<ChBody> ground;
    double total_weight;
};

ContactForceTest::ContactForceTest() : sys(nullptr) {
    bool use_mat_properties = false;
    ChSystemSMC::ContactForceModel force_model = ChSystemSMC::Hooke;
    ChSystemSMC::TangentialDisplacementModel tdispl_model = ChSystemSMC::OneStep;

    float young_modulus = 2e4f;
    float friction = 0.4f;
    float restitution = 0;
    float adhesion = 0;

    float kn = 2e4;
    float gn = 5e2;
    float kt = 0;
    float gt = 0;

    std::shared_ptr<ChContactMaterial> material;

    switch (GetParam()) {
        case ChContactMethod::SMC: {
            ChSystemMulticoreSMC* sysSMC = new ChSystemMulticoreSMC;
            sysSMC->GetSettings()->solver.contact_force_model = force_model;
            sysSMC->GetSettings()->solver.tangential_displ_mode = tdispl_model;
            sysSMC->GetSettings()->solver.use_material_properties = use_mat_properties;
            sys = sysSMC;

            auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
            mat->SetYoungModulus(young_modulus);
            mat->SetRestitution(restitution);
            mat->SetFriction(friction);
            mat->SetAdhesion(adhesion);
            mat->SetKn(kn);
            mat->SetGn(gn);
            mat->SetKt(kt);
            mat->SetGt(gt);
            material = mat;

            break;
        }
        case ChContactMethod::NSC: {
            ChSystemMulticoreNSC* sysNSC = new ChSystemMulticoreNSC;
            sysNSC->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
            sysNSC->GetSettings()->solver.max_iteration_normal = 0;
            sysNSC->GetSettings()->solver.max_iteration_sliding = 100;
            sysNSC->GetSettings()->solver.max_iteration_spinning = 0;
            sysNSC->ChangeSolverType(SolverType::APGD);
            sys = sysNSC;

            auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
            mat->SetRestitution(restitution);
            mat->SetFriction(friction);
            material = mat;

            break;
        }

        default:
            break;
    }

    // Set associated collision detection system
    sys->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    // Set other sys properties
    double gravity = -9.81;
    sys->SetGravitationalAcceleration(ChVector3d(0, 0, gravity));
    sys->GetSettings()->solver.tolerance = 1e-5;
    sys->GetSettings()->solver.max_iteration_bilateral = 100;
    sys->GetSettings()->solver.clamp_bilaterals = false;
    sys->GetSettings()->solver.bilateral_clamp_speed = 1000;

    // Create the falling balls
    unsigned int num_balls = 8;
    double radius = 0.5;
    double mass = 5;
    ChVector3d pos(0, 0, 1.1 * radius);
    ChQuaternion<> rot(1, 0, 0, 0);
    ChVector3d init_vel(0, 0, 0);
    ChVector3d init_omg(0, 0, 0);

    std::vector<std::shared_ptr<ChBody>> balls(num_balls);
    total_weight = 0;
    for (unsigned int i = 0; i < num_balls; i++) {
        auto ball = chrono_types::make_shared<ChBody>();

        ball->SetMass(mass);
        ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector3d(1, 1, 1));
        ball->SetPos(pos + ChVector3d(i * 2 * radius, i * 2 * radius, 0));
        ball->SetRot(rot);
        ball->SetPosDt(init_vel);
        ball->SetAngVelParent(init_omg);
        ball->EnableCollision(true);
        ball->SetFixed(false);

        auto ct_shape = chrono_types::make_shared<ChCollisionShapeSphere>(material, radius);
        ball->AddCollisionShape(ct_shape);

        auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(radius);
        sphere->SetColor(ChColor(1, 0, 1));
        ball->AddVisualShape(sphere);

        sys->AddBody(ball);
        balls[i] = ball;

        total_weight += ball->GetMass();
    }
    total_weight *= gravity;
    ////std::cout << "Total weight = " << total_weight << std::endl;

    // Create container box
    ground = utils::CreateBoxContainer(sys, material, ChVector3d(20, 20, 2 * radius), 0.1);
}

TEST_P(ContactForceTest, simulate) {
    double end_time = 3.0;    // total simulation time
    double start_time = 0.5;  // start check after this period
    double time_step = 1e-3;

    double rtol = 1e-3;  // validation relative error

    while (sys->GetChTime() < end_time) {
#ifdef USE_OPENGL
        opengl::ChVisualSystemOpenGL vis;
        vis.AttachSystem(sys);
        vis.SetWindowTitle("");
        vis.SetWindowSize(1200, 800);
        vis.SetRenderMode(opengl::WIREFRAME);
        vis.Initialize();
        vis.AddCamera(ChVector3d(20, 0, 0), ChVector3d(0, 0, 0));
        vis.SetCameraVertical(CameraVerticalDir::Z);

        if (!vis.Run())
            break;
        sys->DoStepDynamics(time_step);
        vis.Render();
#else
        sys->DoStepDynamics(time_step);
#endif

        sys->GetContactContainer()->ComputeContactForces();
        ChVector3d contact_force = ground->GetContactForce();
        ////std::cout << "t = " << sys->GetChTime() << " num contacts = " << sys->GetNumContacts()
        ////          << "  force =  " << contact_force.z() << std::endl;

        if (sys->GetChTime() > start_time) {
            ASSERT_LT(std::abs(1 - contact_force.z() / total_weight), rtol);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(ChronoMulticore,
                         ContactForceTest,
                         ::testing::Values(ChContactMethod::NSC, ChContactMethod::SMC));
