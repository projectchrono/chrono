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
#include "chrono/assets/ChSphereShape.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
//#define USE_OPENGL
#endif

#include "unit_testing.h"

using namespace chrono;

class ContactForceTest : public ::testing::TestWithParam<ChContactMethod> {
  protected:
    ContactForceTest();
    ~ContactForceTest() { delete system; }

    ChSystemMulticore* system;
    std::shared_ptr<ChBody> ground;
    double total_weight;
};

ContactForceTest::ContactForceTest() {
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

    std::shared_ptr<ChMaterialSurface> material;
    switch (GetParam()) {
        case ChContactMethod::SMC: {
            ChSystemMulticoreSMC* sys = new ChSystemMulticoreSMC;
            sys->GetSettings()->solver.contact_force_model = force_model;
            sys->GetSettings()->solver.tangential_displ_mode = tdispl_model;
            sys->GetSettings()->solver.use_material_properties = use_mat_properties;
            system = sys;

            auto mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
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
            ChSystemMulticoreNSC* sys = new ChSystemMulticoreNSC;
            sys->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
            sys->GetSettings()->solver.max_iteration_normal = 0;
            sys->GetSettings()->solver.max_iteration_sliding = 100;
            sys->GetSettings()->solver.max_iteration_spinning = 0;
            sys->ChangeSolverType(SolverType::APGD);
            system = sys;

            auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            mat->SetRestitution(restitution);
            mat->SetFriction(friction);
            material = mat;

            break;
        }
    }

    // Set other system properties
    double gravity = -9.81;
    system->Set_G_acc(ChVector<>(0, gravity, 0));
    system->GetSettings()->solver.tolerance = 1e-5;
    system->GetSettings()->solver.max_iteration_bilateral = 100;
    system->GetSettings()->solver.clamp_bilaterals = false;
    system->GetSettings()->solver.bilateral_clamp_speed = 1000;

    // Create the falling balls
    unsigned int num_balls = 8;
    double radius = 0.5;
    double mass = 5;
    ChVector<> pos(0, 1.1 * radius, 0);
    ChQuaternion<> rot(1, 0, 0, 0);
    ChVector<> init_vel(0, 0, 0);
    ChVector<> init_omg(0, 0, 0);

    std::vector<std::shared_ptr<ChBody>> balls(num_balls);
    total_weight = 0;
    for (unsigned int i = 0; i < num_balls; i++) {
        auto ball = std::shared_ptr<ChBody>(system->NewBody());

        ball->SetMass(mass);
        ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector<>(1, 1, 1));
        ball->SetPos(pos + ChVector<>(i * 2 * radius, 0, i * 2 * radius));
        ball->SetRot(rot);
        ball->SetPos_dt(init_vel);
        ball->SetWvel_par(init_omg);
        ball->SetCollide(true);
        ball->SetBodyFixed(false);

        ball->GetCollisionModel()->ClearModel();
        ball->GetCollisionModel()->AddSphere(material, radius);
        ball->GetCollisionModel()->BuildModel();

        auto sphere = chrono_types::make_shared<ChSphereShape>();
        sphere->GetSphereGeometry().rad = radius;
        sphere->SetColor(ChColor(1, 0, 1));
        ball->AddAsset(sphere);

        system->AddBody(ball);
        balls[i] = ball;

        total_weight += ball->GetMass();
    }
    total_weight *= gravity;
    ////std::cout << "Total weight = " << total_weight << std::endl;

    // Create container box
    ground = utils::CreateBoxContainer(system, 0, material, ChVector<>(20, 20, 2 * radius), 0.1, ChVector<>(0, 0, 0),
                                       ChQuaternion<>(1, 0, 0, 0), true, true, false, false);

    // Create the OpenGL visualization window
#ifdef USE_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1200, 800, "", system);
    gl_window.SetCamera(ChVector<>(20, 0, 0), ChVector<>(0, 0, 0), ChVector<>(0, 1, 0), (float)radius,
                        (float)radius);
    gl_window.SetRenderMode(opengl::WIREFRAME);
#endif
}

TEST_P(ContactForceTest, simulate) {
    double end_time = 3.0;    // total simulation time
    double start_time = 0.5;  // start check after this period
    double time_step = 1e-3;

    double rtol = 1e-3;  // validation relative error

    while (system->GetChTime() < end_time) {
#ifdef USE_OPENGL
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        if (!gl_window.Active())
            break;
        gl_window.DoStepDynamics(time_step);
        gl_window.Render();
#else
        system->DoStepDynamics(time_step);
#endif

        system->GetContactContainer()->ComputeContactForces();
        ChVector<> contact_force = ground->GetContactForce();
        ////std::cout << "t = " << system->GetChTime() << " num contacts = " << system->GetNumContacts()
        ////          << "  force =  " << contact_force.y() << std::endl;

        if (system->GetChTime() > start_time) {
            ASSERT_LT(std::abs(1 - contact_force.y() / total_weight), rtol);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(ChronoMulticore,
                         ContactForceTest,
                         ::testing::Values(ChContactMethod::NSC, ChContactMethod::SMC));
