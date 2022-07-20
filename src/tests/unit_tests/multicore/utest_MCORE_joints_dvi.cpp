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
// Authors: Radu Serban
// =============================================================================
//
// Test for bilateral joint constraints in a NSC sys.
//
// The mechanism consists of three bodies (ground, sled, and pendulum) with a
// prismatic joint between ground and sled and a revolute joint between sled and
// pendulum.
// The sys is simulated with different combinations of solver settings
// (type of solver, solver mode, maximum number of iterations).  Constraint
// violations are monitored and verified.
//
// =============================================================================

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono/ChConfig.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/utils/ChUtilsCreators.h"

#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

#include "unit_testing.h"

using namespace chrono;
using namespace chrono::collision;

struct Options {
    SolverMode mode;
    SolverType type;

    uint max_iter_bilateral;
    uint max_iter_normal;
    uint max_iter_sliding;
};

class JointsDVI : public ::testing::TestWithParam<Options> {
  protected:
    JointsDVI() : animate(false) {
        opts = GetParam();

        // Additional solver settings
        double tolerance = 1e-5;

        bool clamp_bilaterals = false;
        double bilateral_clamp_speed = 1000;

        // Problem parameters
        double init_vel = 2;

        // Create the mechanical sys
        sys = new ChSystemMulticoreNSC();
        sys->Set_G_acc(ChVector<>(0, 0, -9.81));

        // Set number of threads
        sys->SetNumThreads(1);

        // Edit sys settings
        sys->GetSettings()->solver.tolerance = tolerance;
        sys->GetSettings()->solver.max_iteration_bilateral = opts.max_iter_bilateral;
        sys->GetSettings()->solver.clamp_bilaterals = clamp_bilaterals;
        sys->GetSettings()->solver.bilateral_clamp_speed = bilateral_clamp_speed;

        sys->GetSettings()->solver.solver_mode = opts.mode;
        sys->GetSettings()->solver.max_iteration_normal = opts.max_iter_normal;
        sys->GetSettings()->solver.max_iteration_sliding = opts.max_iter_sliding;
        sys->GetSettings()->solver.max_iteration_spinning = 0;
        sys->GetSettings()->solver.alpha = 0;
        sys->GetSettings()->solver.contact_recovery_speed = 1e4;
        sys->ChangeSolverType(opts.type);

        // Create the ground body
        auto ground = std::shared_ptr<ChBody>(sys->NewBody());
        ground->SetIdentifier(-1);
        ground->SetBodyFixed(true);
        ground->SetCollide(false);
        sys->AddBody(ground);

        // Create the sled body
        auto sled = std::shared_ptr<ChBody>(sys->NewBody());
        sled->SetIdentifier(1);
        sled->SetMass(550);
        sled->SetInertiaXX(ChVector<>(100, 100, 100));
        sled->SetPos(ChVector<>(0, 0, 0));
        sled->SetPos_dt(ChVector<>(init_vel, 0, 0));
        sled->SetBodyFixed(false);
        sled->SetCollide(false);

        auto box_sled = chrono_types::make_shared<ChBoxShape>();
        box_sled->GetBoxGeometry().Size = ChVector<>(1, 0.25, 0.25);
        sled->AddVisualShape(box_sled, ChFrame<>());

        sys->AddBody(sled);

        // Create the wheel body
        auto wheel = std::shared_ptr<ChBody>(sys->NewBody());
        wheel->SetIdentifier(2);
        wheel->SetMass(350);
        wheel->SetInertiaXX(ChVector<>(50, 138, 138));
        wheel->SetPos(ChVector<>(2, 0, 0));
        wheel->SetRot(ChQuaternion<>(1, 0, 0, 0));
        wheel->SetPos_dt(ChVector<>(init_vel, 0, 0));
        wheel->SetBodyFixed(false);
        wheel->SetCollide(true);

        auto wheel_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

        wheel->GetCollisionModel()->ClearModel();
        utils::AddCylinderGeometry(wheel.get(), wheel_mat, 0.3, 0.1, ChVector<>(0, 0, 0), Q_from_AngZ(CH_C_PI_2));
        wheel->GetCollisionModel()->BuildModel();

        sys->AddBody(wheel);

        // Create and initialize translational joint ground - sled
        prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
        prismatic->Initialize(ground, sled, ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngY(CH_C_PI_2)));
        sys->AddLink(prismatic);

        // Create and initialize revolute joint sled - wheel
        revolute = chrono_types::make_shared<ChLinkLockRevolute>();
        revolute->Initialize(wheel, sled, ChCoordsys<>(ChVector<>(1, 0, 0), Q_from_AngX(CH_C_PI_2)));
        sys->AddLink(revolute);
    }

    Options opts;
    bool animate;
    ChSystemMulticoreNSC* sys;
    std::shared_ptr<ChLinkLockPrismatic> prismatic;
    std::shared_ptr<ChLinkLockRevolute> revolute;
};

TEST_P(JointsDVI, simulate) {
    ////std::cout << "Solver type:  " << as_integer(opts.type) << "  mode:  " << as_integer(opts.mode)
    ////          << "  max_iter_bilateral: " << opts.max_iter_bilateral << "  max_iter_normal: " <<
    ///opts.max_iter_normal /          << "  max_iter_sliding: " << opts.max_iter_sliding << std::endl;

    // Maximum allowable constraint violation
    double max_cnstr_violation = 1e-4;

    double time_end = 2;
    double time_step = 1e-3;

    if (animate) {
#ifdef CHRONO_OPENGL
        opengl::ChVisualSystemOpenGL vis;
        vis.AttachSystem(sys);
        vis.SetWindowTitle("");
        vis.SetWindowSize(1280, 720);
        vis.SetRenderMode(opengl::WIREFRAME);
        vis.Initialize();
        vis.SetCameraPosition(ChVector<>(0, -8, 0), ChVector<>(0, 0, 0));
        vis.SetCameraVertical(CameraVerticalDir::Z);

        while (sys->GetChTime() < time_end) {
            if (vis.Run()) {
                sys->DoStepDynamics(time_step);
                vis.Render();
            }
        }
#endif
    } else {
        while (sys->GetChTime() < time_end) {
            // Advance simulation.
            sys->DoStepDynamics(time_step);

            // Check constraints for prismatic joint
            ChVectorDynamic<> pC = prismatic->GetConstraintViolation();
            for (int i = 0; i < 5; i++) {
                ASSERT_NEAR(pC(i), 0.0, max_cnstr_violation);
            }

            // Check constraints for revolute joint
            ChVectorDynamic<> rC = revolute->GetConstraintViolation();
            for (int i = 0; i < 5; i++) {
                ASSERT_NEAR(rC(i), 0.0, max_cnstr_violation);
            }
        }
    }
}

std::vector<Options> options{
    {SolverMode::NORMAL, SolverType::APGDREF, 100, 1000, 0},
    {SolverMode::NORMAL, SolverType::APGDREF, 0, 1000, 0},
    {SolverMode::SLIDING, SolverType::APGDREF, 100, 0, 1000},
    {SolverMode::SLIDING, SolverType::APGDREF, 0, 0, 1000},

    ////{SolverMode::NORMAL, SolverType::APGD, 100, 1000, 0},
    ////{SolverMode::NORMAL, SolverType::APGD, 0, 1000, 0},
    ////{SolverMode::SLIDING, SolverType::APGD, 100, 0, 1000},
    ////{SolverMode::SLIDING, SolverType::APGD, 0, 0, 1000}
};

INSTANTIATE_TEST_SUITE_P(ChronoMulticore, JointsDVI, ::testing::ValuesIn(options));
