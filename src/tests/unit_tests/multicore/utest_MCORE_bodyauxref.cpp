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
// Test for ChBodyAuxRef (body defined with respect to a non-centroidal frame),
// using two identical pendulums, modeled as a ChBody and as a ChBodyAuxRef,
// respectively.
//
// The two pendulums have length 2 and are pinned to ground through revolute
// joints with the rotation axis along the global Y axis. The absolute locations
// of the revolute joints are at (0, 1, 0) and (0, -1, 0), respectively.
//
// The ChBody pendulum is defined with respect to a centroidal frame (as assumed
// by ChBody) located at the geometric center of the pendulum, with the X axis
// along the length of the pendulum.
// The ChBodyAuxRef pendulum is defined with respect to a local frame, parallel
// to its centroidal frame but located at the pin location.  In other words, the
// center of mass of the pendulum is at (1, 0, 0), relative to the body frame.
//
// The two pendulums move under the action of gravity, acting along the negative
// global Z direction.
//
// =============================================================================

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono/ChConfig.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/utils/ChUtilsCreators.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

#include "unit_testing.h"

using namespace chrono;
using namespace chrono::collision;

TEST(ChronoMulticore, bodyauxref) {
    bool animate = false;

    // Settings
    double time_end = 5;
    double time_step = 1e-3;

    double tolerance = 1e-5;

    int max_iteration_bilateral = 100;
    int max_iteration_normal = 0;
    int max_iteration_sliding = 100;
    int max_iteration_spinning = 0;

    bool clamp_bilaterals = false;
    double bilateral_clamp_speed = 1000;

    // Create the mechanical system
    ChSystemMulticoreNSC* system = new ChSystemMulticoreNSC();
    system->Set_G_acc(ChVector<>(0, 0, -9.81));

    // Set number of threads
    system->SetNumThreads(1);

    // Edit system settings
    system->GetSettings()->solver.tolerance = tolerance;
    system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    system->GetSettings()->solver.clamp_bilaterals = clamp_bilaterals;
    system->GetSettings()->solver.bilateral_clamp_speed = bilateral_clamp_speed;

    system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    system->ChangeSolverType(SolverType::APGD);

    // Define a couple of rotations for later use
    ChQuaternion<> y2x;
    ChQuaternion<> z2y;
    y2x.Q_from_AngZ(-CH_C_PI / 2);
    z2y.Q_from_AngX(-CH_C_PI / 2);

    // Create the ground body
    auto ground = std::shared_ptr<ChBody>(system->NewBody());
    ground->SetBodyFixed(true);
    system->AddBody(ground);

    // Attach a visualization asset representing the Y axis.
    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = ChVector<>(0.02, 3, 0.02);
    ground->AddVisualShape(box);

    // Create a pendulum modeled using ChBody
    auto pend_1 = std::shared_ptr<ChBody>(system->NewBody());
    system->AddBody(pend_1);
    pend_1->SetIdentifier(1);
    pend_1->SetBodyFixed(false);
    pend_1->SetCollide(false);
    pend_1->SetMass(1);
    pend_1->SetInertiaXX(ChVector<>(0.2, 1, 1));

    // Attach a visualization asset. Note that the cylinder is defined with
    // respect to the centroidal reference frame (which is the body reference
    // frame for a ChBody).
    auto cyl_1 = chrono_types::make_shared<ChCylinderShape>();
    cyl_1->GetCylinderGeometry().p1 = ChVector<>(0, -1, 0);
    cyl_1->GetCylinderGeometry().p2 = ChVector<>(0, 1, 0);
    cyl_1->GetCylinderGeometry().rad = 0.2;
    pend_1->AddVisualShape(cyl_1, ChFrame<>(ChVector<>(0, 0, 0), y2x));

    // Specify the initial position of the pendulum (horizontal, pointing towards
    // positive X). In this case, we set the absolute position of its center of
    // mass.
    pend_1->SetPos(ChVector<>(1, 1, 0));

    // Create a revolute joint to connect pendulum to ground. We specify the link
    // coordinate frame in the absolute frame.
    auto rev_1 = chrono_types::make_shared<ChLinkLockRevolute>();
    rev_1->Initialize(ground, pend_1, ChCoordsys<>(ChVector<>(0, 1, 0), z2y));
    system->AddLink(rev_1);

    // Create a pendulum modeled using ChBodyAuxRef
    auto pend_2 = std::shared_ptr<ChBodyAuxRef>(system->NewBodyAuxRef());
    system->Add(pend_2);
    pend_2->SetIdentifier(2);
    pend_2->SetBodyFixed(false);
    pend_2->SetCollide(false);
    pend_2->SetMass(1);
    pend_2->SetInertiaXX(ChVector<>(0.2, 1, 1));
    // NOTE: the inertia tensor must still be expressed in the centroidal frame!

    // Attach a visualization asset. Note that now the cylinder is defined with
    // respect to the body reference frame.
    auto cyl_2 = chrono_types::make_shared<ChCylinderShape>();
    cyl_2->GetCylinderGeometry().p1 = ChVector<>(0, -1, 0);
    cyl_2->GetCylinderGeometry().p2 = ChVector<>(0, 1, 0);
    cyl_2->GetCylinderGeometry().rad = 0.2;
    pend_2->AddVisualShape(cyl_2, ChFrame<>(ChVector<>(1, 0, 0), y2x));

    // In this case, we must specify the centroidal frame, relative to the body
    // reference frame.
    pend_2->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(1, 0, 0), ChQuaternion<>(1, 0, 0, 0)));

    // Specify the initial position of the pendulum (horizontal, pointing towards
    // positive X).  Here, we want to specify the position of the body reference
    // frame (relative to the absolute frame). Recall that the body reference
    // frame is located at the pin.
    pend_2->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(0, -1, 0)));

    // Create a revolute joint to connect pendulum to ground. We specify the link
    // coordinate frame in the absolute frame.
    auto rev_2 = chrono_types::make_shared<ChLinkLockRevolute>();
    rev_2->Initialize(ground, pend_2, ChCoordsys<>(ChVector<>(0, -1, 0), z2y));
    system->AddLink(rev_2);

    // Tolerances
    double pos_tol = 1e-6;
    double vel_tol = 1e-6;
    double acc_tol = 1e-5;

    double quat_tol = 1e-6;
    double avel_tol = 1e-6;
    double aacc_tol = 1e-5;

    // Perform the simulation
    if (animate) {
#ifdef CHRONO_OPENGL
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.Initialize(1280, 720, "BodyAuxRef", system);
        gl_window.SetCamera(ChVector<>(6, -6, 1), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
        gl_window.SetRenderMode(opengl::WIREFRAME);
        gl_window.StartDrawLoop(time_step);
#else
        std::cout << "OpenGL support not available.  Cannot animate mechanism." << std::endl;
        FAIL();
#endif
    } else {
        while (system->GetChTime() < time_end) {
            system->DoStepDynamics(time_step);

            ASSERT_NEAR(pend_1->GetPos().x(), pend_2->GetPos().x(), pos_tol);
            ASSERT_NEAR(pend_1->GetPos().z(), pend_2->GetPos().z(), pos_tol);

            Assert_near(pend_1->GetRot(), pend_2->GetRot(), quat_tol);

            Assert_near(pend_1->GetPos_dt(), pend_2->GetPos_dt(), vel_tol);
            Assert_near(pend_1->GetWvel_par(), pend_2->GetWvel_par(), avel_tol);

            Assert_near(pend_1->GetPos_dtdt(), pend_2->GetPos_dtdt(), acc_tol);
            Assert_near(pend_1->GetWacc_par(), pend_2->GetWacc_par(), aacc_tol);
        }
    }
}
