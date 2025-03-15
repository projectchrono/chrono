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
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/utils/ChUtilsCreators.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "../ut_utils.h"

using namespace chrono;

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

    // Create the mechanical sys
    ChSystemMulticoreNSC* sys = new ChSystemMulticoreNSC();
    sys->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Set number of threads
    sys->SetNumThreads(1);

    // Edit sys settings
    sys->GetSettings()->solver.tolerance = tolerance;
    sys->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    sys->GetSettings()->solver.clamp_bilaterals = clamp_bilaterals;
    sys->GetSettings()->solver.bilateral_clamp_speed = bilateral_clamp_speed;

    sys->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    sys->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    sys->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    sys->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    sys->ChangeSolverType(SolverType::APGD);

    // Define a couple of rotations for later use
    ChQuaternion<> y2x;
    ChQuaternion<> z2y;
    y2x.SetFromAngleZ(-CH_PI / 2);
    z2y.SetFromAngleX(-CH_PI / 2);

    // Create the ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys->AddBody(ground);

    // Attach a visualization asset representing the Y axis.
    auto box = chrono_types::make_shared<ChVisualShapeBox>(0.04, 6, 0.04);
    ground->AddVisualShape(box);

    // Create a pendulum modeled using ChBody
    auto pend_1 = chrono_types::make_shared<ChBody>();
    sys->AddBody(pend_1);
    pend_1->SetFixed(false);
    pend_1->EnableCollision(false);
    pend_1->SetMass(1);
    pend_1->SetInertiaXX(ChVector3d(0.2, 1, 1));

    // Specify the initial position of the pendulum (horizontal, pointing towards
    // positive X). In this case, we set the absolute position of its center of
    // mass.
    pend_1->SetPos(ChVector3d(1, 1, 0));

    // Create a revolute joint to connect pendulum to ground. We specify the link
    // coordinate frame in the absolute frame.
    auto rev_1 = chrono_types::make_shared<ChLinkLockRevolute>();
    rev_1->Initialize(ground, pend_1, ChFrame<>(ChVector3d(0, 1, 0), z2y));
    sys->AddLink(rev_1);

    // Create a pendulum modeled using ChBodyAuxRef
    auto pend_2 = chrono_types::make_shared<ChBodyAuxRef>();
    sys->Add(pend_2);
    pend_2->SetFixed(false);
    pend_2->EnableCollision(false);
    pend_2->SetMass(1);
    pend_2->SetInertiaXX(ChVector3d(0.2, 1, 1));
    // NOTE: the inertia tensor must still be expressed in the centroidal frame!

    // In this case, we must specify the centroidal frame, relative to the body
    // reference frame.
    pend_2->SetFrameCOMToRef(ChFrame<>(ChVector3d(1, 0, 0), ChQuaternion<>(1, 0, 0, 0)));

    // Specify the initial position of the pendulum (horizontal, pointing towards
    // positive X).  Here, we want to specify the position of the body reference
    // frame (relative to the absolute frame). Recall that the body reference
    // frame is located at the pin.
    pend_2->SetFrameRefToAbs(ChFrame<>(ChVector3d(0, -1, 0)));

    // Create a revolute joint to connect pendulum to ground. We specify the link
    // coordinate frame in the absolute frame.
    auto rev_2 = chrono_types::make_shared<ChLinkLockRevolute>();
    rev_2->Initialize(ground, pend_2, ChFrame<>(ChVector3d(0, -1, 0), z2y));
    sys->AddLink(rev_2);

    // Tolerances
    double pos_tol = 1e-6;
    double vel_tol = 1e-6;
    double acc_tol = 1e-5;

    double quat_tol = 1e-6;
    double avel_tol = 1e-6;
    double aacc_tol = 1e-5;

    // Perform the simulation
    if (animate) {
#ifdef CHRONO_VSG
        auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
        vis->AttachSystem(sys);
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

        while (vis->Run()) {
            sys->DoStepDynamics(time_step);
            vis->Render();
        }
#else
        std::cout << "Run-time visualization not available.  Cannot animate mechanism." << std::endl;
        FAIL();
#endif
    } else {
        while (sys->GetChTime() < time_end) {
            sys->DoStepDynamics(time_step);

            ASSERT_NEAR(pend_1->GetPos().x(), pend_2->GetPos().x(), pos_tol);
            ASSERT_NEAR(pend_1->GetPos().z(), pend_2->GetPos().z(), pos_tol);

            Assert_near(pend_1->GetRot(), pend_2->GetRot(), quat_tol);

            Assert_near(pend_1->GetPosDt(), pend_2->GetPosDt(), vel_tol);
            Assert_near(pend_1->GetAngVelParent(), pend_2->GetAngVelParent(), avel_tol);

            Assert_near(pend_1->GetPosDt2(), pend_2->GetPosDt2(), acc_tol);
            Assert_near(pend_1->GetAngAccParent(), pend_2->GetAngAccParent(), aacc_tol);
        }
    }
}
