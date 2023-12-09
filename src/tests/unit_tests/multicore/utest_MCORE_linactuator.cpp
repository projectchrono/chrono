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
// Test for linear actuator
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"

#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

#include "unit_testing.h"

using namespace chrono;

// -----------------------------------------------------------------------------
// Problem definition
// -----------------------------------------------------------------------------
double mass = 1.0;
ChVector<> inertiaXX(1, 1, 1);
ChVector<> gravity(0, 0, -9.80665);

class ChLinActuatorTest
    : public ::testing::TestWithParam<std::tuple<ChContactMethod, double, ChQuaternion<>>> {
  protected:
    ChLinActuatorTest();
    ~ChLinActuatorTest() { delete sys; }

    void VerifySolution(double time);

    ChSystemMulticore* sys;
    ChContactMethod cm;
    ChQuaternion<> rot;
    double speed;
    std::shared_ptr<ChBody> plate;
    std::shared_ptr<ChLinkLockPrismatic> prismatic;
    std::shared_ptr<ChLinkLinActuator> actuator;
    bool animate;
};

// Construct mechanical system
ChLinActuatorTest::ChLinActuatorTest() : animate(false) {
    cm = std::get<0>(GetParam());
    speed = std::get<1>(GetParam());
    rot = std::get<2>(GetParam());

    // Unit vector along translation axis, expressed in global frame
    ChVector<> axis = rot.GetZaxis();

    // Settings
    double tolerance = 1e-5;
    int max_iteration_bilateral = 100;
    int max_iteration_normal = 0;
    int max_iteration_sliding = 0;
    int max_iteration_spinning = 0;
    bool clamp_bilaterals = false;
    double bilateral_clamp_speed = 1000;

    // Create the mechanical system
    switch (cm) {
        case ChContactMethod::SMC:
            sys = new ChSystemMulticoreSMC();
            break;
        case ChContactMethod::NSC:
            sys = new ChSystemMulticoreNSC();
            break;
    }
    sys->Set_G_acc(gravity);

    // Set associated collision system
    sys->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    // Set number of threads
    sys->SetNumThreads(1);

    // Edit system settings
    sys->GetSettings()->solver.tolerance = tolerance;
    sys->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    sys->GetSettings()->solver.clamp_bilaterals = clamp_bilaterals;
    sys->GetSettings()->solver.bilateral_clamp_speed = bilateral_clamp_speed;

    if (cm == ChContactMethod::NSC) {
        ChSystemMulticoreNSC* msystemNSC = static_cast<ChSystemMulticoreNSC*>(sys);
        msystemNSC->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
        msystemNSC->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
        msystemNSC->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
        msystemNSC->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
        msystemNSC->ChangeSolverType(SolverType::APGD);
    }

    // Create the ground body.
    auto ground = chrono_types::make_shared<ChBody>();

    sys->AddBody(ground);
    ground->SetBodyFixed(true);

    auto box_g = chrono_types::make_shared<ChVisualShapeBox>(0.1, 0.1, 5);
    ground->AddVisualShape(box_g, ChFrame<>(2.5 * axis, rot));

    // Create the plate body.
    plate = chrono_types::make_shared<ChBody>();
    sys->AddBody(plate);
    plate->SetPos(ChVector<>(0, 0, 0));
    plate->SetRot(rot);
    plate->SetPos_dt(speed * axis);
    plate->SetMass(mass);
    plate->SetInertiaXX(inertiaXX);

    auto box_p = chrono_types::make_shared<ChVisualShapeBox>(1, 1, 0.2);
    plate->AddVisualShape(box_p);

    // Create prismatic (translational) joint between plate and ground.
    // We set the ground as the "master" body (second one in the initialization
    // call) so that the link coordinate system is expressed in the ground frame.
    prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(plate, ground, ChCoordsys<>(ChVector<>(0, 0, 0), rot));
    sys->AddLink(prismatic);

    // Create a ramp function to impose constant speed.  This function returns
    //   y(t) = 0 + t * speed
    //   y'(t) = speed
    auto actuator_fun = chrono_types::make_shared<ChFunction_Ramp>(0.0, speed);

    // Create the linear actuator, connecting the plate to the ground.
    // Here, we set the plate as the master body (second one in the initialization
    // call) so that the link coordinate system is expressed in the plate body
    // frame.
    actuator = chrono_types::make_shared<ChLinkLinActuator>();
    ChVector<> pt1 = ChVector<>(0, 0, 0);
    ChVector<> pt2 = axis;
    actuator->Initialize(ground, plate, false, ChCoordsys<>(pt1, rot), ChCoordsys<>(pt2, rot));
    actuator->SetDistanceOffset(1);
    actuator->SetActuatorFunction(actuator_fun);
    sys->AddLink(actuator);
}

// Verify simulation results against analytical solution at the specified time.
void ChLinActuatorTest::VerifySolution(double time) {
    // Tolerances
    double pos_tol = 1e-6;
    double vel_tol = 1e-6;
    double acc_tol = 1e-5;

    double quat_tol = 1e-6;
    double avel_tol = 1e-6;
    double aacc_tol = 1e-5;

    double rforce_tol = 1e-5;
    double rtorque_tol = 5e-3;

    double cnstr_tol = 1e-10;

    // Unit vector along translation axis, expressed in global frame
    ChVector<> axis = rot.GetZaxis();

    // Position, velocity, and acceleration (expressed in global frame)
    // ----------------------------------------------------------------

    ChVector<> pos = plate->GetPos();
    ChVector<> vel = plate->GetPos_dt();
    ChVector<> acc = plate->GetPos_dtdt();

    // The motion must be constant speed along the translation axis.
    ChVector<> pos_an = time * speed * axis;
    ChVector<> vel_an = speed * axis;
    ChVector<> acc_an = ChVector<>(0, 0, 0);

    ChVector<> pos_delta = pos - pos_an;
    ASSERT_LT(pos_delta.Length(), pos_tol);

    ChVector<> vel_delta = vel - vel_an;
    ASSERT_LT(vel_delta.Length(), vel_tol);

    ChVector<> acc_delta = acc - acc_an;
    ASSERT_LT(acc_delta.Length(), acc_tol);

    // Orientation and angular velocity / acceleration (expressed in global frame)
    // ---------------------------------------------------------------------------

    ChQuaternion<> quat = plate->GetRot();
    ChVector<> avel = plate->GetWvel_par();
    ChVector<> aacc = plate->GetWacc_par();

    // The motion must maintain constant orientation of the plate body.
    ChQuaternion<> quat_an = rot;
    ChVector<> avel_an = ChVector<>(0, 0, 0);
    ChVector<> aacc_an = ChVector<>(0, 0, 0);

    ChQuaternion<> quat_delta = quat - quat_an;
    ASSERT_LT(quat_delta.Length(), quat_tol);

    ChVector<> avel_delta = avel - avel_an;
    ASSERT_LT(avel_delta.Length(), avel_tol);

    ChVector<> aacc_delta = aacc - aacc_an;
    ASSERT_LT(aacc_delta.Length(), aacc_tol);

    // Reaction force and torque in prismatic joint
    // --------------------------------------------

    // These are expressed in the link coordinate system. We convert them to
    // the coordinate system of Body2 (in our case this is the ground).
    ChCoordsys<> linkCoordsysP = prismatic->GetLinkRelativeCoords();
    ChVector<> rforceP = prismatic->Get_react_force();
    ChVector<> rforceP_ground = linkCoordsysP.TransformDirectionLocalToParent(rforceP);

    ChVector<> rtorqueP = prismatic->Get_react_torque();
    ChVector<> rtorqueP_ground = linkCoordsysP.TransformDirectionLocalToParent(rtorqueP);

    // The reaction force in the prismatic joint is perpendicular to the
    // translation direction. This can be obtained from a force diagram.
    ChVector<> rforceP_an = gravity - Vdot(gravity, axis) * axis;
    ChVector<> rforceP_delta = rforceP_ground - rforceP_an;
    ASSERT_LT(rforceP_delta.Length(), rforce_tol);

    // The reaction torque at the joint location on ground has a non-zero
    // component in the y direction only.
    ChVector<> rtorqueP_an = Vcross(pos_an, rforceP_an);
    ChVector<> rtorqueP_delta = rtorqueP_ground - rtorqueP_an;
    ASSERT_LT(rtorqueP_delta.Length(), rtorque_tol);

    // Reaction force and torque in linear actuator
    // --------------------------------------------

    // These are expressed in the link coordinate system. The reaction force
    // represents the force that needs to be applied to the plate in order to
    // maintain the prescribed constant velocity.
    ChVector<> rforceA = actuator->Get_react_force();
    ChVector<> rtorqueA = actuator->Get_react_torque();

    // Analytically, the driving force can be obtained from a force diagram along
    // the translation axis.
    double rforceA_an = mass * Vdot(acc_an - gravity, axis);
    ASSERT_NEAR(-rforceA.x(), rforceA_an, rforce_tol);
    ////double rforceA_delta = (-rforceA.x()) - rforceA_an;
    ////if (std::abs(rforceA_delta) > rforce_tol) {
    ////    std::cout << "   at t = " << time << "   rforceA = " << -rforceA.x() << "  "
    ////        << "   rforceA_an = " << rforceA_an << "  "
    ////        << "   rforceA - rforceA_an = " << rforceA_delta << std::endl;
    ////    return false;
    ////}

    ChVector<> rtorqueA_an = ChVector<>(0, 0, 0);
    ChVector<> rtorqueA_delta = rtorqueA - rtorqueA_an;
    ASSERT_LT(rtorqueA_delta.Length(), rtorque_tol);

    // Constraint violations in prismatic joint
    // ----------------------------------------

    ChVectorDynamic<> CP = prismatic->GetConstraintViolation();
    for (int i = 0; i < 5; i++) {
        ASSERT_NEAR(CP(i), 0.0, cnstr_tol);
        ////if (std::abs(CP(i)) > cnstr_tol) {
        ////    std::cout << "   at t = " << time << "  constraint violation (prismatic " << i  << ") = " << CP(i) << std::endl;
        ////    return false;
        ////}
    }

    // Constraint violations in linear actuator
    // ----------------------------------------

    ChVectorDynamic<> CA = actuator->GetConstraintViolation();
    ASSERT_NEAR(CA(0), 0.0, cnstr_tol);
    ////if (std::abs(CA(00)) > cnstr_tol) {
    ////    std::cout << "   at t = " << time << "  constraint violation (actuator) = " << CA(0)
    ////        << std::endl;
    ////    return false;
    ////}
}

TEST_P(ChLinActuatorTest, simulate) {
    ////std::cout << "Contact Method: " << cm << std::endl;
    ////std::cout << "Speed: " << speed << std::endl;
    ////std::cout << "Rot: " << rot.e0() << " " << rot.e1() << " " << rot.e2() << " " << rot.e3() << std::endl;

    double time_end = 2;
    double time_step = 1e-4;
    double time = 0;

    if (animate) {
#ifdef CHRONO_OPENGL
        opengl::ChVisualSystemOpenGL vis;
        vis.AttachSystem(sys);
        vis.SetWindowTitle("");
        vis.SetWindowSize(1280, 720);
        vis.SetRenderMode(opengl::WIREFRAME);
        vis.Initialize();
        vis.AddCamera(ChVector<>(0, -10, 0), ChVector<>(0, 0, 0));
        vis.SetCameraVertical(CameraVerticalDir::Z);

        while (time < time_end) {
            // Advance simulation by one step.
            sys->DoStepDynamics(time_step);
            vis.Render();
            time += time_step;
            VerifySolution(time);
        }
#else
        std::cout << "OpenGL support not available.  Cannot animate mechanism." << std::endl;
        FAIL();
#endif
    } else {
        while (time < time_end) {
            sys->DoStepDynamics(time_step);
            time += time_step;
            VerifySolution(time);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(ChronoMulticore,
                         ChLinActuatorTest,
                         ::testing::Combine(::testing::Values(ChContactMethod::NSC, ChContactMethod::SMC),
                                            ::testing::Values(1.0, 0.5),
                                            ::testing::Values(QUNIT, Q_from_AngY(CH_C_PI / 4))));
