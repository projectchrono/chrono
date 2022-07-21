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
// Test for the assembly analysis.
//
// =============================================================================

////#include <cfloat>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include <cmath>

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"

using namespace chrono;

static void TestVector(const ChVector<>& v1, const ChVector<>& v2, double tol) {
    ASSERT_NEAR(v1.x(), v2.x(), tol);
    ASSERT_NEAR(v1.y(), v2.y(), tol);
    ASSERT_NEAR(v1.z(), v2.z(), tol);
}

static void TestQuaternion(const ChQuaternion<double>& q1, const ChQuaternion<double>& q2, double tol) {
    ASSERT_NEAR(q1.e0(), q2.e0(), tol);
    ASSERT_NEAR(q1.e1(), q2.e1(), tol);
    ASSERT_NEAR(q1.e2(), q2.e2(), tol);
    ASSERT_NEAR(q1.e3(), q2.e3(), tol);
}

TEST(FullAssembly, Assemble) {
    double mass = 1.0;                     // mass of pendulum
    double length = 4.0;                   // length of pendulum
    ChVector<> inertiaXX(0.04, 0.1, 0.1);  // mass moments of inertia of pendulum (centroidal frame)
    double g = 9.80665;                    // gravitational acceleration

    ChVector<> jointLoc(1, 2, 3);                       // absolute location of revolute joint
    double jointAngle = -CH_C_PI_4;                     // joint rotation angle (about global X axis)
    ChQuaternion<> jointRot = Q_from_AngX(jointAngle);  // orientation of revolute joint

    // Create the mechanical system
    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0.0, 0.0, -g));

    // Integrator settings
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.SetSolverMaxIterations(100);
    sys.SetSolverForceTolerance(1e-4);

    // Create the ground body
    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetIdentifier(0);
    ground->SetBodyFixed(true);

    // Create the pendulum body in an initial configuration at rest, with an
    // orientation that matches the specified joint orientation and a position
    // consistent with the specified joint location.
    // The pendulum CG is assumed to be at half its length.
    auto pendulum = chrono_types::make_shared<ChBody>();
    sys.AddBody(pendulum);
    pendulum->SetIdentifier(1);
    pendulum->SetPos(jointLoc + jointRot.Rotate(ChVector<>(length / 2, 0, 0)));
    pendulum->SetRot(jointRot);
    pendulum->SetMass(mass);
    pendulum->SetInertiaXX(inertiaXX);

    // Create revolute joint between pendulum and ground at "loc" in the global
    // reference frame. The revolute joint's axis of rotation will be the Z axis
    // of the specified rotation matrix.
    auto revoluteJoint = chrono_types::make_shared<ChLinkLockRevolute>();
    revoluteJoint->Initialize(pendulum, ground, ChCoordsys<>(jointLoc, jointRot));
    ////auto revoluteJoint = chrono_types::make_shared<ChLinkRevolute>();
    ////revoluteJoint->Initialize(pendulum, ground, ChFrame<>(jointLoc, jointRot));
    sys.AddLink(revoluteJoint);

    // Perform a system assembly.
    ////sys.DoAssembly(AssemblyLevel::VELOCITY | AssemblyLevel::ACCELERATION);
    sys.DoFullAssembly();

    // Extract position, velocity, and acceleration of pendulum body.
    ChVector<> pos = pendulum->GetPos();
    ChQuaternion<> rot = pendulum->GetRot();
    ChVector<> lin_vel = pendulum->GetPos_dt();
    ChVector<> ang_vel = pendulum->GetWvel_par();
    ChVector<> lin_acc = pendulum->GetPos_dtdt();
    ChVector<> ang_acc = pendulum->GetWacc_par();

    // Joint frame on 2nd body (ground), expressed in the body frame
    ChCoordsys<> linkCoordsys = revoluteJoint->GetLinkRelativeCoords();

    // Reaction force and torque on ground, expressed in joint frame
    ChVector<> rfrc = revoluteJoint->Get_react_force();
    ChVector<> rtrq = revoluteJoint->Get_react_torque();

    // Reaction force and torque on ground, expressed in ground frame
    rfrc = linkCoordsys.TransformDirectionLocalToParent(rfrc);
    rtrq = linkCoordsys.TransformDirectionLocalToParent(rtrq);

    // Analytical solution

    // Position and orientation
    ChVector<> pos_ref = jointLoc + ChVector<>(0.5 * length, 0, 0);
    ChQuaternion<> rot_ref(std::cos(jointAngle / 2), std::sin(jointAngle / 2), 0, 0);

    // Linear and angular velocities (expressed in absolute frame)
    ChVector<> lin_vel_ref(0, 0, 0);
    ChVector<> ang_vel_ref(0, 0, 0);

    // Angular acceleration (expressed in local frame)
    double omg_z = -0.5 * mass * g * length * std::sin(jointAngle) / (inertiaXX.z() + 0.25 * mass * length * length);

    // Angular acceleration (expressed in absolute frame)
    ChVector<> ang_acc_ref(0, -std::sin(jointAngle) * omg_z, std::cos(jointAngle) * omg_z);

    // Linear acceleration (expressed in absolute frame)
    ChVector<> lin_acc_ref(0, 0.5 * length * std::cos(jointAngle) * omg_z, 0.5 * length * std::sin(jointAngle) * omg_z);

    // Lagrange multipliers
    double lambda_1 = 0;
    double lambda_2 = -0.5 * mass * length * std::cos(jointAngle) * omg_z;
    double lambda_3 = -mass * g - 0.5 * mass * length * std::sin(jointAngle) * omg_z;
    double lambda_4 = -0.5 * length * std::sin(jointAngle) * lambda_2 + 0.5 * length * std::cos(jointAngle) * lambda_3;
    double lambda_5 = 0;

    // Reaction force and torque on first body (ground), at joint location, expressed in absolute frame
    ChVector<> rfrc_ref(lambda_1, lambda_2, lambda_3);
    ChVector<> rtrq_ref(lambda_5, -std::cos(jointAngle) * lambda_4, -std::sin(jointAngle) * lambda_4);

    // Compare simulation and analytical solution
    TestVector(pos, pos_ref, 1e-3);
    TestQuaternion(rot, rot_ref, 1e-4);
    TestVector(lin_vel, lin_vel_ref, 1e-4);
    TestVector(ang_vel, ang_vel_ref, 1e-4);
    TestVector(lin_acc, lin_acc_ref, 1e-2);
    TestVector(ang_acc, ang_acc_ref, 1e-2);
    TestVector(rfrc, rfrc_ref, 1e-2);
    TestVector(rtrq, rtrq_ref, 1e-2);
}
