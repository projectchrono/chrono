// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// Unit test for ChBody::IntLoadResidual_F_applied.
// The applied loads on a body include the loads from body accumulators and from
// ChForce objects, but exclude the gravitational force and the gyroscopic torque
// (both included in ChBody::IntLoadResidual_F).
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"

#include "gtest/gtest.h"

using namespace chrono;

const double tol = 1e-12;

static void CheckVector(const ChVectorDynamic<>& v, const ChVector3d& expected, int offset) {
    ASSERT_NEAR(v(offset + 0), expected.x(), tol);
    ASSERT_NEAR(v(offset + 1), expected.y(), tol);
    ASSERT_NEAR(v(offset + 2), expected.z(), tol);
}

TEST(ChBody, applied_loads) {
    ChSystemNSC sys;
    ChVector3d gravity(0, 0, -9.81);
    sys.SetGravitationalAcceleration(gravity);

    double mass = 2;
    ChVector3d inertia(1, 2, 3);
    ChVector3d ang_vel_loc(0.5, -1.0, 2.0);

    auto body = chrono_types::make_shared<ChBody>();
    body->SetMass(mass);
    body->SetInertiaXX(inertia);
    body->SetPos(ChVector3d(1, 2, 3));
    body->SetRot(QuatFromAngleAxis(CH_PI / 5, ChVector3d(1, 1, 1).GetNormalized()));
    body->SetAngVelLocal(ang_vel_loc);
    sys.AddBody(body);

    // Loads from an accumulator
    auto idx = body->AddAccumulator();
    body->AccumulateForce(idx, ChVector3d(1, -2, 3), ChVector3d(1.5, 2.5, 3.5), false);
    body->AccumulateTorque(idx, ChVector3d(0.1, 0.2, 0.3), true);

    // Load from a ChForce (world-fixed direction, applied at a point away from the COM)
    auto force = chrono_types::make_shared<ChForce>();
    body->AddForce(force);
    force->SetMode(ChForce::ForceType::FORCE);
    force->SetFrame(ChForce::ReferenceFrame::WORLD);
    force->SetAlign(ChForce::AlignmentFrame::WORLD_DIR);
    force->SetVpoint(ChVector3d(0.5, 2, 3));
    force->SetDir(ChVector3d(0, 1, 0));
    force->SetMforce(5);

    sys.Update();

    ChVector3d f_force;
    ChVector3d f_torque;
    force->GetBodyForceTorque(f_force, f_torque);
    ASSERT_GT(f_torque.Length(), 0.1);

    ChVector3d applied_force = body->GetAccumulatedForce(idx) + f_force;
    ChVector3d applied_torque = body->GetAccumulatedTorque(idx) + f_torque;

    // The residual functions are called through the base class interface (as done by the system and assemblies)
    ChPhysicsItem& item = *body;

    ChVectorDynamic<> R_applied(6);
    R_applied.setZero();
    item.IntLoadResidual_F_applied(0, R_applied, 1.0);
    CheckVector(R_applied, applied_force, 0);
    CheckVector(R_applied, applied_torque, 3);

    // The full residual adds the gravitational force and subtracts the gyroscopic torque
    ChVector3d gyro = Vcross(ang_vel_loc, ChVector3d(inertia.x() * ang_vel_loc.x(), inertia.y() * ang_vel_loc.y(), inertia.z() * ang_vel_loc.z()));
    ASSERT_GT(gyro.Length(), 0.1);

    ChVectorDynamic<> R_full(6);
    R_full.setZero();
    item.IntLoadResidual_F(0, R_full, 1.0);
    CheckVector(R_full, applied_force + mass * gravity, 0);
    CheckVector(R_full, applied_torque - gyro, 3);

    // Scaling factor
    ChVectorDynamic<> R_scaled(6);
    R_scaled.setZero();
    item.IntLoadResidual_F_applied(0, R_scaled, -2.0);
    CheckVector(R_scaled, -2.0 * applied_force, 0);
    CheckVector(R_scaled, -2.0 * applied_torque, 3);
}
