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
// Author: Radu Serban
// =============================================================================
//
// Unit test for run-time modification of the added mass blocks of a
// ChLoadHydrodynamics.
//
// A free body is driven by a constant force and torque. Halfway through the
// simulation, its added mass block is replaced. The accelerations over each
// half must reflect the added mass in effect at that time, for solvers that
// work with the full (KRM-assembled) mass matrix and for Schur complement-based
// solvers that rely on the inverse of the total mass matrix provided by the
// hydrodynamic load.
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLoadHydrodynamics.h"

using namespace chrono;

static const double tol = 1e-8;

static const double mass = 2;
static const double inertia = 1;
static const ChVector3d force(10, 0, 0);
static const ChVector3d torque(0, 0, 4);

static const double step = 1e-3;
static const int num_steps = 100;

// Diagonal 6x6 added mass block with given translational and rotational coefficients.
static ChMatrix66d AddedMassBlock(double am_trans, double am_rot) {
    ChMatrix66d M = ChMatrix66d::Zero();
    M.diagonal() << am_trans, am_trans, am_trans, am_rot, am_rot, am_rot;
    return M;
}

// How the added mass block is modified halfway through the simulation.
enum class UpdateMethod {
    UPDATE,  // ChLoadHydrodynamics::UpdateBodyAddedMassBlocks (6x6 self blocks)
    SET      // ChLoadHydrodynamics::SetBodyAddedMassBlocks (full 6x6n blocks)
};

static void RunTest(ChSolver::Type solver_type, UpdateMethod method) {
    const double am_trans_1 = 3, am_rot_1 = 1;
    const double am_trans_2 = 6, am_rot_2 = 3;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(VNULL);
    sys.SetSolverType(solver_type);

    auto body = chrono_types::make_shared<ChBody>();
    body->SetMass(mass);
    body->SetInertiaXX(ChVector3d(inertia, inertia, inertia));
    sys.AddBody(body);

    auto acc = body->AddAccumulator();
    body->AccumulateForce(acc, force, body->GetPos(), false);
    body->AccumulateTorque(acc, torque, false);

    ChBodyAddedMassBlocks body_blocks;
    body_blocks.push_back({body, ChMatrixDynamic<>(AddedMassBlock(am_trans_1, am_rot_1))});
    auto hydro_load = chrono_types::make_shared<ChLoadHydrodynamics>(body_blocks);
    sys.Add(hydro_load);

    // First half: original added mass
    for (int i = 0; i < num_steps; i++)
        sys.DoStepDynamics(step);
    auto v1 = body->GetLinVel();
    auto w1 = body->GetAngVelParent();

    double t = num_steps * step;
    ASSERT_NEAR(v1.x(), force.x() / (mass + am_trans_1) * t, tol);
    ASSERT_NEAR(w1.z(), torque.z() / (inertia + am_rot_1) * t, tol);

    // Modify the added mass block
    switch (method) {
        case UpdateMethod::UPDATE:
            hydro_load->UpdateBodyAddedMassBlocks({AddedMassBlock(am_trans_2, am_rot_2)});
            break;
        case UpdateMethod::SET:
            hydro_load->SetBodyAddedMassBlocks({ChMatrixDynamic<>(AddedMassBlock(am_trans_2, am_rot_2))});
            break;
    }

    // Second half: modified added mass
    for (int i = 0; i < num_steps; i++)
        sys.DoStepDynamics(step);
    auto v2 = body->GetLinVel();
    auto w2 = body->GetAngVelParent();

    ASSERT_NEAR(v2.x() - v1.x(), force.x() / (mass + am_trans_2) * t, tol);
    ASSERT_NEAR(w2.z() - w1.z(), torque.z() / (inertia + am_rot_2) * t, tol);
}

// Direct solver: works with the KRM-assembled mass matrix (reference behavior).
TEST(ChLoadHydrodynamics, update_sparse_qr) {
    RunTest(ChSolver::Type::SPARSE_QR, UpdateMethod::UPDATE);
}

TEST(ChLoadHydrodynamics, set_sparse_qr) {
    RunTest(ChSolver::Type::SPARSE_QR, UpdateMethod::SET);
}

// Schur complement-based solvers: rely on the inverse of the total mass matrix.
TEST(ChLoadHydrodynamics, update_apgd) {
    RunTest(ChSolver::Type::APGD, UpdateMethod::UPDATE);
}

TEST(ChLoadHydrodynamics, set_apgd) {
    RunTest(ChSolver::Type::APGD, UpdateMethod::SET);
}

TEST(ChLoadHydrodynamics, update_barzilaiborwein) {
    RunTest(ChSolver::Type::BARZILAIBORWEIN, UpdateMethod::UPDATE);
}

TEST(ChLoadHydrodynamics, set_barzilaiborwein) {
    RunTest(ChSolver::Type::BARZILAIBORWEIN, UpdateMethod::SET);
}

// PSOR does not support systems with KRM blocks (the mechanism used by ChLoadHydrodynamics) and must throw.
TEST(ChLoadHydrodynamics, psor_not_supported) {
    ASSERT_THROW(RunTest(ChSolver::Type::PSOR, UpdateMethod::UPDATE), std::runtime_error);
}
