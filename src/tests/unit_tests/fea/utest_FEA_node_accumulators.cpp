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
// Unit tests for force (and torque) accumulators on FEA nodes.
// - accumulated forces are added to (and do not replace) the force set with SetForce
// - accumulators are independent of each other
// - ChNodeFEAxyzrot accumulators behave exactly like ChBody accumulators
// - ChMesh convenience functions for adding and clearing accumulators on all mesh nodes
// - accumulators are not copied with a node
// - accumulated forces drive the node dynamics
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzD.h"
#include "chrono/fea/ChNodeFEAxyzP.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

#include "gtest/gtest.h"

using namespace chrono;
using namespace chrono::fea;

const double tol = 1e-12;

// Evaluate the generalized applied forces of a node (residual term R = F).
static ChVectorDynamic<> NodeResidual(ChNodeFEAbase& node) {
    ChVectorDynamic<> R(node.GetNumCoordsVelLevel());
    R.setZero();
    node.NodeIntLoadResidual_F(0, R, 1.0);
    return R;
}

// Evaluate the applied loads of a node (no gyroscopic terms).
static ChVectorDynamic<> NodeAppliedLoads(ChNodeFEAbase& node) {
    ChVectorDynamic<> R(node.GetNumCoordsVelLevel());
    R.setZero();
    node.NodeIntLoadResidual_F_applied(0, R, 1.0);
    return R;
}

static void CheckVector(const ChVectorDynamic<>& v, const ChVector3d& expected, int offset = 0) {
    ASSERT_NEAR(v(offset + 0), expected.x(), tol);
    ASSERT_NEAR(v(offset + 1), expected.y(), tol);
    ASSERT_NEAR(v(offset + 2), expected.z(), tol);
}

static void CheckVector(const ChVector3d& v, const ChVector3d& expected) {
    ASSERT_NEAR(v.x(), expected.x(), tol);
    ASSERT_NEAR(v.y(), expected.y(), tol);
    ASSERT_NEAR(v.z(), expected.z(), tol);
}

// -----------------------------------------------------------------------------

TEST(ChNodeFEAxyz, accumulators) {
    ChVector3d F0(1, 2, 3);
    ChVector3d F1(-0.5, 0.25, 4);
    ChVector3d F2(10, -20, 30);

    auto node = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(1, 0, 0));
    node->SetForce(F0);

    // No accumulators: only the force set with SetForce
    CheckVector(NodeResidual(*node), F0);

    // Accumulators are added to the node force
    auto idx1 = node->AddAccumulator();
    auto idx2 = node->AddAccumulator();
    ASSERT_NE(idx1, idx2);

    node->EmptyAccumulator(idx1);
    node->EmptyAccumulator(idx2);
    node->AccumulateForce(idx1, F1);
    node->AccumulateForce(idx1, F1);
    node->AccumulateForce(idx2, F2);

    CheckVector(node->GetAccumulatedForce(idx1), 2 * F1);
    CheckVector(node->GetAccumulatedForce(idx2), F2);
    CheckVector(NodeAppliedLoads(*node), F0 + 2 * F1 + F2);
    CheckVector(NodeResidual(*node), F0 + 2 * F1 + F2);

    // The same total load is passed to the solver variables
    node->VariablesFbReset();
    node->VariablesFbLoadForces(1.0);
    CheckVector(ChVectorDynamic<>(node->Variables().Force()), F0 + 2 * F1 + F2);

    // Setting the node force does not affect the accumulators
    node->SetForce(VNULL);
    CheckVector(NodeResidual(*node), 2 * F1 + F2);

    // Emptying an accumulator does not affect the others
    node->EmptyAccumulator(idx1);
    CheckVector(node->GetAccumulatedForce(idx1), VNULL);
    CheckVector(NodeResidual(*node), F2);
}

TEST(ChNodeFEAxyz, derived_nodes) {
    // Nodes derived from ChNodeFEAxyz inherit the accumulators (loaded in the position-level residual entries)
    ChVector3d F0(1, 2, 3);
    ChVector3d F1(4, 5, 6);

    auto node = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(0, 0, 0), ChVector3d(1, 0, 0));
    node->SetForce(F0);
    auto idx = node->AddAccumulator();
    node->AccumulateForce(idx, F1);

    auto R = NodeResidual(*node);
    CheckVector(R, F0 + F1, 0);
    CheckVector(R, VNULL, 3);
}

TEST(ChNodeFEAxyz, copy) {
    // Accumulators are not copied with the node
    ChVector3d F0(1, 2, 3);
    ChVector3d F1(4, 5, 6);

    ChNodeFEAxyz node(ChVector3d(1, 0, 0));
    node.SetForce(F0);
    auto idx = node.AddAccumulator();
    node.AccumulateForce(idx, F1);

    ChNodeFEAxyz node_copy(node);
    CheckVector(NodeAppliedLoads(node_copy), F0);
    CheckVector(NodeResidual(node_copy), F0);

    ChNodeFEAxyz node_assigned;
    auto idx_assigned = node_assigned.AddAccumulator();
    node_assigned.AccumulateForce(idx_assigned, F1);
    node_assigned = node;
    CheckVector(node_assigned.GetAccumulatedForce(idx_assigned), F1);
    CheckVector(NodeResidual(node_assigned), F0 + F1);
}

// -----------------------------------------------------------------------------

TEST(ChNodeFEAxyzrot, accumulators) {
    ChFramed frame(ChVector3d(1, 2, 3), QuatFromAngleAxis(CH_PI / 3, ChVector3d(1, 1, 0).GetNormalized()));

    ChVector3d F0(1, 2, 3);
    ChVector3d T0(0.1, 0.2, 0.3);
    ChVector3d F1(-0.5, 0.25, 4);
    ChVector3d P1(1.5, 2.5, 3.5);
    ChVector3d T2(10, -20, 30);

    auto node = chrono_types::make_shared<ChNodeFEAxyzrot>(frame);
    node->SetForce(F0);
    node->SetTorque(T0);

    // A rigid body with the same frame, loaded through its accumulators, is used as reference
    auto body = chrono_types::make_shared<ChBody>();
    body->SetPos(frame.GetPos());
    body->SetRot(frame.GetRot());

    auto idx1 = node->AddAccumulator();
    auto idx2 = node->AddAccumulator();
    auto bidx1 = body->AddAccumulator();
    auto bidx2 = body->AddAccumulator();

    node->AccumulateForce(idx1, F1, P1, false);
    node->AccumulateTorque(idx2, T2, false);
    node->AccumulateTorque(idx2, T2, true);
    body->AccumulateForce(bidx1, F1, P1, false);
    body->AccumulateTorque(bidx2, T2, false);
    body->AccumulateTorque(bidx2, T2, true);

    CheckVector(node->GetAccumulatedForce(idx1), body->GetAccumulatedForce(bidx1));
    CheckVector(node->GetAccumulatedTorque(idx1), body->GetAccumulatedTorque(bidx1));
    CheckVector(node->GetAccumulatedForce(idx2), body->GetAccumulatedForce(bidx2));
    CheckVector(node->GetAccumulatedTorque(idx2), body->GetAccumulatedTorque(bidx2));

    // Expected total load: force in absolute frame, torque in node frame
    ChVector3d force = F0 + F1;
    ChVector3d torque = T0 + frame.TransformDirectionParentToLocal(Vcross(P1 - frame.GetPos(), F1)) + frame.TransformDirectionParentToLocal(T2) + T2;

    auto A = NodeAppliedLoads(*node);
    CheckVector(A, force, 0);
    CheckVector(A, torque, 3);

    // The node is at rest, so there is no gyroscopic torque
    auto R = NodeResidual(*node);
    CheckVector(R, force, 0);
    CheckVector(R, torque, 3);

    node->VariablesFbReset();
    node->VariablesFbLoadForces(1.0);
    CheckVector(ChVectorDynamic<>(node->Variables().Force()), force, 0);
    CheckVector(ChVectorDynamic<>(node->Variables().Force()), torque, 3);

    // Emptying an accumulator does not affect the others nor the node force and torque
    node->EmptyAccumulator(idx1);
    R = NodeResidual(*node);
    CheckVector(R, F0, 0);
    CheckVector(R, T0 + frame.TransformDirectionParentToLocal(T2) + T2, 3);
}

TEST(ChNodeFEAxyzrot, applied_loads) {
    // For a spinning node, the residual includes the gyroscopic torque but the applied loads do not
    ChVector3d F0(1, 2, 3);
    ChVector3d T0(0.1, 0.2, 0.3);
    ChVector3d T1(-1, 0.5, 2);
    ChVector3d w(0.5, -1.0, 2.0);  // angular velocity, in node frame

    auto node = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFramed(ChVector3d(1, 0, 0)));
    node->SetMass(2);
    node->GetInertia() = ChMatrix33<>(ChVector3d(1, 2, 3));
    node->SetForce(F0);
    node->SetTorque(T0);
    node->SetAngVelLocal(w);
    auto idx = node->AddAccumulator();
    node->AccumulateTorque(idx, T1, true);

    ChVector3d gyro = Vcross(w, ChVector3d(1 * w.x(), 2 * w.y(), 3 * w.z()));
    ASSERT_GT(gyro.Length(), 0.1);

    auto A = NodeAppliedLoads(*node);
    CheckVector(A, F0, 0);
    CheckVector(A, T0 + T1, 3);

    auto R = NodeResidual(*node);
    CheckVector(R, F0, 0);
    CheckVector(R, T0 + T1 - gyro, 3);
}

TEST(ChNodeFEAxyzrot, copy) {
    // Accumulators are not copied with the node
    ChVector3d F0(1, 2, 3);
    ChVector3d F1(4, 5, 6);

    ChNodeFEAxyzrot node(ChFramed(ChVector3d(1, 0, 0)));
    node.SetForce(F0);
    auto idx = node.AddAccumulator();
    node.AccumulateForce(idx, F1, node.GetPos(), false);

    ChNodeFEAxyzrot node_copy(node);
    CheckVector(NodeAppliedLoads(node_copy), F0, 0);
    CheckVector(NodeResidual(node_copy), F0, 0);

    ChNodeFEAxyzrot node_assigned;
    auto idx_assigned = node_assigned.AddAccumulator();
    node_assigned.AccumulateForce(idx_assigned, F1, node_assigned.GetPos(), false);
    node_assigned = node;
    CheckVector(node_assigned.GetAccumulatedForce(idx_assigned), F1);
    CheckVector(NodeResidual(node_assigned), F0 + F1, 0);
}

// -----------------------------------------------------------------------------

TEST(ChMesh, accumulators) {
    auto mesh = chrono_types::make_shared<ChMesh>();

    auto node_xyz = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 0, 0));
    auto node_xyzD = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(1, 0, 0), ChVector3d(1, 0, 0));
    auto node_xyzP = chrono_types::make_shared<ChNodeFEAxyzP>(ChVector3d(2, 0, 0));
    auto node_xyzrot = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFramed(ChVector3d(3, 0, 0)));
    mesh->AddNode(node_xyz);
    mesh->AddNode(node_xyzD);
    mesh->AddNode(node_xyzP);
    mesh->AddNode(node_xyzrot);

    // An accumulator added directly to one of the nodes (e.g., by some other user)
    auto other_idx = node_xyzD->AddAccumulator();
    node_xyzD->AccumulateForce(other_idx, ChVector3d(7, 8, 9));

    auto indices = mesh->AddAccumulators();
    ASSERT_EQ(indices.size(), mesh->GetNumNodes());
    ASSERT_EQ(indices[0], 0u);
    ASSERT_EQ(indices[1], 1u);
    ASSERT_EQ(indices[2], ChMesh::NO_ACCUMULATOR);
    ASSERT_EQ(indices[3], 0u);

    ChVector3d F(1, 2, 3);
    node_xyz->AccumulateForce(indices[0], F);
    node_xyzD->AccumulateForce(indices[1], F);
    node_xyzrot->AccumulateForce(indices[3], F, node_xyzrot->GetPos(), false);

    CheckVector(NodeResidual(*node_xyz), F);
    CheckVector(NodeResidual(*node_xyzD), F + ChVector3d(7, 8, 9));
    CheckVector(NodeResidual(*node_xyzrot), F, 0);

    // Clearing the mesh accumulators does not affect the other accumulator
    mesh->EmptyAccumulators(indices);
    CheckVector(NodeResidual(*node_xyz), VNULL);
    CheckVector(NodeResidual(*node_xyzD), ChVector3d(7, 8, 9));
    CheckVector(NodeResidual(*node_xyzrot), VNULL, 0);
}

// -----------------------------------------------------------------------------

TEST(ChNodeFEAxyz, dynamics) {
    // A free node, loaded with a force set with SetForce and forces in two accumulators, moves with an acceleration
    // corresponding to the total applied force.
    double mass = 2.0;
    double step = 1e-3;
    int num_steps = 100;
    ChVector3d F0(1, 0, 0);
    ChVector3d F1(0, 2, 0);
    ChVector3d F2(0, 0, -3);

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(VNULL);

    auto mesh = chrono_types::make_shared<ChMesh>();
    auto node = chrono_types::make_shared<ChNodeFEAxyz>(ChVector3d(0, 0, 0));
    node->SetMass(mass);
    mesh->AddNode(node);
    sys.Add(mesh);

    node->SetForce(F0);
    auto idx1 = node->AddAccumulator();
    auto idx2 = node->AddAccumulator();

    for (int i = 0; i < num_steps; i++) {
        node->EmptyAccumulator(idx1);
        node->EmptyAccumulator(idx2);
        node->AccumulateForce(idx1, F1);
        node->AccumulateForce(idx2, F2);
        sys.DoStepDynamics(step);
    }

    ChVector3d vel = (F0 + F1 + F2) / mass * (num_steps * step);
    ASSERT_NEAR(node->GetPosDt().x(), vel.x(), 1e-10);
    ASSERT_NEAR(node->GetPosDt().y(), vel.y(), 1e-10);
    ASSERT_NEAR(node->GetPosDt().z(), vel.z(), 1e-10);
}
