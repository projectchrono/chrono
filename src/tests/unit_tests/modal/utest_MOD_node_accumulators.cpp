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
// Check that loads applied on internal items of a modal assembly are accounted
// for in the reduced model:
// - loads applied through FEA node accumulators on internal nodes act exactly
//   like the force and torque set directly on the nodes;
// - a fixed internal node (which has no coordinates) does not shift the loads
//   applied on the other internal nodes;
// - loads applied on an internal body through ChForce objects act exactly like
//   the same loads applied through a body accumulator.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChForce.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono_modal/ChModalAssembly.h"
#include "chrono_modal/ChModalSolverUndamped.h"
#include "chrono_modal/ChUnsymGenEigenvalueSolver.h"

#include "gtest/gtest.h"

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;

// How the loads are applied.
enum class LoadMode {
    NODE,              // node force and torque (SetForce, SetTorque) on the last internal node
    ACCUMULATOR,       // two node accumulators on the last internal node
    MIXED,             // half through the node force and torque, half through an accumulator
    FIXED_NODE,        // as NODE, with an additional fixed node first in the internal mesh
    BODY_ACCUMULATOR,  // accumulator on an internal body attached to the last internal node
    BODY_FORCE         // ChForce objects on an internal body attached to the last internal node
};

// Simulate a modal-reduced cantilever beam with loads applied at (or at a body attached to) its last internal node.
// Return the final position of the free end (a boundary node).
ChVector3d RunModel(LoadMode mode) {
    double beam_L = 6;
    int n_elements = 8;
    double step_size = 0.005;
    int num_steps = 100;
    unsigned int num_modes = 12;

    ChVector3d force(0, -1, 0.5);
    ChVector3d torque(0, 2, 0);  // in node frame

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(VNULL);
    sys.SetSolver(chrono_types::make_shared<ChSolverSparseQR>());

    auto modal_assembly = chrono_types::make_shared<ChModalAssembly>();
    modal_assembly->SetReductionType(ChModalAssembly::ReductionType::HERTING);
    modal_assembly->SetInternalNodesUpdate(true);
    modal_assembly->SetUseStaticCorrection(false);
    sys.Add(modal_assembly);

    auto mesh_internal = chrono_types::make_shared<ChMesh>();
    modal_assembly->AddInternal(mesh_internal);
    auto mesh_boundary = chrono_types::make_shared<ChMesh>();
    modal_assembly->Add(mesh_boundary);
    mesh_internal->SetAutomaticGravity(false);
    mesh_boundary->SetAutomaticGravity(false);

    auto section = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();
    section->SetDensity(1000);
    section->SetYoungModulus(100e6);
    section->SetShearModulusFromPoisson(0.31);
    section->SetRayleighDampingBeta(0.1);
    section->SetRayleighDampingAlpha(0.001);
    section->SetAsRectangularSection(0.05, 0.3);

    // A fixed node, not connected to any element, has no coordinates
    if (mode == LoadMode::FIXED_NODE) {
        auto node_fixed = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(0, 1, 0)));
        node_fixed->SetFixed(true);
        mesh_internal->AddNode(node_fixed);
    }

    auto node_A = chrono_types::make_shared<ChNodeFEAxyzrot>();
    mesh_boundary->AddNode(node_A);
    auto node_B = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(beam_L, 0, 0)));
    mesh_boundary->AddNode(node_B);

    ChBuilderBeamEuler builder;
    builder.BuildBeam(mesh_internal, section, n_elements, node_A, node_B, ChVector3d(0, 1, 0));

    auto base = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 2, 200);
    base->SetFixed(true);
    base->SetPos(ChVector3d(-0.5, 0, 0));
    modal_assembly->Add(base);

    auto root = chrono_types::make_shared<ChLinkMateGeneric>();
    root->Initialize(node_A, base, ChFrame<>(ChVector3d(0, 0, 1), QUNIT));
    modal_assembly->Add(root);

    // Load the last internal node
    auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(mesh_internal->GetNodes().back());
    std::shared_ptr<ChBody> body;
    if (mode == LoadMode::BODY_ACCUMULATOR || mode == LoadMode::BODY_FORCE) {
        body = chrono_types::make_shared<ChBody>();
        body->SetMass(10);
        body->SetInertiaXX(ChVector3d(1, 1, 1));
        body->SetPos(node->GetPos());
        body->SetRot(node->GetRot());
        modal_assembly->AddInternal(body);

        auto weld = chrono_types::make_shared<ChLinkMateFix>();
        weld->Initialize(node, body, ChFrame<>(node->GetPos(), node->GetRot()));
        modal_assembly->AddInternal(weld);
    }

    switch (mode) {
        case LoadMode::NODE:
        case LoadMode::FIXED_NODE:
            node->SetForce(force);
            node->SetTorque(torque);
            break;
        case LoadMode::ACCUMULATOR: {
            auto idx1 = node->AddAccumulator();
            auto idx2 = node->AddAccumulator();
            node->AccumulateForce(idx1, force, node->GetPos(), false);
            node->AccumulateTorque(idx2, torque, true);
            break;
        }
        case LoadMode::MIXED: {
            node->SetForce(0.5 * force);
            node->SetTorque(0.5 * torque);
            auto idx = node->AddAccumulator();
            node->AccumulateForce(idx, 0.5 * force, node->GetPos(), false);
            node->AccumulateTorque(idx, 0.5 * torque, true);
            break;
        }
        case LoadMode::BODY_ACCUMULATOR: {
            auto idx = body->AddAccumulator();
            body->AccumulateForce(idx, force, body->GetPos(), false);
            body->AccumulateTorque(idx, torque, true);
            break;
        }
        case LoadMode::BODY_FORCE: {
            auto body_force = chrono_types::make_shared<ChForce>();
            body->AddForce(body_force);
            body_force->SetMode(ChForce::ForceType::FORCE);
            body_force->SetFrame(ChForce::ReferenceFrame::BODY);
            body_force->SetAlign(ChForce::AlignmentFrame::WORLD_DIR);
            body_force->SetVrelpoint(VNULL);
            body_force->SetDir(force.GetNormalized());
            body_force->SetMforce(force.Length());

            auto body_torque = chrono_types::make_shared<ChForce>();
            body->AddForce(body_torque);
            body_torque->SetMode(ChForce::ForceType::TORQUE);
            body_torque->SetAlign(ChForce::AlignmentFrame::BODY_DIR);
            body_torque->SetRelDir(torque.GetNormalized());
            body_torque->SetMforce(torque.Length());
            break;
        }
    }

    auto eigen_solver = chrono_types::make_shared<ChUnsymGenEigenvalueSolverKrylovSchur>();
    ChModalSolverUndamped<ChUnsymGenEigenvalueSolverKrylovSchur> modal_solver(num_modes, 1e-5, true, false, eigen_solver);
    ChModalDampingRayleigh modal_damping(0.001, 0.1);
    modal_assembly->DoModalReduction(modal_solver, modal_damping);

    // Accumulated loads are kept constant (the accumulators are not emptied during the simulation)
    for (int i = 0; i < num_steps; i++)
        sys.DoStepDynamics(step_size);

    return node_B->GetPos();
}

TEST(ChModalAssembly, node_accumulators) {
    auto pos_node = RunModel(LoadMode::NODE);
    auto pos_acc = RunModel(LoadMode::ACCUMULATOR);
    auto pos_mixed = RunModel(LoadMode::MIXED);

    std::cout << "Free end position (node loads):  " << pos_node << std::endl;
    std::cout << "Free end position (accumulators): " << pos_acc << std::endl;
    std::cout << "Free end position (mixed):        " << pos_mixed << std::endl;

    // The applied loads must have an effect
    ASSERT_GT((pos_node - ChVector3d(6, 0, 0)).Length(), 1e-6);

    // Loads applied through accumulators must give the same results as node loads
    ASSERT_NEAR((pos_acc - pos_node).Length(), 0.0, 1e-10);
    ASSERT_NEAR((pos_mixed - pos_node).Length(), 0.0, 1e-10);
}

TEST(ChModalAssembly, fixed_internal_node) {
    auto pos_node = RunModel(LoadMode::NODE);
    auto pos_fixed = RunModel(LoadMode::FIXED_NODE);

    std::cout << "Free end position (node loads):            " << pos_node << std::endl;
    std::cout << "Free end position (with fixed extra node): " << pos_fixed << std::endl;

    // A fixed internal node must not change the loads applied on the other internal nodes
    ASSERT_NEAR((pos_fixed - pos_node).Length(), 0.0, 1e-10);
}

TEST(ChModalAssembly, body_forces) {
    auto pos_acc = RunModel(LoadMode::BODY_ACCUMULATOR);
    auto pos_force = RunModel(LoadMode::BODY_FORCE);

    std::cout << "Free end position (body accumulator): " << pos_acc << std::endl;
    std::cout << "Free end position (body ChForce):     " << pos_force << std::endl;

    // The applied loads must have an effect
    ASSERT_GT((pos_acc - ChVector3d(6, 0, 0)).Length(), 1e-6);

    // Loads applied through ChForce objects must give the same results as through an accumulator
    ASSERT_NEAR((pos_force - pos_acc).Length(), 0.0, 1e-10);
}
