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
// Check that loads applied through FEA node accumulators on internal nodes of a
// modal assembly are accounted for in the reduced model, exactly like the force
// and torque set directly on the nodes.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChBodyEasy.h"
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

// How the loads are applied to the internal node.
enum class LoadMode {
    NODE,         // node force and torque (SetForce, SetTorque)
    ACCUMULATOR,  // two node accumulators
    MIXED         // half through the node force and torque, half through an accumulator
};

// Simulate a modal-reduced cantilever beam with loads applied at its last internal node.
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
    switch (mode) {
        case LoadMode::NODE:
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
