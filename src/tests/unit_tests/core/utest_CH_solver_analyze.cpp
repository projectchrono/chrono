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
// Tests for skipping the analyze phase of direct sparse linear solvers.
//
// The implicit integrators request an analysis of the system matrix (fill-reducing
// ordering and symbolic factorization) only if the system was flagged as modified.
// The direct solvers also perform one whenever the matrix size or its number of
// nonzeros changed since the last analysis.
//
// The model is an ANCF cable hinged to the ground, with one rigid body attached to
// an interior node, one attached to the tip node, and one unconnected body that is
// initially fixed. Each run is compared with a run in which the system is flagged as
// modified at every step (which forces an analysis at every step):
// - without structural changes, only the first step performs an analysis;
// - fixing the tip body during the run changes the problem size;
// - fixing the tip body and releasing the unconnected body keeps the problem size
//   but changes the sparsity pattern.
// The structural changes are not flagged, so the direct solver must detect them.
// In all cases, the results must match those of the run with an analysis at every step.
//
// =============================================================================

#include <algorithm>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/fea/ChBeamSectionCableANCF.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChMesh.h"

using namespace chrono;
using namespace chrono::fea;

namespace {

const unsigned int num_steps = 40;
const unsigned int change_step = 5;
const double step_size = 1e-3;

// Sparse LU solver that counts the analyze phases performed during factorization.
class CountingSparseLU : public ChDirectSolverLS {
  public:
    virtual Type GetType() const override { return Type::CUSTOM; }

    unsigned int num_analyze = 0;

  private:
    virtual bool FactorizeMatrix(bool analyze) override {
        if (analyze) {
            m_engine.analyzePattern(m_mat);
            num_analyze++;
        }
        m_engine.factorize(m_mat);
        return m_engine.info() == Eigen::Success;
    }

    virtual bool SolveSystem() override {
        m_sol = m_engine.solve(m_rhs);
        return m_engine.info() == Eigen::Success;
    }

    virtual void PrintErrorMessage() override {}

    Eigen::SparseLU<ChSparseMatrix, Eigen::COLAMDOrdering<int>> m_engine;
};

// Structural change made during the run, without flagging the system as modified.
enum class Change {
    NONE,    // no change
    SIZE,    // fix the tip body
    PATTERN  // fix the tip body and release the unconnected body
};

struct Result {
    std::vector<ChVector3d> pos;  // final positions of nodes and bodies
    unsigned int num_analyze;     // number of analyze phases
};

std::shared_ptr<ChBody> CreateBody(ChSystem& sys, const ChVector3d& pos) {
    auto body = chrono_types::make_shared<ChBody>();
    body->SetMass(0.05);
    body->SetInertiaXX(ChVector3d(1e-4, 1e-4, 1e-4));
    body->SetPos(pos);
    sys.AddBody(body);
    return body;
}

Result Run(ChTimestepper::Type type, Change change, bool analyze_every_step) {
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, -9.8, 0));

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.AddBody(ground);

    // Bodies are created first, so that they precede the mesh nodes in the system matrix.
    // When the tip body is fixed and the unconnected body released, the interior body moves in the matrix.
    const int num_elements = 8;
    const double length = 1.0;
    auto free_body = CreateBody(sys, ChVector3d(0, -1, 0));
    auto mid_body = CreateBody(sys, ChVector3d(length / 2, 0, 0));
    auto tip_body = CreateBody(sys, ChVector3d(length, 0, 0));
    free_body->SetFixed(true);

    auto section = chrono_types::make_shared<ChBeamSectionCableANCF>();
    section->SetDiameter(0.01);
    section->SetYoungModulus(1e7);
    section->SetDensity(1000);

    auto mesh = chrono_types::make_shared<ChMesh>();
    std::vector<std::shared_ptr<ChNodeFEAxyzD>> nodes;
    for (int i = 0; i <= num_elements; i++) {
        auto node = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(i * length / num_elements, 0, 0), ChVector3d(1, 0, 0));
        mesh->AddNode(node);
        nodes.push_back(node);
    }
    for (int i = 0; i < num_elements; i++) {
        auto element = chrono_types::make_shared<ChElementCableANCF>();
        element->SetNodes(nodes[i], nodes[i + 1]);
        element->SetSection(section);
        mesh->AddElement(element);
    }
    sys.Add(mesh);

    auto hinge = chrono_types::make_shared<ChLinkNodeFrame>();
    hinge->Initialize(nodes.front(), ground);
    sys.Add(hinge);

    auto mid_link = chrono_types::make_shared<ChLinkNodeFrame>();
    mid_link->Initialize(nodes[num_elements / 2], mid_body);
    sys.Add(mid_link);

    auto tip_link = chrono_types::make_shared<ChLinkNodeFrame>();
    tip_link->Initialize(nodes.back(), tip_body);
    sys.Add(tip_link);

    auto solver = chrono_types::make_shared<CountingSparseLU>();
    sys.SetSolver(solver);
    sys.SetTimestepperType(type);

    for (unsigned int i = 0; i < num_steps; i++) {
        if (i == change_step && change != Change::NONE) {
            tip_body->SetFixed(true);
            if (change == Change::PATTERN)
                free_body->SetFixed(false);
        }
        if (analyze_every_step)
            sys.ForceUpdate();
        sys.DoStepDynamics(step_size);
    }

    Result result;
    for (const auto& node : nodes)
        result.pos.push_back(node->GetPos());
    result.pos.push_back(free_body->GetPos());
    result.pos.push_back(mid_body->GetPos());
    result.pos.push_back(tip_body->GetPos());
    result.num_analyze = solver->num_analyze;

    return result;
}

void Check(ChTimestepper::Type type, Change change, unsigned int expected_num_analyze) {
    auto ref = Run(type, change, true);
    auto res = Run(type, change, false);

    EXPECT_EQ(ref.num_analyze, num_steps);
    EXPECT_EQ(res.num_analyze, expected_num_analyze);

    double max_diff = 0;
    for (size_t i = 0; i < ref.pos.size(); i++)
        max_diff = std::max(max_diff, (res.pos[i] - ref.pos[i]).Length());
    EXPECT_LT(max_diff, 1e-10);
}

}  // namespace

class SolverAnalyze : public ::testing::TestWithParam<ChTimestepper::Type> {};

TEST_P(SolverAnalyze, no_structural_change) {
    Check(GetParam(), Change::NONE, 1);
}

TEST_P(SolverAnalyze, unflagged_size_change) {
    Check(GetParam(), Change::SIZE, 2);
}

TEST_P(SolverAnalyze, unflagged_pattern_change) {
    Check(GetParam(), Change::PATTERN, 2);
}

INSTANTIATE_TEST_SUITE_P(ImplicitIntegrators,
                         SolverAnalyze,
                         ::testing::Values(ChTimestepper::Type::EULER_IMPLICIT,
                                           ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED,
                                           ChTimestepper::Type::EULER_IMPLICIT_PROJECTED,
                                           ChTimestepper::Type::TRAPEZOIDAL,
                                           ChTimestepper::Type::TRAPEZOIDAL_LINEARIZED,
                                           ChTimestepper::Type::NEWMARK,
                                           ChTimestepper::Type::HHT),
                         [](const ::testing::TestParamInfo<ChTimestepper::Type>& info) { return ChTimestepper::GetTypeAsString(info.param); });
