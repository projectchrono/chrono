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
// Benchmark test for sparse matrix setup (assembly of system matrix).
// This provides a measure of the effect and performance of using the "sparsity
// learner".
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/utils/ChBenchmark.h"

#include "chrono/core/ChCSMatrix.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/fea/ChElementShellANCF.h"
#include "chrono/fea/ChMesh.h"

#include "chrono_mkl/ChSolverMKL.h"

using namespace chrono;
using namespace chrono::fea;

template <int N>
class SystemFixture : public ::benchmark::Fixture {
  public:
    void SetUp(const ::benchmark::State& st) override {
        m_system = new ChSystemSMC();
        m_system->Set_G_acc(ChVector<>(0, -9.8, 0));

        m_solver = chrono_types::make_shared<ChSolverMKL<>>();
        m_solver->SetSparsityPatternLock(true);
        m_solver->SetVerbose(false);
        m_system->SetSolver(m_solver);

        // Mesh properties
        double length = 1;
        double width = 0.1;
        double thickness = 0.01;

        double rho = 500;
        ChVector<> E(2.1e7, 2.1e7, 2.1e7);
        ChVector<> nu(0.3, 0.3, 0.3);
        ChVector<> G(8.0769231e6, 8.0769231e6, 8.0769231e6);
        auto mat = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu, G);

        // Create mesh nodes and elements
        auto mesh = chrono_types::make_shared<ChMesh>();
        m_system->Add(mesh);

        int n_nodes = 2 * (1 + N);
        double dx = length / N;
        ChVector<> dir(0, 1, 0);

        auto nodeA = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(0, 0, -width / 2), dir);
        auto nodeB = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(0, 0, +width / 2), dir);
        nodeA->SetFixed(true);
        nodeB->SetFixed(true);
        mesh->AddNode(nodeA);
        mesh->AddNode(nodeB);

        for (int i = 1; i <= N; i++) {
            auto nodeC = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(i * dx, 0, -width / 2), dir);
            auto nodeD = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(i * dx, 0, +width / 2), dir);
            mesh->AddNode(nodeC);
            mesh->AddNode(nodeD);

            auto element = chrono_types::make_shared<ChElementShellANCF>();
            element->SetNodes(nodeA, nodeB, nodeD, nodeC);
            element->SetDimensions(dx, width);
            element->AddLayer(thickness, 0 * CH_C_DEG_TO_RAD, mat);
            element->SetAlphaDamp(0.0);
            element->SetGravityOn(false);
            mesh->AddElement(element);

            nodeA = nodeC;
            nodeB = nodeD;
        }
    }

    void TearDown(const ::benchmark::State&) override { delete m_system; }

    void Report(benchmark::State& st) {
        auto descr = m_system->GetSystemDescriptor();
        auto num_it = st.iterations();
        st.counters["SIZE"] = descr->CountActiveVariables() + descr->CountActiveConstraints();
        st.counters["LS_Jacobian"] = m_system->GetTimerJacobian() * 1e3 / num_it;
        st.counters["LS_Setup"] = m_system->GetTimerSetup() * 1e3 / num_it;
        st.counters["LS_Solve"] = m_system->GetTimerSolver() * 1e3 / num_it;
    }

    ChSystemSMC* m_system;
    std::shared_ptr<ChSolverMKL<>> m_solver;
};

#define BM_LEARNER(TEST_NAME, N)                                                      \
    BENCHMARK_TEMPLATE_DEFINE_F(SystemFixture, TEST_NAME, N)(benchmark::State & st) { \
        while (st.KeepRunning()) {                                                    \
            m_solver->ForceSparsityPatternUpdate(true);                               \
            m_system->DoStaticLinear();                                               \
        }                                                                             \
        Report(st);                                                                   \
    }                                                                                 \
    BENCHMARK_REGISTER_F(SystemFixture, TEST_NAME)->Unit(benchmark::kMillisecond);

#define BM_NO_LEARNER(TEST_NAME, N)                                                   \
    BENCHMARK_TEMPLATE_DEFINE_F(SystemFixture, TEST_NAME, N)(benchmark::State & st) { \
        while (st.KeepRunning()) {                                                    \
            m_system->DoStaticLinear();                                               \
        }                                                                             \
        Report(st);                                                                   \
    }                                                                                 \
    BENCHMARK_REGISTER_F(SystemFixture, TEST_NAME)->Unit(benchmark::kMillisecond);

BM_LEARNER(Learner_500, 500)
BM_NO_LEARNER(NoLearner_500, 500)

BM_LEARNER(Learner_1000, 1000)
BM_NO_LEARNER(NoLearner_1000, 1000)

BM_LEARNER(Learner_2000, 2000)
BM_NO_LEARNER(NoLearner_2000, 2000)

BM_LEARNER(Learner_4000, 4000)
BM_NO_LEARNER(NoLearner_4000, 4000)

BM_LEARNER(Learner_8000, 8000)
BM_NO_LEARNER(NoLearner_8000, 8000)
