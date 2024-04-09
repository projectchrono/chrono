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

#include "chrono/core/ChMatrix.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChMesh.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_MUMPS
    #include "chrono_mumps/ChSolverMumps.h"
#endif

using namespace chrono;
using namespace chrono::fea;

template <int N>
class SystemFixture : public ::benchmark::Fixture {
  public:
    void SetUp(const ::benchmark::State& st) override {
        m_system = new ChSystemSMC();
        m_system->SetGravitationalAcceleration(ChVector3d(0, -9.8, 0));

        // Mesh properties
        double length = 1;
        double width = 0.1;
        double thickness = 0.01;

        double rho = 500;
        ChVector3d E(2.1e7, 2.1e7, 2.1e7);
        ChVector3d nu(0.3, 0.3, 0.3);
        ChVector3d G(8.0769231e6, 8.0769231e6, 8.0769231e6);
        auto mat = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu, G);

        // Create mesh nodes and elements
        auto mesh = chrono_types::make_shared<ChMesh>();
        m_system->Add(mesh);

        double dx = length / N;
        ChVector3d dir(0, 1, 0);

        auto nodeA = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(0, 0, -width / 2), dir);
        auto nodeB = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(0, 0, +width / 2), dir);
        nodeA->SetFixed(true);
        nodeB->SetFixed(true);
        mesh->AddNode(nodeA);
        mesh->AddNode(nodeB);

        for (int i = 1; i <= N; i++) {
            auto nodeC = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(i * dx, 0, -width / 2), dir);
            auto nodeD = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(i * dx, 0, +width / 2), dir);
            mesh->AddNode(nodeC);
            mesh->AddNode(nodeD);

            auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
            element->SetNodes(nodeA, nodeB, nodeD, nodeC);
            element->SetDimensions(dx, width);
            element->AddLayer(thickness, 0 * CH_DEG_TO_RAD, mat);
            element->SetAlphaDamp(0.0);
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
        st.counters["LS_Setup"] = m_system->GetTimerLSsetup() * 1e3 / num_it;
        st.counters["LS_Solve"] = m_system->GetTimerLSsolve() * 1e3 / num_it;

        auto solver = std::static_pointer_cast<ChDirectSolverLS>(m_system->GetSolver());
        st.counters["LS_Setup_assembly"] = solver->GetTimeSetup_Assembly() * 1e3 / num_it;
        st.counters["LS_Setup_call"] = solver->GetTimeSetup_SolverCall() * 1e3 / num_it;
        st.counters["LS_Solve_assembly"] = solver->GetTimeSolve_Assembly() * 1e3 / num_it;
        st.counters["LS_Solve_call"] = solver->GetTimeSolve_SolverCall() * 1e3 / num_it;
    }

  protected:
    ChSystemSMC* m_system;
};

#define BM_SOLVER_MKL(TEST_NAME, N, WITH_LEARNER)                                     \
    BENCHMARK_TEMPLATE_DEFINE_F(SystemFixture, TEST_NAME, N)(benchmark::State & st) { \
        auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();                \
        solver->UseSparsityPatternLearner(WITH_LEARNER);                              \
        solver->LockSparsityPattern(true);                                            \
        solver->SetVerbose(false);                                                    \
        m_system->SetSolver(solver);                                                  \
        while (st.KeepRunning()) {                                                    \
            solver->ForceSparsityPatternUpdate();                                     \
            m_system->DoStaticLinear();                                               \
        }                                                                             \
        Report(st);                                                                   \
    }                                                                                 \
    BENCHMARK_REGISTER_F(SystemFixture, TEST_NAME)->Unit(benchmark::kMillisecond);

#define BM_SOLVER_MUMPS(TEST_NAME, N, WITH_LEARNER)                                   \
    BENCHMARK_TEMPLATE_DEFINE_F(SystemFixture, TEST_NAME, N)(benchmark::State & st) { \
        auto solver = chrono_types::make_shared<ChSolverMumps>();                     \
        solver->UseSparsityPatternLearner(WITH_LEARNER);                              \
        solver->LockSparsityPattern(true);                                            \
        solver->SetVerbose(false);                                                    \
        m_system->SetSolver(solver);                                                  \
        while (st.KeepRunning()) {                                                    \
            solver->ForceSparsityPatternUpdate();                                     \
            m_system->DoStaticLinear();                                               \
        }                                                                             \
        Report(st);                                                                   \
    }                                                                                 \
    BENCHMARK_REGISTER_F(SystemFixture, TEST_NAME)->Unit(benchmark::kMillisecond);

#define BM_SOLVER_QR(TEST_NAME, N, WITH_LEARNER)                                      \
    BENCHMARK_TEMPLATE_DEFINE_F(SystemFixture, TEST_NAME, N)(benchmark::State & st) { \
        auto solver = chrono_types::make_shared<ChSolverSparseQR>();                  \
        solver->UseSparsityPatternLearner(WITH_LEARNER);                              \
        solver->LockSparsityPattern(true);                                            \
        solver->SetVerbose(false);                                                    \
        m_system->SetSolver(solver);                                                  \
        while (st.KeepRunning()) {                                                    \
            solver->ForceSparsityPatternUpdate();                                     \
            m_system->DoStaticLinear();                                               \
        }                                                                             \
        Report(st);                                                                   \
    }                                                                                 \
    BENCHMARK_REGISTER_F(SystemFixture, TEST_NAME)->Unit(benchmark::kMillisecond);

#ifdef CHRONO_PARDISO_MKL
BM_SOLVER_MKL(MKL_learner_500, 500, true)
BM_SOLVER_MKL(MKL_no_learner_500, 500, false)
BM_SOLVER_MKL(MKL_learner_1000, 1000, true)
BM_SOLVER_MKL(MKL_no_learner_1000, 1000, false)
BM_SOLVER_MKL(MKL_learner_2000, 2000, true)
BM_SOLVER_MKL(MKL_no_learner_2000, 2000, false)
BM_SOLVER_MKL(MKL_learner_4000, 4000, true)
BM_SOLVER_MKL(MKL_no_learner_4000, 4000, false)
BM_SOLVER_MKL(MKL_learner_8000, 8000, true)
BM_SOLVER_MKL(MKL_no_learner_8000, 8000, false)
#endif

#ifdef CHRONO_MUMPS
BM_SOLVER_MUMPS(MUMPS_learner_500, 500, true)
BM_SOLVER_MUMPS(MUMPS_no_learner_500, 500, false)
BM_SOLVER_MUMPS(MUMPS_learner_1000, 1000, true)
BM_SOLVER_MUMPS(MUMPS_no_learner_1000, 1000, false)
BM_SOLVER_MUMPS(MUMPS_learner_2000, 2000, true)
BM_SOLVER_MUMPS(MUMPS_no_learner_2000, 2000, false)
BM_SOLVER_MUMPS(MUMPS_learner_4000, 4000, true)
BM_SOLVER_MUMPS(MUMPS_no_learner_4000, 4000, false)
BM_SOLVER_MUMPS(MUMPS_learner_8000, 8000, true)
BM_SOLVER_MUMPS(MUMPS_no_learner_8000, 8000, false)
#endif

BM_SOLVER_QR(QR_learner_500, 500, true)
BM_SOLVER_QR(QR_no_learner_500, 500, false)
BM_SOLVER_QR(QR_learner_1000, 1000, true)
BM_SOLVER_QR(QR_no_learner_1000, 1000, false)
BM_SOLVER_QR(QR_learner_2000, 2000, true)
BM_SOLVER_QR(QR_no_learner_2000, 2000, false)
BM_SOLVER_QR(QR_learner_4000, 4000, true)
BM_SOLVER_QR(QR_no_learner_4000, 4000, false)
BM_SOLVER_QR(QR_learner_8000, 8000, true)
BM_SOLVER_QR(QR_no_learner_8000, 8000, false)

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);
    ::benchmark::RunSpecifiedBenchmarks();
}
