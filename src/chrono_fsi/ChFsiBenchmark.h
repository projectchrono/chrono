// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Huzaifa Unjhawala, Radu Serban
// =============================================================================
//
// Utilities for performance benchmarking of Chrono::FSI simulations using the
// Google benchmark framework.
//
// =============================================================================

#ifndef CH_FSI_BENCHMARK_H
#define CH_FSI_BENCHMARK_H

#include "chrono/utils/ChBenchmark.h"
#include "chrono_fsi/ChFsiSystem.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_base
/// @{

/// Base class for a Chrono FSI benchmark test.
/// Extends ChBenchmarkTest with information from a CFD solver.S
class ChFsiBenchmarkTest : public utils::ChBenchmarkTest {
  public:
    ChFsiBenchmarkTest();
    virtual ~ChFsiBenchmarkTest() {}

    virtual ChFsiSystem* GetFsiSystem() = 0;
    virtual ChSystem* GetSystem() override final { return &GetFsiSystem()->GetMultibodySystem(); }

    virtual void ResetTimers() override;
    virtual void UpdateTimers() override;

    double m_timer_CFD;   ///< time for CFD
    double m_timer_MBS;   ///< time for MBS
};

inline ChFsiBenchmarkTest::ChFsiBenchmarkTest()
    : utils::ChBenchmarkTest(),
      m_timer_CFD(0),
      m_timer_MBS(0) {}

inline void ChFsiBenchmarkTest::ResetTimers() {
    utils::ChBenchmarkTest::ResetTimers();
    m_timer_step += GetFsiSystem()->GetTimerStep();  // overwrite with timer for MBS+CFD
    m_timer_CFD = 0;
    m_timer_MBS = 0;
}

inline void ChFsiBenchmarkTest::UpdateTimers() {
    utils::ChBenchmarkTest::UpdateTimers();
    m_timer_CFD += GetFsiSystem()->GetTimerCFD();
    m_timer_MBS += GetFsiSystem()->GetTimerMBD();
}

// =============================================================================

/// Define and register a test named TEST_NAME using the specified ChFsiBenchmark TEST.
/// This method benchmarks consecutive (in time) simulation batches and is therefore
/// appropriate for cases where the cost per step is expected to be relatively uniform.
/// An initial SKIP_STEPS integration steps are performed for hot start, after which
/// measurements are conducted for batches of SIM_STEPS integration steps.
/// The test is repeated REPETITIONS number of times, to collect statistics.
/// Note that each reported benchmark result may require simulations of several batches
/// (controlled by the benchmark library in order to stabilize timing results).
#define CH_FSI_BM_SIMULATION_LOOP(TEST_NAME, TEST, SKIP_STEPS, SIM_STEPS, REPETITIONS) \
    using TEST_NAME = chrono::fsi::ChFsiBenchmarkFixture<TEST, SKIP_STEPS>;            \
    BENCHMARK_DEFINE_F(TEST_NAME, SimulateLoop)(benchmark::State & st) {               \
        while (st.KeepRunning()) {                                                     \
            m_test->Simulate(SIM_STEPS);                                               \
        }                                                                              \
        Report(st);                                                                    \
    }                                                                                  \
    BENCHMARK_REGISTER_F(TEST_NAME, SimulateLoop)->Unit(benchmark::kMillisecond)->Repetitions(REPETITIONS);

/// Define and register a test named TEST_NAME using the specified ChBenchmark TEST.
/// This method benchmarks a single simulation interval and is appropriate for cases
/// where the cost of simulating a given length time interval can vary significantly
/// from interval to interval.
/// For each measurement, the underlying model is recreated from scratch. An initial
/// SKIP_STEPS integration steps are performed for hot start, after which a single
/// batch of SIM_STEPS is timed and recorded.
/// The test is repeated REPETITIONS number of times, to collect statistics.
#define CH_FSI_BM_SIMULATION_ONCE(TEST_NAME, TEST, SKIP_STEPS, SIM_STEPS, REPETITIONS) \
    using TEST_NAME = chrono::fsi::ChFsiBenchmarkFixture<TEST, 0>;                        \
    BENCHMARK_DEFINE_F(TEST_NAME, SimulateOnce)(benchmark::State & st) {               \
        Reset(SKIP_STEPS);                                                             \
        while (st.KeepRunning()) {                                                     \
            m_test->Simulate(SIM_STEPS);                                               \
        }                                                                              \
        Report(st);                                                                    \
    }                                                                                  \
    BENCHMARK_REGISTER_F(TEST_NAME, SimulateOnce)                                      \
        ->Unit(benchmark::kMillisecond)                                                \
        ->Iterations(1)                                                                \
        ->Repetitions(REPETITIONS);

// =============================================================================

/// Generic benchmark fixture for Chrono tests.
/// The first template parameter is a ChFsiBenchmarkTest.
/// The second template parameter is the initial number of simulation steps (hot start).
template <typename TEST, int SKIP>
class ChFsiBenchmarkFixture : public utils::ChBenchmarkFixture<TEST, SKIP> {
  public:
    ChFsiBenchmarkFixture() {}
    ~ChFsiBenchmarkFixture() {}

    virtual void Report(benchmark::State& st) override {
        utils::ChBenchmarkFixture<TEST, SKIP>::Report(st);
        st.counters["CFD_Total"] = this->m_test->m_timer_CFD * 1e3;
        st.counters["MBS_Total"] = this->m_test->m_timer_MBS * 1e3;
    }
};

/// @} fsi_base

}  // end namespace fsi
}  // end namespace chrono
#endif
