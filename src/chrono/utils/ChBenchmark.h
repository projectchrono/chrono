// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Utilities for performance benchmarking of Chrono simulations using the Google
// benchmark framework.
//
// =============================================================================

#ifndef CH_BENCHMARK_H
#define CH_BENCHMARK_H

#include "chrono_thirdparty/googlebenchmark/include/benchmark/benchmark.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace utils {

/// @addtogroup chrono_utils
/// @{

/// Base class for a Chrono benchmark test.
/// A derived class should set up a complete Chrono model in its constructor and implement
/// GetSystem (to return a pointer to the underlying Chrono system) and ExecuteStep (to perform
/// all operations required to advance the system state by one time step).
/// Timing information for various phases of the simulation is collected for a sequence of steps.
class ChBenchmarkTest {
  public:
    ChBenchmarkTest();
    virtual ~ChBenchmarkTest() {}

    virtual void ExecuteStep() = 0;
    virtual ChSystem* GetSystem() = 0;

    void Simulate(int num_steps);
    void ResetTimers();

    double m_timer_step;              ///< time for performing simulation
    double m_timer_advance;           ///< time for integration
    double m_timer_jacobian;          ///< time for evaluating/loading Jacobian data
    double m_timer_ls_setup;          ///< time for solver setup
    double m_timer_ls_solve;          ///< time for solver solve
    double m_timer_collision;         ///< time for collision detection
    double m_timer_collision_broad;   ///< time for broad-phase collision
    double m_timer_collision_narrow;  ///< time for narrow-phase collision
    double m_timer_setup;             ///< time for system update
    double m_timer_update;            ///< time for system update
};

inline ChBenchmarkTest::ChBenchmarkTest()
    : m_timer_step(0),
      m_timer_advance(0),
      m_timer_jacobian(0),
      m_timer_ls_setup(0),
      m_timer_ls_solve(0),
      m_timer_collision(0),
      m_timer_collision_broad(0),
      m_timer_collision_narrow(0),
      m_timer_setup(0),
      m_timer_update(0) {}

inline void ChBenchmarkTest::Simulate(int num_steps) {
    ////std::cout << "  simulate from t=" << GetSystem()->GetChTime() << " for steps=" << num_steps << std::endl;
    ResetTimers();
    for (int i = 0; i < num_steps; i++) {
        ExecuteStep();
        m_timer_step += GetSystem()->GetTimerStep();
        m_timer_advance += GetSystem()->GetTimerAdvance();
        m_timer_jacobian += GetSystem()->GetTimerJacobian();
        m_timer_ls_setup += GetSystem()->GetTimerLSsetup();
        m_timer_ls_solve += GetSystem()->GetTimerLSsolve();
        m_timer_collision += GetSystem()->GetTimerCollision();
        m_timer_collision_broad += GetSystem()->GetTimerCollisionBroad();
        m_timer_collision_narrow += GetSystem()->GetTimerCollisionNarrow();
        m_timer_setup += GetSystem()->GetTimerSetup();
        m_timer_update += GetSystem()->GetTimerUpdate();
    }
}

inline void ChBenchmarkTest::ResetTimers() {
    m_timer_step = 0;
    m_timer_advance = 0;
    m_timer_jacobian = 0;
    m_timer_ls_setup = 0;
    m_timer_ls_solve = 0;
    m_timer_collision = 0;
    m_timer_collision_broad = 0;
    m_timer_collision_narrow = 0;
    m_timer_setup = 0;
    m_timer_update = 0;
}

// =============================================================================

/// Define and register a test named TEST_NAME using the specified ChBenchmark TEST.
/// This method benchmarks consecutive (in time) simulation batches and is therefore
/// appropriate for cases where the cost per step is expected to be relatively uniform.
/// An initial SKIP_STEPS integration steps are performed for hot start, after which
/// measurements are conducted for batches of SIM_STEPS integration steps.
/// The test is repeated REPETITIONS number of times, to collect statistics.
/// Note that each reported benchmark result may require simulations of several batches
/// (controlled by the benchmark library in order to stabilize timing results).
#define CH_BM_SIMULATION_LOOP(TEST_NAME, TEST, SKIP_STEPS, SIM_STEPS, REPETITIONS) \
    using TEST_NAME = chrono::utils::ChBenchmarkFixture<TEST, SKIP_STEPS>;         \
    BENCHMARK_DEFINE_F(TEST_NAME, SimulateLoop)(benchmark::State & st) {           \
        while (st.KeepRunning()) {                                                 \
            m_test->Simulate(SIM_STEPS);                                           \
        }                                                                          \
        Report(st);                                                                \
    }                                                                              \
    BENCHMARK_REGISTER_F(TEST_NAME, SimulateLoop)->Unit(benchmark::kMillisecond)->Repetitions(REPETITIONS);

/// Define and register a test named TEST_NAME using the specified ChBenchmark TEST.
/// This method benchmarks a single simulation interval and is appropriate for cases
/// where the cost of simulating a given length time interval can vary significantly
/// from interval to interval.
/// For each measurement, the underlying model is recreated from scratch. An initial
/// SKIP_STEPS integration steps are performed for hot start, after which a single
/// batch of SIM_STEPS is timed and recorded.
/// The test is repeated REPETITIONS number of times, to collect statistics.
#define CH_BM_SIMULATION_ONCE(TEST_NAME, TEST, SKIP_STEPS, SIM_STEPS, REPETITIONS) \
    using TEST_NAME = chrono::utils::ChBenchmarkFixture<TEST, 0>;                  \
    BENCHMARK_DEFINE_F(TEST_NAME, SimulateOnce)(benchmark::State & st) {           \
        Reset(SKIP_STEPS);                                                         \
        while (st.KeepRunning()) {                                                 \
            m_test->Simulate(SIM_STEPS);                                           \
        }                                                                          \
        Report(st);                                                                \
    }                                                                              \
    BENCHMARK_REGISTER_F(TEST_NAME, SimulateOnce)                                  \
        ->Unit(benchmark::kMillisecond)                                            \
        ->Iterations(1)                                                            \
        ->Repetitions(REPETITIONS);

// =============================================================================

/// Generic benchmark fixture for Chrono tests.
/// The first template parameter is a ChBenchmarkTest.
/// The second template parameter is the initial number of simulation steps (hot start).
template <typename TEST, int SKIP>
class ChBenchmarkFixture : public ::benchmark::Fixture {
  public:
    ChBenchmarkFixture() : m_test(nullptr) {
        ////std::cout << "CREATE TEST" << std::endl;
        if (SKIP != 0) {
            m_test = new TEST();
            m_test->Simulate(SKIP);
        }
    }

    ~ChBenchmarkFixture() { delete m_test; }

    void Report(benchmark::State& st) {
        st.counters["Step_Total"] = m_test->m_timer_step * 1e3;
        st.counters["Step_Advance"] = m_test->m_timer_advance * 1e3;
        st.counters["Step Setup"] = m_test->m_timer_setup * 1e3;
        st.counters["Step_Update"] = m_test->m_timer_update * 1e3;
        st.counters["LS_Jacobian"] = m_test->m_timer_jacobian * 1e3;
        st.counters["LS_Setup"] = m_test->m_timer_ls_setup * 1e3;
        st.counters["LS_Solve"] = m_test->m_timer_ls_solve * 1e3;
        st.counters["CD_Total"] = m_test->m_timer_collision * 1e3;
        st.counters["CD_Broad"] = m_test->m_timer_collision_broad * 1e3;
        st.counters["CD_Narrow"] = m_test->m_timer_collision_narrow * 1e3;
    }

    void Reset(int num_init_steps) {
        ////std::cout << "RESET" << std::endl;
        delete m_test;
        m_test = new TEST();
        m_test->Simulate(num_init_steps);
    }

    TEST* m_test;
};

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
