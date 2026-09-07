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
// Authors: Chrono contributors
// =============================================================================
//
// Shared reporting for the SCM scaling benchmarks:
//   btest_VEH_wheelSCM   (small)
//   btest_VEH_hmmwvSCM   (medium)
//   btest_VEH_largeSCM   (large)
//
// The total step time these tests report is not, on its own, a useful number for
// SCM work. In the small case most of it is the constraint solver, and a change
// that moved SCM cost by 30% would be inside the run-to-run spread of the solver.
// So each test also accumulates SCM's own per-step timers over the timed window
// and reports them as counters, in milliseconds per step.
//
// SCMTerrain resets its timers at the top of every step, so they have to be read
// and summed after each Advance -- which is what ScmStats::Accumulate does.
//
// =============================================================================

#ifndef SCM_BENCHMARK_UTILS_H
#define SCM_BENCHMARK_UTILS_H

#include <benchmark/benchmark.h>

#include "chrono/utils/ChBenchmark.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

namespace scm_bench {

/// Per-step SCM timings, summed over a run.
struct ScmStats {
    double active_domains = 0;
    double ray_casting = 0;
    double contact_patches = 0;
    double contact_forces = 0;
    double bulldozing = 0;
    double vis_update = 0;
    double ray_casts = 0;
    double contact_patch_count = 0;
    long long steps = 0;

    void Reset() { *this = ScmStats(); }

    /// Call once per step, after SCMTerrain::Advance.
    void Accumulate(const chrono::vehicle::SCMTerrain& terrain) {
        active_domains += terrain.GetTimerActiveDomains();
        ray_casting += terrain.GetTimerRayCasting();
        contact_patches += terrain.GetTimerContactPatches();
        contact_forces += terrain.GetTimerContactForces();
        bulldozing += terrain.GetTimerBulldozing();
        vis_update += terrain.GetTimerVisUpdate();
        ray_casts += terrain.GetNumRayCasts();
        contact_patch_count += terrain.GetNumContactPatches();
        steps++;
    }

    /// Report as milliseconds per step (counts as per-step averages).
    void Report(benchmark::State& st, const chrono::vehicle::SCMTerrain& terrain) const {
        double n = (steps > 0) ? (double)steps : 1.0;
        double total = active_domains + ray_casting + contact_patches + contact_forces + bulldozing + vis_update;

        st.counters["SCM_Total"] = total / n;
        st.counters["SCM_Domains"] = active_domains / n;
        st.counters["SCM_RayCast"] = ray_casting / n;
        st.counters["SCM_Patches"] = contact_patches / n;
        st.counters["SCM_Forces"] = contact_forces / n;
        st.counters["SCM_Bulldoze"] = bulldozing / n;
        st.counters["SCM_VisUpd"] = vis_update / n;
        st.counters["SCM_Rays"] = ray_casts / n;

        // The quantity the node-storage scheme is judged on: entries in the modified-node map.
        // O(map size), so read once, here, and never inside the timed window.
        st.counters["SCM_Nodes"] = (double)terrain.GetModifiedNodes(true).size();
    }
};

}  // namespace scm_bench

/// Same contract as CH_BM_SIMULATION_ONCE, plus the SCM counters above.
/// TEST must expose `scm_bench::ScmStats m_scm` and `const SCMTerrain& GetTerrain() const`.
#define CH_BM_SCM_SIMULATION_ONCE(TEST_NAME, TEST, SKIP_STEPS, SIM_STEPS, REPETITIONS) \
    using TEST_NAME = chrono::utils::ChBenchmarkFixture<TEST, 0>;                      \
    BENCHMARK_DEFINE_F(TEST_NAME, SimulateOnce)(benchmark::State & st) {               \
        Reset(SKIP_STEPS);                                                             \
        m_test->m_scm.Reset(); /* discard the hot-start window */                      \
        while (st.KeepRunning()) {                                                     \
            m_test->Simulate(SIM_STEPS);                                               \
        }                                                                              \
        Report(st);                                                                    \
        m_test->m_scm.Report(st, m_test->GetTerrain());                                \
    }                                                                                  \
    BENCHMARK_REGISTER_F(TEST_NAME, SimulateOnce)                                      \
        ->Unit(benchmark::kMillisecond)                                                \
        ->Iterations(1)                                                                \
        ->Repetitions(REPETITIONS);

#endif
