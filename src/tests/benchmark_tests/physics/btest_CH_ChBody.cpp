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

#include <iostream>
#include <benchmark/benchmark.h>

#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChSystemNSC.h"

using namespace chrono;

// Benchmarking fixture: create system and add bodies
class SystemFixture : public ::benchmark::Fixture {
  public:
    void SetUp(const ::benchmark::State& st) override {
        const int num_bodies = 100000;
        current_time = 1;
        time_step = 0.1;
        sys = new ChSystemNSC();
        for (int i = 0; i < num_bodies; i++) {
            auto body = chrono_types::make_shared<ChBody>();
            body->SetPos(ChVector3d(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0));
            sys->AddBody(body);
        }
    }

    void TearDown(const ::benchmark::State&) override { delete sys; }

    ChSystemNSC* sys;
    double current_time;
    double time_step;
};

// Utility macros for benchmarking body operations with different signatures
#define BM_BODY_OP_TIME(OP)                                              \
    BENCHMARK_DEFINE_F(SystemFixture, OP)(benchmark::State & st) {       \
        for (auto _ : st) {                                              \
            for (auto body : sys->GetBodies()) {                         \
                body->OP(current_time, false);                           \
            }                                                            \
        }                                                                \
        st.SetItemsProcessed(st.iterations() * sys->GetBodies().size()); \
    }                                                                    \
    BENCHMARK_REGISTER_F(SystemFixture, OP)->Unit(benchmark::kMicrosecond);

#define BM_BODY_OP_VOID(OP)                                              \
    BENCHMARK_DEFINE_F(SystemFixture, OP)(benchmark::State & st) {       \
        for (auto _ : st) {                                              \
            for (auto body : sys->GetBodies()) {                         \
                body->OP();                                              \
            }                                                            \
        }                                                                \
        st.SetItemsProcessed(st.iterations() * sys->GetBodies().size()); \
    }                                                                    \
    BENCHMARK_REGISTER_F(SystemFixture, OP)->Unit(benchmark::kMicrosecond);

#define BM_BODY_OP_STEP(OP)                                              \
    BENCHMARK_DEFINE_F(SystemFixture, OP)(benchmark::State & st) {       \
        for (auto _ : st) {                                              \
            for (auto body : sys->GetBodies()) {                         \
                body->OP(time_step);                                     \
            }                                                            \
        }                                                                \
        st.SetItemsProcessed(st.iterations() * sys->GetBodies().size()); \
    }                                                                    \
    BENCHMARK_REGISTER_F(SystemFixture, OP)->Unit(benchmark::kMicrosecond);

// Benchmark individual operations
BM_BODY_OP_TIME(Update)
BM_BODY_OP_TIME(UpdateForces)
BM_BODY_OP_TIME(UpdateMarkers)
BM_BODY_OP_VOID(ClampSpeed)
BM_BODY_OP_VOID(ComputeGyro)

// Benchmark all operations in a single loop
BENCHMARK_DEFINE_F(SystemFixture, SingleLoop)(benchmark::State& st) {
    for (auto _ : st) {
        for (auto body : sys->GetBodies()) {
            body->Update(current_time, false);
            body->UpdateForces(current_time, false);
            body->UpdateMarkers(current_time, false);
            body->ClampSpeed();
            body->ComputeGyro();
        }
    }
    st.SetItemsProcessed(st.iterations() * sys->GetBodies().size());
}
BENCHMARK_REGISTER_F(SystemFixture, SingleLoop)->Unit(benchmark::kMicrosecond);
////BENCHMARK_REGISTER_F(SystemFixture, SingleLoop)->Unit(benchmark::kMicrosecond)->Iterations(1);

////BENCHMARK_MAIN();
