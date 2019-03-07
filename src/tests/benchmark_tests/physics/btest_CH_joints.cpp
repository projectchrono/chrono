// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
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
// Benchmark test for joint Update functions.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChBenchmark.h"

using namespace chrono;

// Benchmarking fixture: create system and add bodies

class LinkLockBM : public ::benchmark::Fixture {
  public:
    void SetUp(const ::benchmark::State& st) override {
        const int N = 100000;

        crt_time = 1;
        time_step = 0.1;

        sys = new ChSystemNSC();
        for (int i = 0; i < N + 1; i++) {
            auto body = std::make_shared<ChBody>();
            body->SetPos(ChVector<>(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0));
            sys->AddBody(body);
        }
        for (int i = 0; i < N; i++) {
            auto joint = std::make_shared<ChLinkLockRevolute>();
            auto b1 = sys->Get_bodylist()[i];
            auto b2 = sys->Get_bodylist()[i + 1];
            auto loc = 0.5 * (b1->GetPos() + b2->GetPos());
            joint->Initialize(b1, b2, ChCoordsys<>(loc, QUNIT));
            sys->AddLink(joint);
        }
    }

    void TearDown(const ::benchmark::State&) override { delete sys; }

    ChSystemNSC* sys;
    double crt_time;
    double time_step;
};

// Utility macros for benchmarking joint operations with different signatures

#define BM_LINK_OP_TIME(TEST_NAME, AS_TYPE, OP)                                    \
    BENCHMARK_DEFINE_F(LinkLockBM, TEST_NAME)(benchmark::State & st) {             \
        for (auto _ : st) {                                                        \
            for (auto link : sys->Get_linklist()) {                                \
                std::static_pointer_cast<ChLinkLock>(link)->AS_TYPE::OP(crt_time); \
            }                                                                      \
        }                                                                          \
        st.SetItemsProcessed(st.iterations() * sys->Get_linklist().size());        \
    }                                                                              \
    BENCHMARK_REGISTER_F(LinkLockBM, TEST_NAME)->Unit(benchmark::kMicrosecond);

#define BM_LINK_OP_VOID(TEST_NAME, AS_TYPE, OP)                             \
    BENCHMARK_DEFINE_F(LinkLockBM, TEST_NAME)(benchmark::State & st) {      \
        for (auto _ : st) {                                                 \
            for (auto link : sys->Get_linklist()) {                         \
                std::static_pointer_cast<ChLinkLock>(link)->AS_TYPE::OP();  \
            }                                                               \
        }                                                                   \
        st.SetItemsProcessed(st.iterations() * sys->Get_linklist().size()); \
    }                                                                       \
    BENCHMARK_REGISTER_F(LinkLockBM, TEST_NAME)->Unit(benchmark::kMicrosecond);

// Benchmark individual operations

BM_LINK_OP_TIME(UpdateTime_LinkMarkers, ChLinkMarkers, UpdateTime)
BM_LINK_OP_TIME(UpdateTime_LinkLock, ChLinkLock, UpdateTime)

BM_LINK_OP_TIME(UpdateForces_LinkMarkers, ChLinkMarkers, UpdateForces)
BM_LINK_OP_TIME(UpdateForces_LinkLock, ChLinkLock, UpdateForces)

BM_LINK_OP_VOID(UpdateRelMarkerCoords_LinkMarkers, ChLinkMarkers, UpdateRelMarkerCoords)

BM_LINK_OP_VOID(UpdateState_LinkLock, ChLinkLock, UpdateState)

BM_LINK_OP_TIME(Update_LinkMarkers, ChLinkMarkers, Update)
BM_LINK_OP_TIME(Update_LinkLock, ChLinkLock, Update)

// Main function

BENCHMARK_MAIN();
