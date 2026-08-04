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
// Authors: Dan Negrut
// =============================================================================
//
// Concurrent FIRST access to ChVisualMaterial::Default().
//
// This is one test in its own executable, and the reason is the whole point of the file. Default()
// initializes a static on first call, so the hazard exists only on the first call in a process.
// Chrono::Sensor reaches it from ChOptixEngine::ConstructScene(), and every engine has its own
// scene thread, so two engines coming up together can both be that first caller.
//
// Sharing a binary with the other default-material tests destroys the test. Those tests call
// Default() too, gtest runs them in declaration order, and by the time a concurrency test ran the
// singleton would already be built, so the threads would race over nothing. That is not a
// hypothesis: with the pre-fix implementation of Default() swapped back in, this test passed inside
// the shared binary and failed 200 out of 200 runs as its own binary. One test alone in a process
// is what makes it the first caller.
//
// What it catches: the pattern this change replaced, a namespace-scope pointer initialized under
// "if (!ptr) ptr = ...", which has no synchronization at all. C++11 guarantees thread-safe
// initialization of a function-local static instead, so the shipped code needs no lock and gets
// none. Measured on the old code: every run returned more than one distinct material, worst case 7
// of 8 threads holding a different object, and some runs aborted outright in the allocator, since
// racing shared_ptr assignments corrupt the reference count.
//
// =============================================================================

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/assets/ChVisualMaterial.h"

using namespace chrono;

TEST(DefaultVisualMaterialFirstAccess, concurrent_first_access_returns_one_object) {
    constexpr int kThreads = 16;
    std::vector<std::shared_ptr<ChVisualMaterial>> seen(kThreads);
    std::vector<std::thread> threads;
    threads.reserve(kThreads);

    // The gate is load-bearing. Spawning threads in a loop and letting each run as it is created
    // gives the first one time to finish initializing before the second starts, and then there is
    // no concurrent first access left to test. Every thread parks here until all of them exist.
    std::atomic<int> ready{0};
    std::atomic<bool> gate{false};

    for (int i = 0; i < kThreads; ++i) {
        threads.emplace_back([&seen, &ready, &gate, i]() {
            ready.fetch_add(1, std::memory_order_release);
            while (!gate.load(std::memory_order_acquire))
                std::this_thread::yield();
            seen[i] = ChVisualMaterial::Default();
        });
    }
    while (ready.load(std::memory_order_acquire) < kThreads)
        std::this_thread::yield();
    gate.store(true, std::memory_order_release);

    for (auto& t : threads)
        t.join();

    for (int i = 0; i < kThreads; ++i) {
        ASSERT_NE(seen[i], nullptr) << "thread " << i << " got null";
        EXPECT_EQ(seen[i], seen[0]) << "thread " << i << " got a different object";
    }
}
