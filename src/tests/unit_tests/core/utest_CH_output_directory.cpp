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
// Author: Radu Serban
// =============================================================================
//
// Unit test for CreateOutputDirectory.
//
// The main check is that concurrent attempts to create the same directory all
// succeed, as happens when several processes (e.g., participants in a
// co-simulation) create a shared output directory at startup. Threads within one
// process race at the file system level exactly as separate processes do.
//
// =============================================================================

#include <atomic>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "chrono/core/ChDataPath.h"

using namespace chrono;

static std::filesystem::path TestRoot() {
    return std::filesystem::path(GetChronoTestOutputPath()) / "output_directory";
}

// Several threads attempt to create the same directory at the same time; all must report success.
// The parent directory exists, so that the threads only race on creating the directory itself (missing parent
// directories are covered by a separate test).
TEST(CreateOutputDirectory, concurrent_creation) {
    const int num_threads = 16;
    const int num_rounds = 50;

    auto root = TestRoot() / "concurrent";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);

    for (int round = 0; round < num_rounds; round++) {
        auto dir = root / ("round_" + std::to_string(round));

        std::atomic<bool> start{false};
        std::atomic<int> ready{0};
        std::vector<char> results(num_threads, 0);
        std::vector<std::thread> threads;

        for (int i = 0; i < num_threads; i++) {
            threads.emplace_back([&, i]() {
                ready++;
                while (!start)
                    std::this_thread::yield();
                try {
                    results[i] = CreateOutputDirectory(dir) ? 1 : 0;
                } catch (...) {
                    results[i] = 0;  // an exception counts as a failure
                }
            });
        }

        // Release all threads at once, to maximize contention
        while (ready < num_threads)
            std::this_thread::yield();
        start = true;

        for (auto& t : threads)
            t.join();

        for (int i = 0; i < num_threads; i++)
            ASSERT_TRUE(results[i]) << "round " << round << ": thread " << i << " failed to create " << dir;
        ASSERT_TRUE(std::filesystem::is_directory(dir));
    }

    std::filesystem::remove_all(root);
}

// Creating a directory that already exists succeeds.
TEST(CreateOutputDirectory, existing_directory) {
    auto dir = TestRoot() / "existing";
    std::filesystem::remove_all(dir);

    ASSERT_TRUE(CreateOutputDirectory(dir));
    ASSERT_TRUE(CreateOutputDirectory(dir));
    ASSERT_TRUE(CreateOutputDirectory(dir.string()));
    ASSERT_TRUE(std::filesystem::is_directory(dir));

    std::filesystem::remove_all(dir);
}

// Missing parent directories are created.
TEST(CreateOutputDirectory, missing_parents) {
    auto root = TestRoot() / "parents";
    std::filesystem::remove_all(root);

    auto dir = root / "a" / "b" / "c";
    ASSERT_TRUE(CreateOutputDirectory(dir));
    ASSERT_TRUE(std::filesystem::is_directory(dir));

    std::filesystem::remove_all(root);
}

// A path that exists as a regular file is reported as a failure, without throwing.
TEST(CreateOutputDirectory, path_is_file) {
    auto root = TestRoot() / "file";
    std::filesystem::remove_all(root);
    ASSERT_TRUE(CreateOutputDirectory(root));

    auto file = root / "not_a_directory";
    std::ofstream(file) << "test";

    bool result = true;
    ASSERT_NO_THROW(result = CreateOutputDirectory(file));
    ASSERT_FALSE(result);
    ASSERT_NO_THROW(result = CreateOutputDirectory(file / "child"));
    ASSERT_FALSE(result);

    std::filesystem::remove_all(root);
}
