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
//
// Unit tests for the ChOMP wrappers. These must behave like the OpenMP runtime
// functions outside a parallel region, with or without OpenMP support.
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono/utils/ChOpenMP.h"

using namespace chrono;

TEST(ChOMP, outside_parallel_region) {
    // Thread numbers are 0-based: the only thread outside a parallel region is thread 0
    EXPECT_EQ(ChOMP::GetThreadNum(), 0);
    EXPECT_EQ(ChOMP::GetNumThreads(), 1);
    EXPECT_GE(ChOMP::GetNumProcs(), 1);
    EXPECT_GE(ChOMP::GetMaxThreads(), 1);
}
