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
// Authors: Conlain Kelly, Nic Olsen, Dan Negrut
// =============================================================================

#pragma once

#include <climits>
#include <cuda_runtime.h>
#include <cstdio>
#include <cstdlib>

typedef longlong3 int64_t3;

constexpr size_t BD_WALL_ID_X_BOT = 0;
constexpr size_t BD_WALL_ID_X_TOP = 1;
constexpr size_t BD_WALL_ID_Y_BOT = 2;
constexpr size_t BD_WALL_ID_Y_TOP = 3;
constexpr size_t BD_WALL_ID_Z_BOT = 4;
constexpr size_t BD_WALL_ID_Z_TOP = 5;

constexpr size_t NUM_RESERVED_BC_IDS = 6;

/// At most 8 domains are touched by a sphere
#define MAX_SDs_TOUCHED_BY_SPHERE 8
/// At most 8 domains are touched by a sphere
#define MAX_SPHERES_TOUCHED_BY_SPHERE 12
/// The L-size of an SD should contain, on average, about these many spheres
#define AVERAGE_SPHERES_PER_SD_X_DIR 3.5
/// The D-size of an SD should contain, on average, about these many spheres
#define AVERAGE_SPHERES_PER_SD_Y_DIR 3.5
/// The H-size of an SD should contain, on average, about these many spheres
#define AVERAGE_SPHERES_PER_SD_Z_DIR 3.5
/// Anticipated max number of DEs in an SD; used for setting aside memory ahead of time
#define MAX_COUNT_OF_SPHERES_PER_SD 256
/// Value that indicates non-valid ID. The assumption is that an ID is always a positive integer
#define NULL_GRANULAR_ID UINT_MAX
/// Value that controls the length unit. A sphere deforms this many simulation length units under its own weight.
#define PSI_L_DEFAULT 16
/// Value that controls the time unit -- safety factor on the deformation-based time unit
#define PSI_T_DEFAULT 32
/// Maximum number of triangles that an SD can touch
#define MAX_TRIANGLE_COUNT_PER_SD 512u
/// Number of threads in a block when that number is allowed to vary.
#define CUDA_THREADS_PER_BLOCK 128

// NOTE this may change in the future, but until then this is sufficient
constexpr int warp_size = 32;

/// Set up some error checking mechanism to ensure CUDA didn't complain about things.
///  This approach suggested <a
/// href="https://stackoverflow.com/questions/14038589/what-is-the-canonical-way-to-check-for-errors-using-the-cuda-runtime-api">elsewhere</a>.
///  Some nice suggestions for how to use the mechanism are provided at the above link.
///
#define gpuErrchk(ans) \
    { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char* file, int line, bool abort = true) {
    if (code != cudaSuccess) {
        fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
        if (abort)
            exit(code);
    }
}

// Add verbose checks easily
#define INFO_PRINTF(...)                             \
    if (verbosity == INFO || verbosity == METRICS) { \
        printf(__VA_ARGS__);                         \
    }

#define METRICS_PRINTF(...)     \
    if (verbosity == METRICS) { \
        printf(__VA_ARGS__);    \
    }

#define CONDITIONAL_PRINTF(do_print, ...) \
    if (do_print) {                       \
        printf(__VA_ARGS__);              \
    }
