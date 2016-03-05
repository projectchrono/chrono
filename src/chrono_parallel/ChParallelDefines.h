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
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: lots of useful definitions for thrust, includes and enums
// =============================================================================

#pragma once

#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include <iostream>

#ifndef _MSC_VER
#include <fenv.h>
#endif
#include "chrono_parallel/ChApiParallel.h"
#include "chrono_parallel/ChConfigParallel.h"

typedef int shape_type;

#ifdef __CDT_PARSER__
#define BLAZE_SERIAL_SECTION
#define CH_PARALLEL_API
#define custom_vector std::vector
#else
#define custom_vector std::vector
#endif

#if defined _MSC_VER
class NullBuffer : public std::streambuf {
  public:
    int overflow(int c) { return c; }
};
static NullBuffer null_buffer;
static std::ostream null_stream(&null_buffer);
#define LOG(X) null_stream
#else
// Enable thread safe logging
#define ELPP_THREAD_SAFE
#include "thirdparty/easylogging/easylogging.h"
#define LOGGINGENABLED

#endif

#define Thrust_Inclusive_Scan_Sum(x, y)                    \
    thrust::inclusive_scan(x.begin(), x.end(), x.begin()); \
    y = x.back();
#define Thrust_Sort_By_Key(x, y) thrust::sort_by_key(x.begin(), x.end(), y.begin())

#define Run_Length_Encode(y, z, w)                                                                                 \
    (thrust::reduce_by_key(y.begin(), y.end(), thrust::constant_iterator<uint>(1), z.begin(), w.begin()).second) - \
        w.begin()

#define Thrust_Inclusive_Scan(x) thrust::inclusive_scan(x.begin(), x.end(), x.begin())
#define Thrust_Exclusive_Scan(x) thrust::exclusive_scan(x.begin(), x.end(), x.begin())
#define Thrust_Fill(x, y) thrust::fill(x.begin(), x.end(), y)
#define Thrust_Sort(x) thrust::sort(x.begin(), x.end())
#define Thrust_Count(x, y) thrust::count(x.begin(), x.end(), y)
#define Thrust_Sequence(x) thrust::sequence(x.begin(), x.end())
#define Thrust_Equal(x, y) thrust::equal(x.begin(), x.end(), y.begin())
#define Thrust_Max(x) x[thrust::max_element(x.begin(), x.end()) - x.begin()]
#define Thrust_Min(x) x[thrust::min_element(x.begin(), x.end()) - x.begin()]
#define Thrust_Total(x) thrust::reduce(x.begin(), x.end())
#define Thrust_Unique(x) thrust::unique(x.begin(), x.end()) - x.begin();
#define DBG(x) printf(x);

enum SOLVERTYPE {
    STEEPEST_DESCENT,
    GRADIENT_DESCENT,
    CONJUGATE_GRADIENT,
    CONJUGATE_GRADIENT_SQUARED,
    BICONJUGATE_GRADIENT,
    BICONJUGATE_GRADIENT_STAB,
    MINIMUM_RESIDUAL,
    QUASAI_MINIMUM_RESIDUAL,
    APGD,
    APGDREF,
    JACOBI,
    GAUSS_SEIDEL,
    PDIP,
    BB,
    SPGQP
};

enum SOLVERMODE { NORMAL, SLIDING, SPINNING, BILATERAL };

enum COLLISIONSYSTEMTYPE { COLLSYS_PARALLEL, COLLSYS_BULLET_PARALLEL };

enum NARROWPHASETYPE {
    NARROWPHASE_MPR,
    NARROWPHASE_R,
    NARROWPHASE_HYBRID_MPR,
};

// This is set so that parts of the code that have been "flattened" can know what
// type of system is used.
enum SYSTEMTYPE { SYSTEM_DVI, SYSTEM_DEM };

enum BILATERALTYPE { BODY_BODY, SHAFT_SHAFT, SHAFT_SHAFT_SHAFT, SHAFT_BODY, SHAFT_SHAFT_BODY, UNKNOWN };

// Supported Logging Levels
enum LOGGINGLEVEL { LOG_NONE, LOG_INFO, LOG_TRACE, LOG_WARNING, LOG_ERROR };

#ifdef __CUDACC__
#define CUDA_HOST_DEVICE __host__ __device__
#define CUDA_DEVICE __device__
#define CUDA_CONSTANT __device__ __constant__
#define CUDA_SHARED __shared__
#define CUDA_GLOBAL __global__
#else
#define CUDA_HOST_DEVICE
#define CUDA_DEVICE
#define CUDA_CONSTANT
#define CUDA_SHARED
#define CUDA_GLOBAL
#endif

#define max_neighbors 64
#define max_rigid_neighbors 32
