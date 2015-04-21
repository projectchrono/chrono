// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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

#ifndef CHPARALLELDEFINES_H
#define CHPARALLELDEFINES_H

#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#ifndef _MSC_VER
#include <fenv.h>
#endif
#include "chrono_parallel/ChApiParallel.h"
#include "chrono_parallel/ChConfigParallel.h"

#include <thrust/execution_policy.h>
#include <thrust/system/omp/execution_policy.h>

#ifdef _MSC_VER
#define thrust_parallel thrust::cpp::par
#else
#define thrust_parallel thrust::omp::par
#endif

typedef int shape_type;

#ifdef __CDT_PARSER__
#define BLAZE_SERIAL_SECTION
//#define __host__
//#define __device__
//#define __global__
//#define __constant__
//#define __shared__
//#define CUDA_KERNEL_DIM(...) ()
//#define __KERNEL__(...) ()
//#else
//#define CUDA_KERNEL_DIM(...)  <<< __VA_ARGS__ >>>
//#define __KERNEL__(...)  <<< __VA_ARGS__ >>>
#endif

//#define SIM_ENABLE_GPU_MODE
#ifdef SIM_ENABLE_GPU_MODE
#define custom_vector thrust::device_vector
#else
#ifndef __CDT_PARSER__
#define custom_vector thrust::host_vector
#else
using namespace thrust;
#define custom_vector host_vector
#endif
#endif

#ifndef _MSC_VER

// Enable thread safe logging
#define ELPP_THREAD_SAFE
#include "third_party/easylogging/easylogging.h"
#define LOGGINGENABLED
#else
class NullBuffer : public std::streambuf {
 public:
  int overflow(int c) { return c; }
};
static NullBuffer null_buffer;
static std::ostream null_stream(&null_buffer);
#define LOG(X) null_stream
#endif



#define CHVECCAST(v) ChVector<>(v.x, v.y, v.z)
#define CHQUATCAST(q) ChQuaternion<>(q.w, q.x, q.y, q.z)

#define Thrust_Inclusive_Scan_Sum(x, y)                  \
  thrust::inclusive_scan(x.begin(), x.end(), x.begin()); \
  y = x.back();
#define Thrust_Sort_By_Key(x, y) thrust::sort_by_key(x.begin(), x.end(), y.begin())
#define Thrust_Reduce_By_KeyA(x, y, z)                                                                               \
  x = (thrust::reduce_by_key(y.begin(), y.end(), thrust::constant_iterator<uint>(1), y.begin(), z.begin()).second) - \
      z.begin()

#define Thrust_Reduce_By_Key(y, z, w)                                                                            \
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
  PDIP
};

enum SOLVERMODE { NORMAL, SLIDING, SPINNING, BILATERAL };

enum COLLISIONSYSTEMTYPE { COLLSYS_PARALLEL, COLLSYS_BULLET_PARALLEL };

enum NARROWPHASETYPE {
  NARROWPHASE_MPR,
  NARROWPHASE_GJK,
  NARROWPHASE_R,
  NARROWPHASE_HYBRID_MPR,
  NARROWPHASE_HYBRID_GJK
};

// This is set so that parts of the code that have been "flattened" can know what
// type of system is used.
enum SYSTEMTYPE { SYSTEM_DVI, SYSTEM_DEM };

enum BILATERALTYPE { BODY_BODY, SHAFT_SHAFT, SHAFT_SHAFT_SHAFT, SHAFT_BODY, SHAFT_SHAFT_BODY, UNKNOWN };

// DEM contact force model
enum CONTACTFORCEMODEL { HOOKE, HERTZ };
enum TANGENTIALDISPLACEMENTMODE { NONE, ONE_STEP, MULTI_STEP };

// Supported Logging Levels
enum LOGGINGLEVEL { LOG_NONE, LOG_INFO, LOG_TRACE, LOG_WARNING, LOG_ERROR };

#endif
