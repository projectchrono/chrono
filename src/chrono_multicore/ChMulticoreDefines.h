// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Description: lots of useful definitions for thrust, includes and enums
//
// =============================================================================

#pragma once

#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include <iostream>

// Include the ChConfigMulticore header *before* any Thrust headers!
#include "chrono_multicore/ChApiMulticore.h"
#include "chrono_multicore/ChConfigMulticore.h"

#include <thrust/reduce.h>
#include <thrust/gather.h>
#include <thrust/scan.h>
#include <thrust/fill.h>
#include <thrust/copy.h>
#include <thrust/iterator/counting_iterator.h>

#if defined(CHRONO_OPENMP_ENABLED)
#include <thrust/system/omp/execution_policy.h>
#elif defined(CHRONO_TBB_ENABLED)
#include <thrust/system/tbb/execution_policy.h>
#endif

#ifndef _MSC_VER
#include <cfenv>
#endif

typedef int shape_type;

#ifdef __CDT_PARSER__
#define BLAZE_SERIAL_SECTION
#define CH_MULTICORE_API
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
#include "chrono_thirdparty/easylogging/easylogging.h"
#define LOGGINGENABLED

#endif

#if defined(CHRONO_OPENMP_ENABLED)
#define THRUST_PAR thrust::omp::par,
#elif defined(CHRONO_TBB_ENABLED)
#define THRUST_PAR thrust::tbb::par,
#else
#define THRUST_PAR
#endif

/// @addtogroup multicore_module
/// @{

#define Thrust_Inclusive_Scan_Sum(x, y)                               \
    thrust::inclusive_scan(THRUST_PAR x.begin(), x.end(), x.begin()); \
    y = x.back();
#define Thrust_Sort_By_Key(x, y) thrust::sort_by_key(THRUST_PAR x.begin(), x.end(), y.begin())

#define Run_Length_Encode(y, z, w)                                                                                  \
    (thrust::reduce_by_key(THRUST_PAR y.begin(), y.end(), thrust::constant_iterator<uint>(1), z.begin(), w.begin()) \
         .second) -                                                                                                 \
        w.begin()

#define Thrust_Inclusive_Scan(x) thrust::inclusive_scan(THRUST_PAR x.begin(), x.end(), x.begin())
#define Thrust_Exclusive_Scan(x) thrust::exclusive_scan(THRUST_PAR x.begin(), x.end(), x.begin())
#define Thrust_Fill(x, y) thrust::fill(x.begin(), x.end(), y)
#define Thrust_Sort(x) thrust::sort(THRUST_PAR x.begin(), x.end())
#define Thrust_Count(x, y) thrust::count(THRUST_PAR x.begin(), x.end(), y)
#define Thrust_Sequence(x) thrust::sequence(x.begin(), x.end())
#define Thrust_Equal(x, y) thrust::equal(THRUST_PAR x.begin(), x.end(), y.begin())
#define Thrust_Max(x) x[thrust::max_element(THRUST_PAR x.begin(), x.end()) - x.begin()]
#define Thrust_Min(x) x[thrust::min_element(THRUST_PAR x.begin(), x.end()) - x.begin()]
#define Thrust_Total(x) thrust::reduce(THRUST_PAR x.begin(), x.end())
#define Thrust_Unique(x) thrust::unique(THRUST_PAR x.begin(), x.end()) - x.begin();
#define DBG(x) printf(x);

/// @} multicore_module

namespace chrono {

/// @addtogroup multicore_module
/// @{

/// Iterative solver type.
enum class SolverType {
    STEEPEST_DESCENT,            ///< steepest descent
    GRADIENT_DESCENT,            ///< gradient descent
    CONJUGATE_GRADIENT,          ///< conjugate gradient
    CONJUGATE_GRADIENT_SQUARED,  ///< conjugate gradient squared
    BICONJUGATE_GRADIENT,        ///< BiCG
    BICONJUGATE_GRADIENT_STAB,   ///< BiCGStab
    MINIMUM_RESIDUAL,            ///< MINRES (minimum residual)
    QUASI_MINIMUM_RESIDUAL,      ///< Quasi MINRES
    APGD,                        ///< Accelerated Projected Gradient Descent
    APGDREF,                     ///< reference implementation for APGD
    JACOBI,                      ///< Jacobi
    GAUSS_SEIDEL,                ///< Gauss-Seidel
    PDIP,                        ///< Primal-Dual Interior Point
    BB,                          ///< Barzilai-Borwein
    SPGQP                        ///< Spectral Projected Gradient (QP projection)
};

/// Enumeration for solver mode.
enum class SolverMode {
    NORMAL,    ///< solve only normal contact impulses
    SLIDING,   ///< solve for contact and sliding friction impulses
    SPINNING,  ///< solve for rolling resistance impulses
    BILATERAL  ///< solve for bilateral Lagrange multipliers
};

/// Enumeration for the collision system type
enum class CollisionSystemType {
    COLLSYS_MULTICORE,        ///< default collision system
    COLLSYS_BULLET_MULTICORE  ///< Bullet-based collision system
};

/// Enumeration of narrow-phase collision methods.
enum class NarrowPhaseType {
    NARROWPHASE_MPR,        ///< Minkovski Portal Refinement
    NARROWPHASE_R,          ///< analytical collision detection
    NARROWPHASE_HYBRID_MPR  ///< analytical method with fallback on MPR
};

/// Enumeration for system type.
/// Used so that parts of the code that have been "flattened" can know what type of system is used.
enum class SystemType {
    SYSTEM_NSC,  ///< system using non-smooth (complementarity) contact
    SYSTEM_SMC   ///< system using smooth (penalty) contact
};

/// Enumeration for bilateral constraint types.
enum BilateralType {
    BODY_BODY,          ///< constraints between two rigid bodies
    SHAFT_SHAFT,        ///< constraints between two 1-D shaft elements
    SHAFT_SHAFT_SHAFT,  ///< constraints involving 3 1-D shaft elements
    SHAFT_BODY,         ///< constraints between a shaft and a rigid body
    SHAFT_SHAFT_BODY,   ///< constraints involving two shafts and one rigid body
    UNKNOWN             ///< unknow constraint type
};

/// Supported Logging Levels.
enum class LoggingLevel {
    LOG_NONE,     ///< no logging
    LOG_INFO,     ///< info
    LOG_TRACE,    ///< tracing
    LOG_WARNING,  ///< warnings
    LOG_ERROR     ///< errors
};

/// Explicit conversion of scoped enumeration to int (e.g. for streaming).
template <typename Enumeration>
auto as_integer(Enumeration const value) -> typename std::underlying_type<Enumeration>::type {
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

/// Utility to expand an input sequence by
/// replicating each element a variable number of times. For example,
///
///   expand([2,2,2],[A,B,C]) -> [A,A,B,B,C,C]
///   expand([3,0,1],[A,B,C]) -> [A,A,A,C]
///   expand([1,3,2],[A,B,C]) -> [A,B,B,B,C,C]
///
/// The element counts are assumed to be non-negative integers
/// (from Thrust's example expand.cu)
template <typename InputIterator1, typename InputIterator2, typename OutputIterator>
OutputIterator Thrust_Expand(InputIterator1 first1, InputIterator1 last1, InputIterator2 first2, OutputIterator output) {
    typedef typename thrust::iterator_difference<InputIterator1>::type difference_type;

    difference_type input_size = thrust::distance(first1, last1);
    difference_type output_size = thrust::reduce(THRUST_PAR first1, last1);

    // scan the counts to obtain output offsets for each input element
    custom_vector<difference_type> output_offsets(input_size, 0);
    thrust::exclusive_scan(THRUST_PAR first1, last1, output_offsets.begin());

    // scatter the nonzero counts into their corresponding output positions
    custom_vector<difference_type> output_indices(output_size, 0);
    thrust::scatter_if(THRUST_PAR thrust::counting_iterator<difference_type>(0),
                       thrust::counting_iterator<difference_type>(input_size), output_offsets.begin(), first1,
                       output_indices.begin());

    // compute max-scan over the output indices, filling in the holes
    thrust::inclusive_scan(THRUST_PAR output_indices.begin(), output_indices.end(), output_indices.begin(),
                           thrust::maximum<difference_type>());

    // gather input values according to index array (output = first2[output_indices])
    OutputIterator output_end = output;
    thrust::advance(output_end, output_size);
    thrust::gather(THRUST_PAR output_indices.begin(), output_indices.end(), first2, output);

    // return output + output_size
    thrust::advance(output, output_size);
    return output;
}


/// @} multicore_module

}  // end namespace chrono

#define max_neighbors 64
#define max_rigid_neighbors 32
