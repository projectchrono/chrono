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
// Description: definition of a real number which can be defined as a float
// (increased speed on some architectures) or a double (increased precision)
// =============================================================================

#ifndef REAL_H
#define REAL_H

#include "chrono_parallel/ChParallelDefines.h"
#include <float.h>


//Check if SSE was found in CMake
#ifdef CHRONO_PARALLEL_HAS_SSE
//Depending on the SSE variable in CMake include the proper header file for that
//version of sse
#ifdef CHRONO_PARALLEL_SSE_1_0
#include <xmmintrin.h>
#elif defined CHRONO_PARALLEL_SSE_2_0
#include <emmintrin.h>
#elif defined CHRONO_PARALLEL_SSE_3_0
#include <pmmintrin.h>
#elif defined CHRONO_PARALLEL_SSE_4_1
#include <smmintrin.h>
#elif defined CHRONO_PARALLEL_SSE_4_2
#include <nmmintrin.h>
#endif
   #ifndef ENABLE_SSE
      #define ENABLE_SSE
   #endif
#else
#undef ENABLE_SSE
#endif

//If the user specified using doubles in CMake make sure that SSE is disabled
#ifdef CHRONO_PARALLEL_USE_DOUBLE
      #undef ENABLE_SSE
#endif
//If the user specified using doubles, define the real type as double
//Also set some constants. The same is done if floats were specified.
#ifdef CHRONO_PARALLEL_USE_DOUBLE
typedef double real;
#define LARGE_REAL 1e30
#define ZERO_EPSILON DBL_EPSILON
#else
typedef float real;
#define LARGE_REAL 1e18f
#define ZERO_EPSILON FLT_EPSILON
#endif


//Clamps a given value a between user specified minimum and maximum values
static inline real clamp(const real & a,
						 const real & clamp_min,
						 const real & clamp_max) {
	if (a < clamp_min) {
		return clamp_min;
	} else if (a > clamp_max) {
		return clamp_max;
	} else {
		return a;
	}

}
//Performs a linear interpolation between a and b using alpha
static inline real lerp(const real &a, const real &b, real alpha) {
	return (a + alpha * (b - a));

}
//Checks if the value is zero to within a certain epsilon
//in this case ZERO_EPSILON is defined based on what the base type of real is
static inline bool IsZero(const real &a) {
	return fabs(a) < ZERO_EPSILON;
}

//Check if two values are equal using a small delta/epsilon value.
//Essentially a fuzzy comparison operator
static inline bool isEqual(const real &_a, const real &_b) {
	real ab;
	ab = fabs(_a - _b);
	if (fabs(ab) < ZERO_EPSILON)
		return 1;
	real a, b;
	a = fabs(_a);
	b = fabs(_b);
	if (b > a) {
		return ab < ZERO_EPSILON * b;
	} else {
		return ab < ZERO_EPSILON * a;
	}
}

//Returns a -1 if the value is negative
//Returns a +1 if the value is positive
//Otherwise returns zero, this should only happen if the given value is zero
static inline real sign(const real &x) {
	if (x < 0) {
		return -1;
	} else if (x > 0) {
		return 1;
	} else {
		return 0;
	}
}


#endif
