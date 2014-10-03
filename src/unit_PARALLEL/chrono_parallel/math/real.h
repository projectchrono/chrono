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
// Description: definition of a floating point number, can defined as a float
// or a double, SSE can be enabed here
// =============================================================================

#ifndef REAL_H
#define REAL_H

#include "chrono_parallel/ChParallelDefines.h"
#include <float.h>



#ifdef CHRONO_PARALLEL_HAS_SSE

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

#ifdef CHRONO_PARALLEL_USE_DOUBLE
      #undef ENABLE_SSE
#endif

#ifdef CHRONO_PARALLEL_USE_DOUBLE
typedef double real;
#define LARGE_REAL 1e30
#define ZERO_EPSILON DBL_EPSILON
#else
typedef float real;
#define LARGE_REAL 1e18f
#define ZERO_EPSILON FLT_EPSILON
#endif



static inline real clamp(const real & a, const real & clamp_min, const real & clamp_max) {
	if (a < clamp_min) {
		return clamp_min;
	} else if (a > clamp_max) {
		return clamp_max;
	} else {
		return a;
	}

}

static inline real lerp(const real &a, const real &b, real alpha) {
	return (a + alpha * (b - a));

}

static inline bool IsZero(const real &a) {
	return fabs(a) < ZERO_EPSILON;
}


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


static inline real sign(const real &x) {
	if (x < 0) {
		return -1;
	} else if (x > 0) {
		return 1;
	} else {
		return 0;
	}
}


template<class T>
real max3(const T &a) {
	return max(a.x, max(a.y, a.z));
}
template<class T>
real min3(const T &a) {
	return min(a.x, min(a.y, a.z));
}

#endif
