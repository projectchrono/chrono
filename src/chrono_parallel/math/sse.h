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
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: definition of a real number which can be defined as a float
// (increased speed on some architectures) or a double (increased precision)
// =============================================================================

#pragma once

#include "chrono_parallel/ChApiParallel.h"
#include "chrono_parallel/ChConfigParallel.h"
#include "chrono_parallel/ChCudaDefines.h"

#include <blaze/system/Version.h>

#ifndef __CUDACC__

// Include appropriate SSE header, depending on supported level
#ifdef CHRONO_HAS_SSE
#ifdef CHRONO_SSE_1_0
#include <xmmintrin.h>
#elif defined CHRONO_SSE_2_0
#include <emmintrin.h>
#elif defined CHRONO_SSE_3_0
#include <pmmintrin.h>
#elif defined CHRONO_SSE_4_1
#include <smmintrin.h>
#elif defined CHRONO_SSE_4_2
#include <nmmintrin.h>
#endif
#endif

// Include AVX header
#ifdef CHRONO_HAS_AVX
#include <immintrin.h>
#endif

// Decide whether to use AVX, SSE, or neither
#if defined(CHRONO_HAS_AVX) && defined(CHRONO_PARALLEL_USE_SIMD) && defined(CHRONO_PARALLEL_USE_DOUBLE)
#define USE_AVX
#undef USE_SSE
#elif defined(CHRONO_HAS_SSE) && defined(CHRONO_PARALLEL_USE_SIMD) && !defined(CHRONO_PARALLEL_USE_DOUBLE)
#undef USE_AVX
#define USE_SSE
#else
#undef USE_AVX
#undef USE_SSE
#endif

// Address an issue with Blaze macros in Vectorization.h pre-3.2
#ifdef _MSC_VER
#if (BLAZE_MAJOR_VERSION < 3) || (BLAZE_MAJOR_VERSION == 3 && BLAZE_MINOR_VERSION < 2)
#undef __AVX__
#undef __AVX2__
#endif
#endif

#endif
