// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================

#pragma once

#include "chrono/core/ChApiCE.h"
#include "chrono/ChConfig.h"

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
#if defined(CHRONO_HAS_AVX) && defined(CHRONO_SIMD_ENABLED) && defined(USE_COLLISION_DOUBLE)
    #define USE_AVX
    #undef USE_SSE
#elif defined(CHRONO_HAS_SSE) && defined(CHRONO_SIMD_ENABLED) && !defined(USE_COLLISION_DOUBLE)
    #undef USE_AVX
    #define USE_SSE
#else
    #undef USE_AVX
    #undef USE_SSE
#endif
