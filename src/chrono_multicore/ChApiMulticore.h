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

#pragma once

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_MULTICORE
// (so that the symbols with 'CH_MULTICORE_API' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_MULTICORE)
    #define CH_MULTICORE_API ChApiEXPORT
#else
    #define CH_MULTICORE_API ChApiIMPORT
#endif

// Macros for specifying type alignment
#if (defined __GNUC__) || (defined __INTEL_COMPILER)
    #define CHRONO_ALIGN_16 __attribute__((aligned(16)))
#elif defined _MSC_VER
    #define CHRONO_ALIGN_16 __declspec(align(16))
#else
    #define CHRONO_ALIGN_16
#endif

#if defined _MSC_VER
    #define fmax fmax
    #define fmin fmin
#endif

#if defined(_WIN32) || defined(_WIN64)
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #define ELPP_WINSOCK2
#endif

/**
    @defgroup multicore_module MULTICORE module
    @brief Module for multicore parallel simulation

    This module implements multicore parallel computing algorithms that can be
    used as a faster alternative to the default simulation algorithms in Chrono.
    This is achieved using OpenMP, CUDA, Thrust, etc.

    For additional information, see:
    - the [installation guide](@ref module_multicore_installation)
    - the [tutorials](@ref tutorial_table_of_content_chrono_multicore)

    @{
        @defgroup multicore_physics Physics objects
        @defgroup multicore_constraint Unilateral constraints
        @defgroup multicore_collision Collision objects
        @defgroup multicore_solver Solvers
        @defgroup multicore_math Math utilities
    @}
*/
