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
// Authors: Conlain Kelly, Nic Olsen, Dan Negrut
// =============================================================================

#pragma once

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_GRANULAR
// (so that the symbols with 'CH_GRANULAR_API' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_GRANULAR)
#define CH_GRANULAR_API ChApiEXPORT
#else
#define CH_GRANULAR_API ChApiIMPORT
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

#if defined(WIN32) || defined(WIN64)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#define ELPP_WINSOCK2
#endif

/**
    @defgroup granular Granular module
    @brief GPU solver for granular dynamics

    This module provides support for modeling systems of granular material on the GPU through CUDA.
    Currently, systems of monodisperse spheres can interact via full-history frictional contact with both
    analytical boundary conditions and triangle meshes.

    For additional information, see:
    -- the [installation guide](@ref module_granular_installation)
    -- the [tutorials](@ref tutorial_root)


    @{
        @defgroup granular_physics Granular Physics objects
        @defgroup granular_solver Granular solver infrastructure
    @}
*/

namespace chrono {

/// @addtogroup granular
/// @{

/// Namespace with classes for the Granular module.
namespace granular {}

/// @}
}  // namespace chrono
