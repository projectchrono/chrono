// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dario Mangoni, Radu Serban
// =============================================================================

#ifndef CHAPI_PARDISOMKL_H
#define CHAPI_PARDISOMKL_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_PARDISOMKL
// (so that the symbols with 'ChApiPardisoMKL' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_PARDISOMKL)
    #define ChApiPardisoMKL ChApiEXPORT
#else
    #define ChApiPardisoMKL ChApiIMPORT
#endif

/**
    @defgroup pardisomkl_module PARDISO-MKL module
    @brief Module for the Intel MKL library Pardiso direct solver

    This module provides an interface to the Pardiso parallel direct solver in the Intel MKL library.

    For additional information, see:
    - the [installation guide](@ref module_mkl_installation)
*/

#endif
