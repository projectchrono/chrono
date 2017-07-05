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

#ifndef CHAPIMKL_H
#define CHAPIMKL_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_MKL
// (so that the symbols with 'ChApiMkl' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_MKL)
#define ChApiMkl ChApiEXPORT
#else
#define ChApiMkl ChApiIMPORT
#endif

/**
    @defgroup mkl_module MKL module
    @brief Module for the Intel MKL library direct solver

    Module provides access to the Intel MKL library. This library is
    currently used in Chrono for its parallel direct solver (Pardiso).

    For additional information, see:
    - the [installation guide](@ref module_mkl_installation)
*/

#endif
