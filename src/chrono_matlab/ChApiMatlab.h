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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHAPIMATLAB_H
#define CHAPIMATLAB_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_MATLAB
// (so that the symbols with 'ChApiPostProcess' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_MATLAB)
    #define ChApiMatlab ChApiEXPORT
#else
    #define ChApiMatlab ChApiIMPORT
#endif

/**
    @defgroup matlab_module MATLAB module
    @brief Interoperation with Matlab(TM)

    Using this module, you can provide interoperation between
    Chrono and the Matlab(TM) software.

    For additional information, see:
    - the [installation guide](@ref module_matlab_installation)
    - the [tutorials](@ref tutorial_root)
*/

#endif
