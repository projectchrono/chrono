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

#ifndef CHAPI_CASCADE_H
#define CHAPI_CASCADE_H

#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_CASCADE
// (so that the symbols with 'ChApiCASCADE' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_CASCADE)
    #define ChApiCASCADE ChApiEXPORT
#else
    #define ChApiCASCADE ChApiIMPORT
#endif

/**
    @defgroup cascade_module CASCADE module
    @brief Tools for interoperation with CAD files.

    This module allows using the OpenCASCADE solid modeling kernel in Chrono::Engine.

    For additional information, see:
    - the [installation guide](@ref module_cascade_installation)
    - the [tutorials](@ref tutorial_root)
*/

namespace chrono {

/// @addtogroup cascade_module
/// @{

/// \brief Namespace with classes for the CASCADE module.
/// The "cascade" namespace contains tools for interoperation with CAD
/// files. The OpenCASCADE open-source library is used to this end:
/// it can load STEP files saved from most 3D CADs.
namespace cascade {}

/// @}

}  // namespace chrono

#endif  // END of header