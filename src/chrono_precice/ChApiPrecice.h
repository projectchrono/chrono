// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_API_PRECICE_H
#define CH_API_PRECICE_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_PRECICE so that symbols tagged with
// 'ChApiPrecice' will be marked as exported. Otherwise, just do not define it if you link the library
// to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_PRECICE)
    #define ChApiPrecice ChApiEXPORT
#else
    #define ChApiPrecice ChApiIMPORT
#endif

/**
    @defgroup precice_module preCICE adapters module
    @brief preCICE adapters for various Chrono modules.

    For additional information, see:
    - the [user manual](@ref manual_precice)
    - the [installation guide](@ref module_precice_installation)
    - the [tutorials](@ref tutorial_root)
*/

namespace chrono {

/// @addtogroup precice_module
/// @{

/// Namespace with classes for the Chrono::PRECICE module.
namespace ch_precice {}

/// @}

}  // namespace chrono

#endif
