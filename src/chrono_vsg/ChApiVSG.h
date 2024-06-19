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
// Authors: Hammad Mazhar
// =============================================================================
// Header for VSG API export
// =============================================================================

#ifndef CHAPIVSG_H
#define CHAPIVSG_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_VSG so
// that the symbols with 'CH_VSG_API' in front of them will be marked as
// exported. When using this library, CH_API_COMPILE_VSG should be left
// undefined so that symbols are imported.

#if defined(CH_API_COMPILE_VSG)
    #define CH_VSG_API ChApiEXPORT
#else
    #define CH_VSG_API ChApiIMPORT
#endif

/**
    @defgroup vsg_module VSG module
    @brief Runtime visualization with VSG

    This module can be used to provide 3D realtime rendering in Chrono::Engine.

    For additional information, see:
    - the [installation guide](@ref module_vsg_installation)
    - the [tutorials](@ref tutorial_root)
*/

namespace chrono {

/// @addtogroup vsg_module
/// @{

/// Namespace with classes for the VSG module.
namespace vsg3d {}

}  // namespace chrono

#endif
