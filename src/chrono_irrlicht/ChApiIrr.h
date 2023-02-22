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

#ifndef CHAPIIRRLICHT_H
#define CHAPIIRRLICHT_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_IRRLICHT
// (so that the symbols with 'ChApiIrr' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_IRRLICHT)
#define ChApiIrr ChApiEXPORT
#else
#define ChApiIrr ChApiIMPORT
#endif

/**
    @defgroup irrlicht_module IRRLICHT module
    @brief Runtime visualization with Irrlicht

    This module can be used to provide 3D realtime rendering in Chrono::Engine.

    For additional information, see:
    - the [installation guide](@ref module_irrlicht_installation)
    - the [tutorials](@ref tutorial_root)
*/

namespace chrono {

/// @addtogroup irrlicht_module
/// @{

/// Namespace with classes for the Irrlicht module.
namespace irrlicht {}

namespace irrlicht {
/// Utilities for interfacing Chrono and Irrlicht
namespace tools {}
}

/// @}

}

#endif