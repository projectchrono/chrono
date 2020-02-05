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

#ifndef CHAPIPOSTPROCESS_H
#define CHAPIPOSTPROCESS_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_POSTPROCESS
// (so that the symbols with 'ChApiPostProcess' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_POSTPROCESS)
#define ChApiPostProcess ChApiEXPORT
#else
#define ChApiPostProcess ChApiIMPORT
#endif

/**
    @defgroup postprocess_module POSTPROCESS module
    @brief Postprocessing tools (for POVray animations, GNUplot, etc.)

    Module provides postprocessing tools that can be used
    to output high quality renderings and other kind of post processed data.

    Currently supported postprocessing targets:
    - POVray
    - GNUplot

    For additional information, see:
    - the [installation guide](@ref module_postprocess_installation)
    - the [tutorials](@ref tutorial_root)
*/

namespace chrono {

/// @addtogroup postprocess_module
/// @{

/// Namespace with classes for the POSTPROCESS module.
namespace postprocess {}

/// @}
}

#endif