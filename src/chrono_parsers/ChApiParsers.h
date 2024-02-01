// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_API_PARSERS_H
#define CH_API_PARSERS_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_PARSERS so that symbols tagged with
// 'ChApiParsers' will be marked as exported. Otherwise, just do not define it if you link the library
// to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_PARSERS)
#define ChApiParsers ChApiEXPORT
#else
#define ChApiParsers ChApiIMPORT
#endif

/**
    @defgroup parsers_module Parsers module
    @brief Parsers for various input data formats

    For additional information, see:
    - the [user manual](@ref manual_parsers)
    - the [installation guide](@ref module_parsers_installation)
    - the [tutorials](@ref tutorial_root)
*/

namespace chrono {

/// @addtogroup parsers_module
/// @{

/// Namespace with classes for the Parsers module.
namespace parsers {}

/// @}

}

#endif
