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

#ifndef CHAPIFEA_H
#define CHAPIFEA_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_FEA
// (so that the symbols with 'ChApiFea' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_FEA)
#define ChApiFea ChApiEXPORT
#else
#define ChApiFea ChApiIMPORT
#endif

/**
    @defgroup fea_module FEA module
    @brief Finite Element Analysis

    This module allows Finite Element Analysis (FEA) in Chrono::Engine.

    For additional information, see:
    - the [manual](manual_fea.html)
    - the [installation guide](@ref module_fea_installation)
    - the [tutorials](@ref tutorial_table_of_content_chrono_fea)

    @{
        @defgroup fea_nodes Nodes
        @defgroup fea_elements Elements
        @defgroup fea_constraints Constraints
        @defgroup fea_math Mathematical support
    @}
*/

namespace chrono {

/// @addtogroup fea_module
/// @{

/// Namespace with classes for the FEA module.
namespace fea {}

/// @}
}

#endif
