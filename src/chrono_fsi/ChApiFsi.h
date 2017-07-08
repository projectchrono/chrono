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
// Author: Arman Pazouki
// =============================================================================
//
// Macro defines for exporting DLL
// =============================================================================

#ifndef CHAPIFSI_H_
#define CHAPIFSI_H_

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_FSI
// (so that the symbols with 'CH_FSI_API' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_FSI)
#define CH_FSI_API ChApiEXPORT
#else
#define CH_FSI_API ChApiIMPORT
#endif

/**
    @defgroup fsi FSI module
    @brief Fluid-Solid Interation modeling and simulation

    This module provides support for modeling multi-phase systems for
    fluid-soild interaction problems.

    For additional information, see:
    - the [installation guide](@ref module_fsi_installation)
    - the [tutorials](@ref tutorial_table_of_content_chrono_fsi)

    @{
        @defgroup fsi_physics Physics objects
        @defgroup fsi_collision Collision objects
        @defgroup fsi_solver Solvers
        @defgroup fsi_math Math utilities
    @}
*/

namespace chrono {

/// @addtogroup fsi
/// @{

/// Namespace with classes for the FSI module.
namespace fsi {}

/// @}
}

#endif
