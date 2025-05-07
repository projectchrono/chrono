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

#ifndef CHAPIPERIDYNAMICS_H
#define CHAPIPERIDYNAMICS_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_PERIDYDAMICS
// (so that the symbols with 'ChApiPeridynamics' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_PERIDYNAMICS)
#define ChApiPeridynamics ChApiEXPORT
#else
#define ChApiPeridynamics ChApiIMPORT
#endif

/**
    @defgroup peridynamics_module PERIDYNAMICS module
    @brief This provides classes for modeling meshless materials.

    Module provides classes and tools for simulating materials with 
    large deformations and fractures, using the peridynamics framework.

    For additional information, see:
    - the [installation guide](@ref module_peridynamics_installation)
    - the [tutorials](@ref tutorial_root)
*/

namespace chrono {

/// @addtogroup peridynamics_module
/// @{

/// Namespace with classes for the PERIDYNAMICS module.
namespace peridynamics {}

/// @}
}

#endif