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
// Author: Arman Pazouki, Wei Hu
// =============================================================================
//
// Macro defines for exporting DLL
// =============================================================================

#ifndef CH_API_FSI_H
#define CH_API_FSI_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

#if defined(CH_API_COMPILE_FSI)
    #define CH_FSI_API ChApiEXPORT
#else
    #define CH_FSI_API ChApiIMPORT
#endif

/**
    @defgroup fsi FSI module
    @brief Fluid-Solid Interation modeling and simulation

    This module provides support for modeling multi-phase systems for
    fluid-soild interaction problems and granular-solid interaction problems.
    When compiling this library, remember to define CH_API_COMPILE_FSI
    (so that the symbols with 'CH_FSI_API' in front of them will be
    marked as exported). Otherwise, just do not define it if you
    link the library to your code, and the symbols will be imported.

    For additional information, see:
    - the [Installation guide](@ref module_fsi_installation)
    - the [Tutorials](@ref tutorial_table_of_content_chrono_fsi)

    @{
        @defgroup fsi_physics Physics objects
        @brief Physics objects for the Chrono::FSI module. Including the
        fluid dynamics system, force system, interface with Chrono core
        module, simulation parameters, and data structures.

        @defgroup fsi_collision Collision objects
        @brief Collision objects handles the neighbor particle searching
        in Chrono::FSI module.

        @defgroup fsi_solver Linear solvers
        @brief Classes for solving a linear linear system via iterative methods.
        Only used when I2SPH is set to solve the fluid dynamics.

        @defgroup fsi_utils Modeling utilities
        @brief Utilities for creating BCE particles, setting parameters via a JSON file,
        and output data into files with specified format

        @defgroup fsi_math Math utilities
        @brief Math utilities for the Chrono::FSI module.

        @defgroup fsi_visualization Run-time visualization
        @brief Classes for run-time visualization of SPH-based FSI problems (VSG-based)
    @}
*/

namespace chrono {

/// @addtogroup fsi
/// @{

/// Namespace with classes for the FSI module.
namespace fsi {

/// Namespace with internal classes for the SPH-based FSI submodule.
namespace sph {}

}  // namespace fsi

/// @}
}  // namespace chrono

#endif
