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

    This module provides support for modeling multi-phase systems for fluid-solid interaction problems.
    Currently, this module consists of the following libraries:
    - Chrono_fsi implements a generic interface between a Chrono mutibody system and an arbitraru FSI-capable fluid solver.
    - Chrono_fsisph implements a concrete, SPH-based FSI fluid solver.
    and granular-solid interaction problems.
    - Chrono_fsisph_vsg implements a customized Chrono::VSG run-time visualization system for SPH FSI problems.
    - Chrono_fsitdpf implements a concrete fluid solver, based on the time-dependent potential flow solver in HydroChrono.

    For additional information, see:
    - the [Installation guide](@ref module_fsi_installation)
    - the [Tutorials](@ref tutorial_table_of_content_chrono_fsi)

    @{
        @defgroup fsi_base Generic FSI interface
        @brief Base classes for a generic FSI Chrono interface
        @defgroup fsisph SPH-based FSI module
        @brief SPH-based fluid solver and FSI interface
        @{
          @defgroup fsisph_physics Physics objects
          @defgroup fsisph_math Math support
          @defgroup fsisph_utils Utilities
          @defgroup fsisph_visualization Visualization
        @}
        @defgroup fsitdpf TDPF-based FSI module
    @}
*/

namespace chrono {

/// @addtogroup fsi
/// @{

/// Namespace with classes for the FSI module.
namespace fsi {

/// Namespace with internal classes for the SPH-based FSI submodule.
namespace sph {}

/// Namespace with internal classes for the TDPF-based FSI submodule.
namespace tdpf {}

}  // namespace fsi

/// @}
}  // namespace chrono

#endif
