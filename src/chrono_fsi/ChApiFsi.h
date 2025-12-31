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
    The Chrono::FSI module provides the following libraries:
    - Chrono_fsi:        implements a generic interface between a Chrono mutibody system and an arbitrary FSI-capable fluid solver.
    - Chrono_fsisph:     concrete, SPH-based FSI solver for incompressible Navier-Stokes and continuous granular dyynamics.
    - Chrono_fsitdpf:    concrete fluid solver, based on the time-dependent potential flow solver in HydroChrono.
    - Chrono_fsisph_vsg: customized VSG-based run-time visualization system for SPH FSI problems.

    For additional information, see:
    - the [Installation guide](@ref module_fsi_installation)
    - the [Tutorials](@ref tutorial_table_of_content_chrono_fsi)

    @{
        @defgroup fsi_base Generic FSI interface
        @brief Base classes for a generic FSI Chrono interface
        @defgroup fsisph SPH-based FSI module
        @brief SPH-based fluid solver and FSI interface
        This sub-module provides a Smoothed Particle Hydrodynamics fluid solver with support for:
        - incompressible Navier-Stokes, and
        - continuous representation model for terramechanics.
        @{
          @defgroup fsisph_physics Physics objects
          @defgroup fsisph_math Math support
          @defgroup fsisph_utils Utilities
          @defgroup fsisph_visualization Visualization
        @}
        @defgroup fsitdpf TDPF-based FSI module
        @brief TDPF-based fluid solver and FSI interface
        This sub-module provides a Time-Dependent Potential Flow fluid solver for FSI problems.
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
