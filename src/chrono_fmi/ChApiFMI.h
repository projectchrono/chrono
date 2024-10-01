// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_API_FMI_H
#define CH_API_FMI_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, define CH_API_COMPILE_FMI, so that symbols decorated with
// 'ChApiFMI' are marked as exported. If the macro is not defined, symbols will be imported.

#if defined(CH_API_COMPILE_FMI)
    #define ChApiFMI ChApiEXPORT
#else
    #define ChApiFMI ChApiIMPORT
#endif

/**
    @defgroup chrono_fmi FMI module
    @brief FMU export and import functionality

    This module provides Chrono support for the Functional Mock-Up Interface (FMI) standard.
    It includes utilities for generating (exporting) Chrono co-simulation Functional Mock-up Units (FMU) and
    for importing FMUs that encapsulate Chrono models and simulations or external FMUs and using them
    in co-simulation with other FMUs or Chrono models.

    For additional information, see:
    - the [installation guide](@ref module_fmi_installation)
    - the [tutorials](@ref tutorial_table_of_content_chrono_fmi)

    @{
      @defgroup chrono_fmi2 FMI 2.0 support
      @defgroup chrono_fmi3 FMI 3.0 support

      @defgroup fmu_tools
      @brief Generic utility library for FMU export and import
      
      For additional information see the fmu_tools [repository](https://github.com/DigitalDynamicsLab/fmu_tools/blob/main/README.md)
      @{
        @defgroup fmu_tools_fmi2 FMI 2.0 utilities
        @defgroup fmu_tools_fmi3 FMI 3.0 utilities
      @}
    @}
*/

namespace chrono {

/// @addtogroup chrono_fmi
/// @{

/// Namespace with classes for the FMI module (FMI 2.0 support).
namespace fmi2 {}

/// Namespace with classes for the FMI module (FMI 3.0 support).
namespace fmi3 {}

/// @}
}  // namespace chrono

#endif
