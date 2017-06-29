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

#ifndef CHAPICOSIMULATION_H
#define CHAPICOSIMULATION_H

#include "chrono/core/ChPlatform.h"

// Chrono::Engine version
//
// This is an integer, as 0xaabbccdd where
// for example version 1.2.0 is 0x00010200

#define CH_VERSION_COSIMULATION_MODULE 0x00000100

// When compiling this library, remember to define CH_API_COMPILE_COSIMULATION
// (so that the symbols with 'ChApiCosimulation' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_COSIMULATION)
#define ChApiCosimulation ChApiEXPORT
#else
#define ChApiCosimulation ChApiIMPORT
#endif

/**
    @defgroup cosimulation_module COSIMULATION module
    @brief Tools for cosimulation. 

    This module allows cosimulation between Chrono::Engine and
    a third party software, using the TCP/IP socket system.

    For additional information, see:
    - the [installation guide](@ref module_cosimulation_installation)
    - the [tutorials](@ref tutorial_root)
*/



namespace chrono {

/// @addtogroup cosimulation_module
/// @{

/// Namespace with classes for the cosimulation module.
namespace cosimul {}

/// @}

}

#endif  // END of header