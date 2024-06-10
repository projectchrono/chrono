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

#ifndef CHAPIMULTIDOMAIN_H
#define CHAPIMULTIDOMAIN_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, CH_API_COMPILE_MULTIDOMAIN will be defined by cmake
// (so that the symbols with 'ChApiMultiDomain' in front of them will be
// marked as exported). This won't be defined when you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_MULTIDOMAIN)
    #define ChApiMultiDomain ChApiEXPORT
#else
    #define ChApiMultiDomain ChApiIMPORT
#endif

/**
    @defgroup multidomain_module MULTIDOMAIN module
    @brief Multi-domain functionality for domain decomposition.

    Module provides functionality so split a large Chrono model 
	into separate models on multiple domains, that will communicate
	to match the solution at the interface shared nodes/bodies.
	
	This can be useful for computing large problems on cluster of
	computing nodes, as on supercomputers, with MPI for inter-domain
	communication. 

    For additional information, see:
    - the [installation guide](@ref module_multidomain_installation)
    - the [tutorials](@ref tutorial_root)
*/

namespace chrono {

/// @addtogroup multidomain_module
/// @{

/// Namespace with classes for the MULTIDOMAIN module.
namespace multidomain {}

/// @}
}  // namespace chrono

#endif