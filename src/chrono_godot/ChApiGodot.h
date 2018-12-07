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

#ifndef CHAPIGODOT_H
#define CHAPIGODOT_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_GODOT
// (so that the symbols with 'ChApiGodot' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_GODOT)
#define CH_GODOT_API ChApiEXPORT
#else
#define CH_GODOT_API ChApiIMPORT
#endif

/**
    @defgroup godot_module GODOT module
    @brief Runtime visualization with Godot

    This module can be used to provide 3D realtime rendering
    in Chrono::Engine.

    For additional information, see:
    - the [installation guide](@ref module_godot_installation)
    - the [tutorials](@ref tutorial_root)
*/

namespace chrono {

/// @addtogroup godot_module
/// @{

/// Namespace with classes for the Godot module.
namespace gd {}

/// @}
}  // namespace chrono

#endif
