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
// Authors: Hammad Mazhar
// =============================================================================
// Header for OpenGL API export
// =============================================================================

#ifndef CHAPIGL_H
#define CHAPIGL_H

#include "chrono/core/ChPlatform.h"

/**
    @defgroup opengl_module OpenGL module
    @brief Runtime visualization with OpenGL

    This module provides support for run-time Chrono visualization using OpenGL.
    Differently from the IRRLICHT module, this system provides a lower-level
    access to the rendering system, when fewer features and faster rendering
    are needed.

    For additional information, see:
    - the [installation guide](@ref module_opengl_installation)
    - the [tutorials](@ref tutorial_root)
*/

// When compiling this library, remember to define CH_API_COMPILE_OPENGL so
// that the symbols with 'CH_OPENGL_API' in front of them will be marked as
// exported. When using this library, CH_API_COMPILE_OPENGL should be left
// undefined so that symbols are imported.

#if defined(CH_API_COMPILE_OPENGL)
#define CH_OPENGL_API ChApiEXPORT
#else
#define CH_OPENGL_API ChApiIMPORT
#endif

#endif
