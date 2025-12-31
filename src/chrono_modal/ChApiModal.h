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

#ifndef CHAPIMODAL_H
#define CHAPIMODAL_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_MODAL
// (so that the symbols with 'ChApiPostProcess' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_MODAL)
#define ChApiModal ChApiEXPORT
#else
#define ChApiModal ChApiIMPORT
#endif

/**
    @defgroup modal MODAL module
    @brief Modal analysis and substructuring

    Using this module, you can perform modal analysis directly in Chrono.
    This can be useful for computing natural frequencies, for computing stability (complex eigenvalue analysis, with
    damping matrices), or for substructuring (where subassemblies are replaced by modal bodies).

    For additional information, see:
    - the [installation guide](@ref module_modal_installation)
    - the [tutorials](@ref tutorial_root)

    @{
        @defgroup modal_vis Run-time visualization
    @}
*/

namespace chrono {

/// @addtogroup modal
/// @{

/// Namespace with classes for the modal module.
namespace modal {}

/// @}

}

#endif
