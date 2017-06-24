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

#ifndef CHUPDATEFLAGS_H
#define CHUPDATEFLAGS_H

#include "chrono/core/ChBitmaskEnums.h"

namespace chrono {

/// Flags to be passed to objects being updated during time stepping.
/// They are useful for optimizing the computations, and other reasons.
/// In fact it happens that not all update functions require the same 'level of completeness'
/// in recomputing the data in the objects, for example a ChTimestepper might require
/// multiple StateScatter() calls before advancing to the next timestep (for instance in 
/// a RungeKutta integrator it takes 4 intermediate StateScatter() calls) but during 
/// the intermediate calls there is no need of updating, say, all 3D meshes in visual assets.
/// This is the reason for these flags. They are used by ChIntegrable::StateScatter()
/// by ChPhysicsItem::Update(), and in other parts of the code.

enum class eChUpdateFlags : unsigned int 
{
    /// Update all data
    ALL                 = 0x0,

    /// This is called when updating at the beginning of a time step
    NEW_STEP            = 0x1,

    /// Avoid updating visual assets, if any
    NO_ASSETS           = 0x2,

    /// Avoid updating collision shapes, if any
    NO_COLLISION_SHAPES = 0x4
};

CH_ENABLE_BITMASK_OPERATORS(eChUpdateFlags)

}  // end namespace chrono

#endif
