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

#ifndef CHPOVRAYASSET_H
#define CHPOVRAYASSET_H

#include "chrono_postprocess/ChPostProcessBase.h"
#include "chrono/assets/ChAsset.h"

namespace chrono {
namespace postprocess {

/// @addtogroup postprocess_module
/// @{

/// Class for telling to the POV ray exporter that the ChPhysicsItem that
/// contain this asset should be exported to POV scripts.
/// Consider it a very simple 'flagging' system.
/// You can add it to object by hand, otherwise is easier to use ChPovRay::Add(), ex.
/// my_pov_exporter.Add(my_physics_object) shortcut.

class ChPovRayAsset : public ChAsset {
  public:
    ChPovRayAsset() {}

    virtual ~ChPovRayAsset() {}
};

/// @} postprocess_module

}  // end namespace postprocess
}  // end namespace chrono

#endif