// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Radu Serban
// =============================================================================

#ifndef CH_GUI_COMPONENT_VSG_H
#define CH_GUI_COMPONENT_VSG_H

#include "chrono_vsg/ChApiVSG.h"

namespace chrono {
namespace vsg3d {

/// @addtogroup vsg_module
/// @{

/// Base class for a GUI component for the VSG run-time visualization system.
class CH_VSG_API ChGuiComponentVSG {
  public:
    ChGuiComponentVSG() {}
    virtual ~ChGuiComponentVSG() {}

    virtual void render() = 0;
};

/// @} vsg_module

}  // namespace vsg3d
}  // namespace chrono

#endif