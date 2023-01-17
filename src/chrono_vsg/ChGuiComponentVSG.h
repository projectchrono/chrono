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
// Screen capture code from https://github.com/vsg-dev/vsgExamples.git
//
// =============================================================================
// Radu Serban
// =============================================================================

#ifndef CHGUI_COMPONENT_VSG_H
#define CHGUI_COMPONENT_VSG_H

#include <vsgImGui/RenderImGui.h>

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
