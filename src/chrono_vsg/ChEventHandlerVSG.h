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

#ifndef CH_EVENT_HANDLER_VSG_H
#define CH_EVENT_HANDLER_VSG_H

#include <vsg/ui/KeyEvent.h>

#include "chrono_vsg/ChApiVSG.h"

namespace chrono {
namespace vsg3d {

/// @addtogroup vsg_module
/// @{

/// Base class for a user-defined event handler for the VSG run-time visualization system.
class CH_VSG_API ChEventHandlerVSG {
  public:
    ChEventHandlerVSG() {}
    virtual ~ChEventHandlerVSG() {}

    virtual void process(vsg::KeyPressEvent& ev) {}
    virtual void process(vsg::KeyReleaseEvent& ev) {}
    virtual void process(vsg::ButtonPressEvent& ev) {}
    virtual void process(vsg::ButtonReleaseEvent& ev) {}
    virtual void process(vsg::MoveEvent& ev) {}
    virtual void process(vsg::TouchEvent& ev) {}
};

/// @} vsg_module

}  // namespace vsg3d
}  // namespace chrono

#endif
