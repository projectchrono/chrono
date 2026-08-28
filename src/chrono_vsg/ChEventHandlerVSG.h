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
#include <vsg/ui/WindowEvent.h>

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

    /// The render window acquired keyboard focus.
    virtual void process(vsg::FocusInEvent& ev) {}

    /// The render window lost keyboard focus.
    /// A handler that tracks which keys are currently held down should treat this as a release of all of them:
    /// key events are delivered to whichever window has focus, so any release that happens from now on is never
    /// seen here.
    virtual void process(vsg::FocusOutEvent& ev) {}
};

/// @} vsg_module

}  // namespace vsg3d
}  // namespace chrono

#endif
