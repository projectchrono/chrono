// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// Screen capture code from https://github.com/vsg-dev/vsgExamples.git
//
// =============================================================================
//
// Implementation of the base Chrono::VSG event handlers.
//
// =============================================================================
// Rainer Gericke, Radu Serban
// =============================================================================

#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_vsg/impl/BaseEventHandlers.h"

namespace chrono {
namespace vsg3d {

ChBaseEventHandlerVSG::ChBaseEventHandlerVSG(ChVisualSystemVSG* app) : m_app(app) {}

void ChBaseEventHandlerVSG::process(vsg::KeyPressEvent& keyPress) {
    if (keyPress.keyBase == 'm' || keyPress.keyModified == 'm') {
        m_app->ToggleGuiVisibility();
    }
    if (keyPress.keyBase == 'n' || keyPress.keyModified == 'n') {
        m_app->GetGuiComponent(m_app->m_camera_gui)->ToggleVisibility();
    }
    if (keyPress.keyBase == vsg::KEY_Escape || keyPress.keyModified == 65307) {
        m_app->Quit();
    }
}

}  // namespace vsg3d
}  // namespace chrono
