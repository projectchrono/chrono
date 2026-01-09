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
// =============================================================================
//
// Implementation of the base Chrono::VSG event handlers.
//
// =============================================================================
// Radu Serban
// =============================================================================

#pragma once

#include "chrono_vsg/ChEventHandlerVSG.h"

namespace chrono {
namespace vsg3d {

class ChVisualSystemVSG;

class ChBaseEventHandlerVSG : public ChEventHandlerVSG {
  public:
    ChBaseEventHandlerVSG(ChVisualSystemVSG* app);
    virtual void process(vsg::KeyPressEvent& keyPress) override;
    ChVisualSystemVSG* m_app;
};

}  // namespace vsg3d
}  // namespace chrono
