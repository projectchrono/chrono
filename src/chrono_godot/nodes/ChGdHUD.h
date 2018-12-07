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
// Authors: Asher Elmquist
// =============================================================================
//
// Class to manage adding, changing, and removing godot scene nodes
//
// =============================================================================
#ifndef CHGDHUD_H
#define CHGDHUD_H

// godot includes
// #include <core/ustring.h>
// #include <scene/gui/button.h>
// #include <scene/gui/control.h>
// #include <scene/gui/label.h>
#include <scene/main/viewport.h>

// chrono includes
#include "chrono/physics/ChSystem.h"

// ChGodotIncludes
#include "chrono_godot/ChApiGodot.h"

namespace chrono {
namespace gd {

class CH_GODOT_API ChGdHUD {
  public:
    ChGdHUD();
    ~ChGdHUD();

    void Initialize(ChSystem* chsystem, Viewport* root);
    void SetDisplay(bool display);
    void Update();

  private:
    // Control* m_hud;
    ChSystem* m_system;

    // Label* time_text;
    std::vector<Object*> m_resourceList;
};

}  // namespace gd
}  // end namespace chrono

#endif
