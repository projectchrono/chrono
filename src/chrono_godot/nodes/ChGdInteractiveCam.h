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
#ifndef CHGDINTERACTIVECAM_H
#define CHGDINTERACTIVECAM_H

#include <scene/3d/camera.h>
// #include <scene/gui/control.h>

#include "chrono_godot/ChApiGodot.h"

namespace chrono {
namespace gd {

class CH_GODOT_API ChGdInteractiveCam : public Camera {
    GDCLASS(ChGdInteractiveCam, Camera);

  public:
    ChGdInteractiveCam();

    // private:
    void _init();
    void _ready();
    void _input(InputEvent input);
    void _unhandled_input(InputEvent event);
    void _unhandled_key_input(InputEventKey event);
    void _process(float delta);
    void _notification(int p_notification);
};
}  // namespace gd
}  // namespace chrono
#endif
