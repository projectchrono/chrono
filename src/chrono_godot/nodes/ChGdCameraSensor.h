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
#ifndef CHGDCAMERASENSOR_H
#define CHGDCAMERASENSOR_H

#include <scene/3d/sprite_3d.h>
#include <scene/main/viewport.h>
// #include <scene/gui/control.h>
namespace chrono {
namespace gd {

class ChGdCameraSensor : public Viewport {
    GDCLASS(ChGdCameraSensor, Viewport);

  public:
    ChGdCameraSensor();

    // private:
    void _init();
    void _ready();
    void _input(InputEvent input);
    void _unhandled_input(InputEvent event);
    void _unhandled_key_input(InputEventKey event);
    void _process(float delta);
    void _notification(int p_notification);
};

class ChGdCamTexture : public Sprite3D {
    GDCLASS(ChGdCamTexture, Sprite3D);

  public:
    ChGdCamTexture();

    // private:
    void _init();
    void _ready();
    void _input(InputEvent input);
    void _unhandled_input(InputEvent event);
    void _unhandled_key_input(InputEventKey event);
    void _process(float delta);
    void _notification(int p_notification);

    bool first_frame = true;
};

}  // namespace gd
}  // namespace chrono
#endif
