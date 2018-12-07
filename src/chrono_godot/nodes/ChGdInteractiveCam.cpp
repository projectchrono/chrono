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
//
//
// =============================================================================

#include "chrono_godot/nodes/ChGdInteractiveCam.h"

#include <iostream>

#include <core/os/input.h>
#include <core/os/input_event.h>
#include <scene/resources/packed_scene.h>

namespace chrono {
namespace gd {

ChGdInteractiveCam::ChGdInteractiveCam() {}

void ChGdInteractiveCam::_init() {
    std::cout << "INTERACTIVE CAMERA INIT CALLED!\n";
}

void ChGdInteractiveCam::_ready() {
    std::cout << "INTERACTIVE CAMERA IS READY!\n";
}

void ChGdInteractiveCam::_input(InputEvent input) {
    std::cout << "\nEVENT RECEIVED\n\n";
}

void ChGdInteractiveCam::_unhandled_input(InputEvent event) {
    std::cout << "\nEVENT RECEIVED: InputEvent\n\n";
}
void ChGdInteractiveCam::_unhandled_key_input(InputEventKey event) {
    std::cout << "\nEVENT RECEIVED: InputEventKey\n\n";
}
void ChGdInteractiveCam::_process(float delta) {
    // TODO: turn the proof of concept mouse controls into working control
    if (Input::get_singleton()->is_mouse_button_pressed(BUTTON_MIDDLE)) {
        std::cout << "Rotating camera\n";
        this->rotate_y(0.00001 * Input::get_singleton()->get_last_mouse_speed().x);
        this->rotate_x(0.00001 * Input::get_singleton()->get_last_mouse_speed().y);
    }

    if (Input::get_singleton()->is_mouse_button_pressed(BUTTON_RIGHT)) {
        std::cout << "zooming camera out\n";
        this->translate_object_local({0, 0, 1});
    }

    if (Input::get_singleton()->is_mouse_button_pressed(BUTTON_LEFT)) {
        std::cout << "zooming camera in\n";
        this->translate_object_local({0, 0, -1});
    }

    // std::cout << "\nPROCESS WITH DELTA: " << delta << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
}
void ChGdInteractiveCam::_notification(int p_notification) {
    if (p_notification == Node::NOTIFICATION_PROCESS)
        _process(0.001);
}

}  // namespace gd
}  // namespace chrono
