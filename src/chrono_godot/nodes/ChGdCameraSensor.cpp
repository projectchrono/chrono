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

#include <scene/resources/packed_scene.h>

#include "chrono_godot/nodes/ChGdCameraSensor.h"

#include <iostream>

namespace chrono {
namespace gd {

ChGdCameraSensor::ChGdCameraSensor() {}

void ChGdCameraSensor::_init() {
    std::cout << "_init() for ChGdCameraSensor\n";
}

void ChGdCameraSensor::_ready() {
    std::cout << "_ready() for ChGdCameraSensor\n";
}

void ChGdCameraSensor::_input(InputEvent input) {
    std::cout << "_input() for ChGdCameraSensor\n";
}

void ChGdCameraSensor::_unhandled_input(InputEvent event) {
    std::cout << "_unhandled_input() for ChGdCameraSensor\n";
}
void ChGdCameraSensor::_unhandled_key_input(InputEventKey event) {
    std::cout << "_unhandled_key_input() for ChGdCameraSensor\n";
}
void ChGdCameraSensor::_process(float delta) {
    std::cout << "_process() for ChGdCameraSensor\n";
    // viewport1->get_texture()->get_data()->save_png("/home/asher/code/chrono_godot/chrono-dev/build/bin/viewport1.png");
    // std::cout<<"Depth: "<<this->get_texture()->get_data()->get_pixel(0,0)<<"std::endl";

    Ref<Image> img = this->get_texture()->get_data();
    std::cout << "Image size: " << img->get_size().x << ", " << img->get_size().y << std::endl;
    img->lock();

    std::cout << "Depth ~Middle: " << img->get_pixel(600, 400).r << std::endl;
    std::cout << "Depth ~TopLeft: " << img->get_pixel(5, 5).r << std::endl;
    img->unlock();
    this->get_texture()->get_data()->save_png(
        "/home/asher/code/chrono_godot/chrono-dev/build-release/bin/CameraSensor.png");
}
void ChGdCameraSensor::_notification(int p_notification) {
    if (p_notification == Node::NOTIFICATION_PROCESS)
        _process(0.001);
}
//
//
//
ChGdCamTexture::ChGdCamTexture() {}

void ChGdCamTexture::_init() {
    std::cout << "_init() for ChGdCamTexture\n";
}

void ChGdCamTexture::_ready() {
    std::cout << "_ready() for ChGdCamTexture\n";
}

void ChGdCamTexture::_input(InputEvent input) {
    std::cout << "_input() for ChGdCamTexture\n";
}

void ChGdCamTexture::_unhandled_input(InputEvent event) {
    std::cout << "_unhandled_input() for ChGdCamTexture\n";
}
void ChGdCamTexture::_unhandled_key_input(InputEventKey event) {
    std::cout << "_unhandled_key_input() for ChGdCamTexture\n";
}
void ChGdCamTexture::_process(float delta) {
    std::cout << "_process() for ChGdCamTexture\n";
    // viewport1->get_texture()->get_data()->save_png("/home/asher/code/chrono_godot/chrono-dev/build/bin/viewport1.png");
    // if (first_frame) {
    //     first_frame = false;
    //     return;
    // }
    this->get_texture()->get_data();
    // this->get_texture()->get_data()->save_png(
    //     "/home/asher/code/chrono_godot/chrono-dev/build-release/bin/CameraSensorTexture.png");
}
void ChGdCamTexture::_notification(int p_notification) {
    if (p_notification == Node::NOTIFICATION_PROCESS)
        _process(0.001);
}

}  // namespace gd
}  // namespace chrono
