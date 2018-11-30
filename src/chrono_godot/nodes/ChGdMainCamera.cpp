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
#include <scene/3d/camera.h>

#include "chrono_godot/nodes/ChGdMainCamera.h"

namespace chrono {
namespace gd {

ChGdMainCamera::ChGdMainCamera(Viewport* viewport,
                               ChVector<> location,
                               ChVector<> target_location,
                               ChVector<> up,
                               float FOV) {
    Camera* camera = memnew(Camera);
    camera->set_name("MainCamera");
    camera->look_at_from_position(GDVector(location), GDVector(target_location), GDVector(up));
    camera->set_fov(FOV);

    viewport->add_child(camera);
}
ChGdMainCamera::~ChGdMainCamera() {
    // memdelete(m_camera);
}

}  // namespace gd
}  // namespace chrono
