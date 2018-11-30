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
#ifndef CHGDMAINCAMERA_H
#define CHGDMAINCAMERA_H

// godot includes

//#include <scene/main/viewport.h>

// chrono includes
#include "chrono/core/ChVector.h"

// ChGodotIncludes
#include "chrono_godot/godot_utils/ChGdUtils.h"

namespace chrono {
namespace gd {

class ChGdMainCamera {
  public:
    ChGdMainCamera(Viewport* viewport, ChVector<> location, ChVector<> target_location, ChVector<> up, float FOV);
    ~ChGdMainCamera();

  private:
    // Camera* m_camera;
};

}  // namespace gd
}  // end namespace chrono

#endif
