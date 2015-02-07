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
// Class to render contact information
// Authors: Hammad Mazhar
// =============================================================================

#ifndef CHOPENGLCONTACTS_H
#define CHOPENGLCONTACTS_H

#include "chrono_opengl/shapes/ChOpenGLCloud.h"

namespace chrono {
namespace opengl {
class CH_OPENGL_API ChOpenGLContacts {
 public:
  ChOpenGLContacts();
  void Draw(const glm::mat4& projection, const glm::mat4& view);
  void TakeDown();
  void Update();

 private:
  ChOpenGLCloud contacts;
  std::vector<glm::vec3> contact_data;
};
}
}
#endif    // END of CHOPENGLCONTACTS_H