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
// Authors: Hammad Mazhar
// =============================================================================
// Material Class
// Stores ambient, diffuse, specular and glow colors
// Useful when sending material info to a renderable object
// =============================================================================

#ifndef CHOPENGLMATERIAL_H
#define CHOPENGLMATERIAL_H

#include "chrono_opengl/core/ChOpenGLBase.h"
#include <glm/glm.hpp>
namespace chrono {
namespace opengl {

class ChOpenGLMaterial : public ChOpenGLBase {
 public:
  // constructor accepts 4 colors and sets them.
  ChOpenGLMaterial(glm::vec3 a, glm::vec3 d, glm::vec3 s) {
    ambient_color = a;
    diffuse_color = d;
    specular_color = s;
  }
  // Dont need to take anything down so this function is empty
  void TakeDown() {}
  glm::vec3 ambient_color;
  glm::vec3 diffuse_color;
  glm::vec3 specular_color;
};
}
}
#endif  // END of CHOPENGLMATERIAL_H
