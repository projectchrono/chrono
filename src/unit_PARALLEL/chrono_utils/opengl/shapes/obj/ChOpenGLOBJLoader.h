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
// Uses the tiny_obj_loader library to load an OBJ file in the proper format
// Authors: Hammad Mazhar
// =============================================================================

#ifndef CHOPENGLOBJLOADER_H
#define CHOPENGLOBJLOADER_H

#include "chrono_utils/opengl/core/ChOpenGLObject.h"
#include "tiny_obj_loader.h"
namespace chrono {
namespace utils {
class ChOpenGLOBJLoader : public ChOpenGLBase {
 public:
   ChOpenGLOBJLoader();
   void LoadObject(
         std::string fname,
         std::vector<std::vector<glm::vec3> > &vertices,
         std::vector<std::vector<glm::vec3> > &normals,
         std::vector<std::vector<glm::vec2> > &texcoords,
         std::vector<std::vector<GLuint> > &indices,
         std::vector<std::string> & names);
   void TakeDown() {

   }

 private:

};
}
}

#endif // END of CHOPENGLOBJLOADER_H
