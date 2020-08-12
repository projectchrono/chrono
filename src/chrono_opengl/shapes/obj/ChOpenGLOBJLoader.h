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
// Uses the tiny_obj_loader library to load an OBJ file in the proper format
// =============================================================================

#ifndef CHOPENGLOBJLOADER_H
#define CHOPENGLOBJLOADER_H

#include "chrono_opengl/core/ChOpenGLObject.h"
#include "chrono_thirdparty/tinyobjloader/tiny_obj_loader.h"

namespace chrono {
namespace opengl {

/// @addtogroup opengl_module
/// @{

/// Uses the tiny_obj_loader library to load an OBJ file in the proper format.
class ChOpenGLOBJLoader : public ChOpenGLBase {
  public:
    ChOpenGLOBJLoader();
    void LoadObject(const char* mesh_file,
                    std::vector<std::vector<glm::vec3> >& vertices,
                    std::vector<std::vector<glm::vec3> >& normals,
                    std::vector<std::vector<glm::vec2> >& texcoords,
                    std::vector<std::vector<GLuint> >& indices,
                    std::vector<std::string>& names);
    void TakeDown() {}

  private:
};

/// @} opengl_module

}  // namespace opengl
}  // namespace chrono

#endif
