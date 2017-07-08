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
// OBJ object class, use this to render an obj file
// Stores one mesh object per obj object in the file
// =============================================================================

#ifndef CHOPENGLOBJ_H
#define CHOPENGLOBJ_H

#include "chrono_opengl/core/ChOpenGLObject.h"
#include "chrono_opengl/shapes/ChOpenGLMesh.h"
#include "chrono_opengl/shapes/obj/ChOpenGLOBJLoader.h"
#include "chrono_opengl/core/ChOpenGLMaterial.h"

namespace chrono {
namespace opengl {

/// @addtogroup opengl_module
/// @{

/// Class for rendering an object.
class CH_OPENGL_API ChOpenGLOBJ : public ChOpenGLBase {
  public:
    ChOpenGLOBJ();
    virtual ~ChOpenGLOBJ();
    bool Initialize(std::string filename, ChOpenGLMaterial mat, ChOpenGLShader* shader);
    bool InitializeString(const char* mesh_data, ChOpenGLMaterial mat, ChOpenGLShader* shader);
    void Update(std::vector<glm::mat4>& model);
    virtual void Draw(const glm::mat4& projection, const glm::mat4& modelview);
    virtual void TakeDown();

  protected:
    std::vector<std::vector<glm::vec3> > vertices;
    std::vector<std::vector<glm::vec3> > normals;
    std::vector<std::vector<glm::vec2> > texcoords;
    std::vector<std::vector<GLuint> > indices;
    std::vector<std::string> names;
    std::vector<ChOpenGLMesh> meshes;
    ChOpenGLOBJLoader loader;
};

/// @} opengl_module

}
}

#endif
