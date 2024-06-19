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
// Generic renderable triangle mesh.
// =============================================================================

#ifndef CHOPENGLMESH_H
#define CHOPENGLMESH_H

/*	Based on code provided by: Perry Kivolowitz
 Draws a planar mesh
 Not super useful in itself but it can be wrapped into other shapes
 Also, texture coordinates are provided for texturing purposes
 */

#include "chrono_opengl/core/ChOpenGLObject.h"
#include "chrono_opengl/core/ChOpenGLVertexAttributes.h"
#include "chrono_opengl/core/ChOpenGLMaterial.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

namespace chrono {
namespace opengl {

/// @addtogroup opengl_module
/// @{

/// Generic renderable triangle mesh.
class CH_OPENGL_API ChOpenGLMesh : public ChOpenGLObject {
  public:
    ChOpenGLMesh();
    bool Initialize(std::vector<glm::vec3>& vertices,
                    std::vector<glm::vec3>& normals,
                    std::vector<glm::vec2>& texcoords,
                    std::vector<GLuint>& indices,
                    ChOpenGLMaterial mat);
    bool Initialize(chrono::ChVisualShapeTriangleMesh* tri_mesh, ChOpenGLMaterial mat);
    bool PostInitialize();
    void Update(std::vector<glm::mat4>& model);

    virtual void Draw(const glm::mat4& projection, const glm::mat4& view) override;
    virtual void TakeDown() override;

  protected:
    typedef ChOpenGLObject super;

    std::vector<ChOpenGLVertexAttributesPN> data;
    glm::vec3 ambient;
    glm::vec3 specular;
    glm::vec3 diffuse;
    int size;
};

/// @} opengl_module

}
}

#endif
