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
// Renders a wireframe view for triangles
// =============================================================================

#ifndef CHOPENGLWIRES_H
#define CHOPENGLWIRES_H

#include "chrono_opengl/core/ChOpenGLObject.h"
#include "chrono_opengl/core/ChOpenGLMaterial.h"

namespace chrono {
namespace opengl {

/// @addtogroup opengl_module
/// @{

/// Renders a wireframe view for triangles.
class CH_OPENGL_API ChOpenGLWires : public ChOpenGLObject {
  public:
    ChOpenGLWires();
    bool Initialize(const std::vector<glm::vec3>& data, ChOpenGLMaterial mat, ChOpenGLShader* shader);

    virtual void Draw(const glm::mat4& projection, const glm::mat4& view) override;
    virtual void TakeDown() override;
    
    void Update(const std::vector<glm::vec3>& data);
    void SetPointSize(const float& pointsize);

  private:
    typedef ChOpenGLObject super;

    std::vector<glm::vec3> vertices;
    glm::vec4 color;
    float point_size;
    GLuint color_handle;
    GLuint point_size_handle;
};

/// @} opengl_module

}
}

#endif
