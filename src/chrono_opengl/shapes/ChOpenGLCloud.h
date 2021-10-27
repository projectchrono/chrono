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
// Generic renderable point cloud.
// =============================================================================

#ifndef CHOPENGLCLOUD_H
#define CHOPENGLCLOUD_H

#include "chrono_opengl/core/ChOpenGLObject.h"
#include "chrono_opengl/core/ChOpenGLMaterial.h"

namespace chrono {
namespace opengl {

/// @addtogroup opengl_module
/// @{

/// Generic renderable point cloud.
class CH_OPENGL_API ChOpenGLCloud : public ChOpenGLObject {
  public:
    ChOpenGLCloud();
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
