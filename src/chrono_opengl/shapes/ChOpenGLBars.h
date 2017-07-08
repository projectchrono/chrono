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

#ifndef CHOPENGLBARS_H
#define CHOPENGLBARS_H

#include "chrono_opengl/core/ChOpenGLObject.h"
#include "chrono_opengl/core/ChOpenGLMaterial.h"
#include "chrono_opengl/core/ChOpenGLVertexAttributes.h"

namespace chrono {
namespace opengl {

/// @addtogroup opengl_module
/// @{

/// Renders rectangular bars.
class CH_OPENGL_API ChOpenGLBars : public ChOpenGLObject {
  public:
    ChOpenGLBars();
    virtual bool Initialize(ChOpenGLShader* _shader);
    virtual void Draw(const glm::mat4& projection = glm::mat4(1), const glm::mat4& view = glm::mat4(1));
    void TakeDown();
    bool PostInitialize();

    void AddBar(double left, double right, double top, double bottom, glm::vec3 color);
    void Update();
    void Clear();

  private:
    std::vector<ChOpenGLVertexAttributesPCN> data;

    typedef ChOpenGLObject super;
};

/// @} opengl_module

}
}

#endif
