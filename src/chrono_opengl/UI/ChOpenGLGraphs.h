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
// Class to render simple plots for the UI
// =============================================================================

#ifndef CHOPENGLGRAPHS_H
#define CHOPENGLGRAPHS_H

#include "chrono_opengl/core/ChOpenGLObject.h"
#include "chrono_opengl/core/ChOpenGLMaterial.h"
#include "chrono_opengl/shapes/ChOpenGLWires.h"

#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace opengl {

/// @addtogroup opengl_module
/// @{

/// Class to render simple plots for the UI
class CH_OPENGL_API ChOpenGLGraphs {
  public:
    ChOpenGLGraphs();
    bool Initialize(ChOpenGLMaterial mat, ChOpenGLShader* _shader);
    void Draw(const glm::mat4& projection, const glm::mat4& modelview);
    void TakeDown();
    void Update(ChSystem* physics_system, const glm::ivec2& window_size);

  private:
    std::vector<glm::vec3> plot_data;
    ChOpenGLWires plots;
};

/// @} opengl_module

}
}

#endif
