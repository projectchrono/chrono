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
// Renders contact points as a point cloud
// =============================================================================

#ifndef CHOPENGLCONTACTS_H
#define CHOPENGLCONTACTS_H

#include "chrono_opengl/core/ChOpenGLBase.h"
#include "chrono_opengl/shapes/ChOpenGLCloud.h"

#include "chrono/physics/ChSystem.h"

namespace chrono {
class ChSystemParallel;
namespace opengl {

/// @addtogroup opengl_module
/// @{

/// Renders contact points as a point cloud
class CH_OPENGL_API ChOpenGLContacts : public ChOpenGLBase {
  public:
    ChOpenGLContacts();
    bool Initialize(ChOpenGLMaterial mat, ChOpenGLShader* shader);
    void Draw(const glm::mat4& projection, const glm::mat4& view);
    void TakeDown();
    void Update(ChSystem* physics_system);
    void SetPointSize(const float& pointsize) { contacts.SetPointSize(pointsize); }

  private:
    void UpdateChrono(ChSystem* physics_system);
    void UpdateChronoParallel(ChSystemParallel* system);

    ChOpenGLCloud contacts;
    std::vector<glm::vec3> contact_data;
};

/// @} opengl_module

}
}

#endif
