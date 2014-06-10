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
// Generic renderable point cloud. Based on code provided by Perry Kivolowitz.
// Authors: Hammad Mazhar
// =============================================================================

#ifndef CHOPENGLCLOUD_H
#define CHOPENGLCLOUD_H

#include "chrono_utils/opengl/core/ChOpenGLObject.h"

namespace chrono {
namespace utils {
class CH_UTILS_OPENGL_API ChOpenGLCloud : public ChOpenGLObject {
 public:
   ChOpenGLCloud();
   virtual bool Initialize(
         const std::vector<glm::vec3>& data);
   virtual void Draw(
         const glm::mat4 & projection,
         const glm::mat4 & modelview);
   void TakeDown();
   void Update( const std::vector<glm::vec3>& data);
 private:
   std::vector<glm::vec3> vertices;
   typedef ChOpenGLObject super;
};
}
}
#endif  // END of CHOPENGLCLOUD_H
