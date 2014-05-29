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
// Generic renderable mesh. Based on code provided by Perry Kivolowitz.
// Authors: Hammad Mazhar
// =============================================================================

#ifndef CHOPENGLMESH_H
#define CHOPENGLMESH_H

/*	Based on code provided by: Perry Kivolowitz
 Draws a planar mesh
 Not super usefull in itself but it can be wrapped into other shapes
 Also, texture coordinates are provided for texturing purposes
 */

#include "chrono_utils/opengl/core/ChOpenGLObject.h"

namespace chrono {
namespace utils {
class CH_UTILS_OPENGL_API ChOpenGLMesh : public ChOpenGLObject {
 public:
   ChOpenGLMesh();
   bool Initialize(
         std::vector<glm::vec3> &vertices,
         std::vector<glm::vec3> &normals,
         std::vector<glm::vec2> &texcoords,
         std::vector<GLuint> &indices,
         ChOpenGLMaterial mat);
   bool PostInitialize();
   virtual void Draw(
         const glm::mat4 & projection,
         const glm::mat4 & modelview);

   void TakeDown();
 protected:
   //std::vector<ChOpenGLVertexAttributesPCN> vertices;
   std::vector<glm::vec3> position;
   std::vector<glm::vec3> normal;
   std::vector<glm::vec3> color_ambient;
   std::vector<glm::vec3> color_diffuse;
   std::vector<glm::vec3> color_specular;
   std::vector<glm::vec2> texcoord;
   typedef ChOpenGLObject super;
};
}
}

#endif  // END of CHOPENGLMESH_H
