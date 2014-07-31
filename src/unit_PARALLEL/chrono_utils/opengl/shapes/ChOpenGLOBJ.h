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
// OBJ object class, use this to render an obj file
// Stores one mesh object per obj object in the file
// Authors: Hammad Mazhar
// =============================================================================

#ifndef CHOPENGLOBJ_H
#define CHOPENGLOBJ_H

#include "chrono_utils/opengl/core/ChOpenGLObject.h"
#include "chrono_utils/opengl/shapes/ChOpenGLMesh.h"
#include "chrono_utils/opengl/shapes/obj/ChOpenGLOBJLoader.h"
namespace chrono {
namespace opengl {
class CH_UTILS_OPENGL_API ChOpenGLOBJ : public ChOpenGLBase {
 public:
   ChOpenGLOBJ();
   bool Initialize(
         std::string filename,
         ChOpenGLMaterial mat,
         ChOpenGLShader * shader);
   void Update(std::vector<glm::mat4> & model);
   virtual void Draw(
         const glm::mat4 & projection,
         const glm::mat4 & modelview);
   void TakeDown();
 protected:

   std::vector<std::vector<glm::vec3> > vertices;
   std::vector<std::vector<glm::vec3> > normals;
   std::vector<std::vector<glm::vec2> > texcoords;
   std::vector<std::vector<GLuint> > indices;
   std::vector<std::string> names;
   std::vector<ChOpenGLMesh> meshes;
   ChOpenGLOBJLoader loader;
};
}
}

#endif  // END of CHOPENGLOBJ_H
