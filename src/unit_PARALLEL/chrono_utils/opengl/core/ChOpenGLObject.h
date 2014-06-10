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
// Object() is a sample base class upon which drawable objects might
// be derived. It assumes that all drawable objects have some geometry
// to buffer. Based on code provided by Perry Kivolowitz
// Authors: Hammad Mazhar
// =============================================================================

#ifndef CHOPENGLOBJECT_H
#define CHOPENGLOBJECT_H

#include "ChOpenGLBase.h"
#include "ChOpenGLShader.h"
#include "ChOpenGLVertexAttributes.h"
#include "ChOpenGLMaterial.h"

namespace chrono {
namespace utils {
class CH_UTILS_OPENGL_API ChOpenGLObject : public ChOpenGLBase {
 public:
   ChOpenGLObject();
   virtual ~ChOpenGLObject();
   virtual void TakeDown();
   virtual bool Initialize();
   virtual bool PostGLInitialize(
         const GLvoid * ptr,
         GLsizeiptr size);
   virtual void Draw(
         const glm::mat4 & projection,
         const glm::mat4 & modelview) = 0;
   void AttachShader(
         ChOpenGLShader * new_shader);

 protected:
   GLuint vertex_array_handle;
   GLuint vertex_data_handle;
   GLuint vertex_element_handle;
   GLuint vertex_ambient_handle;
   GLuint vertex_diffuse_handle;
   GLuint vertex_specular_handle;
   GLuint vertex_model_handle;
   std::vector<GLuint> vertex_indices;
   ChOpenGLShader * shader;
 private:
   void InternalInitialize();
};

}
}

#endif  // END of CHOPENGLOBJECT_H
