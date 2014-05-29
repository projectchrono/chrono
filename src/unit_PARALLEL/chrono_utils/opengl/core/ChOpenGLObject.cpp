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

#include "chrono_utils/opengl/core/ChOpenGLObject.h"

using namespace std;
using namespace glm;
using namespace chrono::utils;

/*	Notice the destructor in this case asserts that all resources
 that don't go away by themselves have ALREADY been released. This
 is because the destructor might be called after a GL context has
 been destroyed, so I force the user of this class to call the
 TakeDown() purposefully.
 */

ChOpenGLObject::ChOpenGLObject() {
   this->InternalInitialize();
   shader = 0;
}

ChOpenGLObject::~ChOpenGLObject() {
   assert(this->vertex_array_handle == GLuint(-1));
   assert(this->vertex_position_handle == GLuint(-1));
   assert(this->vertex_normal_handle == GLuint(-1));
   assert(this->vertex_ambient_handle == GLuint(-1));
   assert(this->vertex_diffuse_handle == GLuint(-1));
   assert(this->vertex_specular_handle == GLuint(-1));
   assert(this->vertex_coordinate_handle == GLuint(-1));
   assert(this->vertex_element_handle == GLuint(-1));
}
//clear all of the internal data structures used by object
void ChOpenGLObject::TakeDown() {
   //clear the indicies
   this->vertex_indices.clear();

   //check if handle is valid, if it is delete the associated vertex arrays
   if (this->vertex_array_handle != GLuint(-1))
      glDeleteVertexArrays(1, &this->vertex_array_handle);

   if (this->vertex_position_handle != GLuint(-1))
      glDeleteVertexArrays(1, &this->vertex_position_handle);

   if (this->vertex_normal_handle != GLuint(-1))
      glDeleteVertexArrays(1, &this->vertex_normal_handle);

   if (this->vertex_ambient_handle != GLuint(-1))
      glDeleteVertexArrays(1, &this->vertex_ambient_handle);

   if (this->vertex_diffuse_handle != GLuint(-1))
      glDeleteVertexArrays(1, &this->vertex_diffuse_handle);

   if (this->vertex_specular_handle != GLuint(-1))
      glDeleteVertexArrays(1, &this->vertex_specular_handle);

   if (this->vertex_coordinate_handle != GLuint(-1))
      glDeleteBuffers(1, &this->vertex_coordinate_handle);

   if (this->vertex_element_handle != GLuint(-1))
      glDeleteBuffers(1, &this->vertex_element_handle);

   this->InternalInitialize();
}

bool ChOpenGLObject::PostGLInitialize(
      GLuint * position_ptr,
      GLuint * normal_ptr,
      GLuint * ambient_ptr,
      GLuint * diffuse_ptr,
      GLuint * specular_ptr,
      GLsizeiptr size) {
   //once all of out data is ready, generate the vertex buffers and bind them
   glGenVertexArrays(1, &vertex_array_handle);
   glBindVertexArray(vertex_array_handle);

   glGenBuffers(1, &vertex_position_handle);
   glBindBuffer(GL_ARRAY_BUFFER, vertex_position_handle);
   glBufferData(GL_ARRAY_BUFFER, size, position_ptr, GL_STATIC_DRAW);

   glGenBuffers(1, &vertex_normal_handle);
   glBindBuffer(GL_ARRAY_BUFFER, vertex_normal_handle);
   glBufferData(GL_ARRAY_BUFFER, size, normal_ptr, GL_STATIC_DRAW);

   glGenBuffers(1, &vertex_ambient_handle);
   glBindBuffer(GL_ARRAY_BUFFER, vertex_ambient_handle);
   glBufferData(GL_ARRAY_BUFFER, size, ambient_ptr, GL_STATIC_DRAW);

   glGenBuffers(1, &vertex_diffuse_handle);
   glBindBuffer(GL_ARRAY_BUFFER, vertex_diffuse_handle);
   glBufferData(GL_ARRAY_BUFFER, size, diffuse_ptr, GL_STATIC_DRAW);

   glGenBuffers(1, &vertex_specular_handle);
   glBindBuffer(GL_ARRAY_BUFFER, vertex_specular_handle);
   glBufferData(GL_ARRAY_BUFFER, size, specular_ptr, GL_STATIC_DRAW);

   glGenBuffers(1, &vertex_element_handle);
   glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vertex_element_handle);
   glBufferData(GL_ELEMENT_ARRAY_BUFFER, vertex_indices.size() * sizeof(GLuint), &vertex_indices[0], GL_STATIC_DRAW);

   glBindBuffer(GL_ARRAY_BUFFER, 0);
   glBindVertexArray(0);

   return !this->GLReturnedError("ChOpenGLObject::PostGLInitialize - on exit");
}

bool ChOpenGLObject::Initialize() {
   this->InternalInitialize();
   return true;
}
//attaches a shader to the object. This can be used in the draw call of any inherited classes
void ChOpenGLObject::AttachShader(
      ChOpenGLShader * new_shader) {
   shader = new_shader;
}

void ChOpenGLObject::InternalInitialize() {
   this->vertex_element_handle = GLuint(-1);
   this->vertex_position_handle = this->vertex_normal_handle = GLuint(-1);
   this->vertex_ambient_handle = this->vertex_diffuse_handle = this->vertex_specular_handle = GLuint(-1);
   this->vertex_array_handle = this->vertex_coordinate_handle = this->vertex_coordinate_handle = GLuint(-1);
}
