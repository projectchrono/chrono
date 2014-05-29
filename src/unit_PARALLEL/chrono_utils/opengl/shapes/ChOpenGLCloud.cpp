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

#include <iostream>
#include "ChOpenGLCloud.h"

using namespace std;
using namespace glm;
using namespace chrono::utils;

ChOpenGLCloud::ChOpenGLCloud()
      :
        ChOpenGLObject() {
}

bool ChOpenGLCloud::Initialize(
      const std::vector<glm::vec3>& data) {
   if (this->GLReturnedError("Background::Initialize - on entry"))
      return false;

   if (vertex_position_handle != GLuint(-1)) {
      glDeleteBuffers(1, &vertex_position_handle);
   }

   if (!super::Initialize()) {
      return false;
   }

   for (int i = 0; i < data.size(); i++) {
      this->vertices.push_back(data[i]);
   }
   if (vertex_array_handle != GLuint(-1)) {
      glGenVertexArrays(1, &vertex_array_handle);
   }
   glBindVertexArray(vertex_array_handle);

   glGenBuffers(1, &vertex_position_handle);
   glBindBuffer(GL_ARRAY_BUFFER, vertex_position_handle);
   glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(vec3), &this->vertices[0], GL_STATIC_DRAW);

   glBindBuffer(GL_ARRAY_BUFFER, 0);
   glBindVertexArray(0);

   if (this->GLReturnedError("Cloud::Initialize - on exit"))
      return false;

   return true;
}

void ChOpenGLCloud::TakeDown() {
   vertices.clear();
   super::TakeDown();
}

void ChOpenGLCloud::Draw(
      const mat4 & projection,
      const mat4 & modelview) {
   if (this->GLReturnedError("Mesh::Draw - on entry"))
      return;
   glEnable(GL_DEPTH_TEST);
   //compute the mvp matrix and normal matricies
   mat4 mvp = projection * modelview;
   mat3 nm = inverse(transpose(mat3(modelview)));

   //Enable the shader
   shader->Use();
   this->GLReturnedError("Mesh::Draw - after use");
   //Send our common uniforms to the shader
   shader->CommonSetup(value_ptr(projection), value_ptr(modelview), value_ptr(mvp), value_ptr(nm));

   this->GLReturnedError("Mesh::Draw - after common setup");
   //Bind and draw! (in this case we draw as triangles)
   glBindVertexArray(this->vertex_array_handle);

   glEnableVertexAttribArray(0);
   glBindBuffer(GL_ARRAY_BUFFER, vertex_position_handle);
   glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);  // Position

   glDrawElements(GL_POINTS, this->vertex_indices.size(), GL_UNSIGNED_INT, (void*) 0);
   this->GLReturnedError("ChOpenGLMesh::Draw - after draw");
   glDisableVertexAttribArray(0);

   glBindVertexArray(0);
   glBindBuffer(GL_ARRAY_BUFFER, 0);
   this->GLReturnedError("Mesh::Draw - after draw");
   glUseProgram(0);

   if (this->GLReturnedError("Mesh::Draw - on exit"))
      return;
}
