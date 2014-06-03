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

#include <iostream>
#include "ChOpenGLMesh.h"

using namespace std;
using namespace glm;
using namespace chrono::utils;

ChOpenGLMesh::ChOpenGLMesh()
      :
        ChOpenGLObject() {
}

bool ChOpenGLMesh::Initialize(
      std::vector<glm::vec3> &vertices,
      std::vector<glm::vec3> &normals,
      std::vector<glm::vec2> &texcoords,
      std::vector<GLuint> &indices,
      ChOpenGLMaterial mat) {
   if (this->GLReturnedError("Mesh::Initialize - on entry")) {
      return false;
   }

   if (!super::Initialize()) {
      return false;
   }

   this->vertex_indices = indices;

   for (unsigned int i = 0; i < vertices.size(); i++) {
      this->data.push_back(ChOpenGLVertexAttributesPADSN(vertices[i], mat.ambient_color, mat.diffuse_color, mat.specular_color, normals[i]));
   }

   PostInitialize();

   if (this->GLReturnedError("ChOpenGLMesh::Initialize - on exit")) {
      return false;
   }

   return true;

}

bool ChOpenGLMesh::PostInitialize() {
   if (this->GLReturnedError("ChOpenGLMesh::PostInitialize - on entry"))
      return false;
   //Generation complete bind everything!
   if (!this->PostGLInitialize((GLuint*) (&this->data[0]), this->data.size() * sizeof(ChOpenGLVertexAttributesPADSN))) {
      return false;
   }


   glEnableVertexAttribArray(0);
   glEnableVertexAttribArray(1);
   glEnableVertexAttribArray(2);
   glEnableVertexAttribArray(3);
   glEnableVertexAttribArray(4);

   glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(ChOpenGLVertexAttributesPADSN), (GLvoid *) (sizeof(vec3) * 0));  //Position
   glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(ChOpenGLVertexAttributesPADSN), (GLvoid *) (sizeof(vec3) * 1));  // Normal
   glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(ChOpenGLVertexAttributesPADSN), (GLvoid *) (sizeof(vec3) * 2));  // Color Ambient
   glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(ChOpenGLVertexAttributesPADSN), (GLvoid *) (sizeof(vec3) * 3));  // Color Diffuse
   glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(ChOpenGLVertexAttributesPADSN), (GLvoid *) (sizeof(vec3) * 4));  // Color Specular

   glBindBuffer(GL_ARRAY_BUFFER, 0);
   glBindVertexArray(0);

   if (this->GLReturnedError("ChOpenGLMesh::PostInitialize - on exit"))
      return false;

   return true;
}
void ChOpenGLMesh::TakeDown() {
   //Clean up the vertex arrtibute data
   this->data.clear();

   super::TakeDown();
}

/*  A note about the index arrays.

 In this example, the index arrays are unsigned ints. If you know
 for certain that the number of vertices will be small enough, you
 can change the index array type to shorts or bytes. This will have
 the two fold benefit of using less storage and transferring fewer
 bytes.
 */

void ChOpenGLMesh::Draw(
      const mat4 & projection,
      const mat4 & modelview) {
   if (this->GLReturnedError("ChOpenGLMesh::Draw - on entry"))
      return;
   //compute the mvp matrix and normal matricies
   mat4 mvp = projection * modelview;
   mat3 nm = inverse(transpose(mat3(modelview)));
   //bind any textures that we need

   //Enable the shader
   shader->Use();
   this->GLReturnedError("ChOpenGLMesh::Draw - after use");
   //Send our common uniforms to the shader
   shader->CommonSetup(value_ptr(projection), value_ptr(modelview), value_ptr(mvp), value_ptr(nm));

   this->GLReturnedError("ChOpenGLMesh::Draw - after common setup");
   //Bind and draw! (in this case we draw as triangles)
   glBindVertexArray(this->vertex_array_handle);
   glBindBuffer(GL_ARRAY_BUFFER, vertex_data_handle);
   glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->vertex_element_handle);

   this->GLReturnedError("ChOpenGLMesh::Draw - before draw");
   glDrawElements(GL_TRIANGLES, this->vertex_indices.size(), GL_UNSIGNED_INT, (void*) 0);
   this->GLReturnedError("ChOpenGLMesh::Draw - after draw");
   //unbind everything and cleanup
   glBindVertexArray(0);
   glBindBuffer(GL_ARRAY_BUFFER, 0);

   glUseProgram(0);

   if (this->GLReturnedError("ChOpenGLMesh::Draw - on exit"))
      return;
}
