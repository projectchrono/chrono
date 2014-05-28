/*  Perry Kivolowitz - University of Wisconsin - Madison
 Computer Sciences Department

 A sample hello world like program demonstrating modern
 OpenGL techniques.

 Created:    2/25/13
 Updates:
 */

#include <iostream>
#include "ChOpenGLMesh.h"

using namespace std;
using namespace glm;
using namespace chrono::utils;

ChOpenGLMesh::ChOpenGLMesh()
      :
        ChOpenGLObject() {
}

void ChOpenGLMesh::Generate(
      int rows,
      int cols,
      ChOpenGLMaterial mat,
      bool flip) {
   //Generate a mesh from a simple grid structure
   for (int i = 0; i < cols; i++) {
      for (int j = 0; j < rows; j++) {
         float x = float(i) / float(cols) - .5f;
         float y = float(j) / float(rows) - .5f;
         this->position.push_back(vec3(x, y, 0));
         this->normal.push_back(normalize(vec3(0, 0, 1)));
         this->color.push_back(mat.ambient_color);

         int curRow = i * rows;
         int nextRow = (i + 1) * rows;
         //Depending on how we want to "flip" the triangles, generate them in a differen order
         if ((i + 1) < cols && (j + 1) < rows) {

            if (flip) {
               this->vertex_indices.push_back(nextRow + (j + 1));
               this->vertex_indices.push_back(nextRow + j);
               this->vertex_indices.push_back(curRow + j);

               this->vertex_indices.push_back(curRow + (j + 1));
               this->vertex_indices.push_back(nextRow + (j + 1));
               this->vertex_indices.push_back(curRow + j);
            } else {

               this->vertex_indices.push_back(curRow + j);
               this->vertex_indices.push_back(nextRow + j);
               this->vertex_indices.push_back(nextRow + (j + 1));

               this->vertex_indices.push_back(curRow + j);
               this->vertex_indices.push_back(nextRow + (j + 1));
               this->vertex_indices.push_back(curRow + (j + 1));

            }
         }

      }
   }
}

bool ChOpenGLMesh::Initialize(
      int rows,
      int cols,
      ChOpenGLMaterial mat,
      bool flip) {
   if (this->GLReturnedError("ChOpenGLMesh::Initialize - on entry")) {
      return false;
   }

   if (!super::Initialize()) {
      return false;
   }

   Generate(rows, cols, mat, flip);

   RecomputeNormals();

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
   if (!this->PostGLInitialize((GLuint*) (&this->position[0]), (GLuint*) (&this->normal[0]), (GLuint*) (&this->color[0]), this->position.size() * sizeof(vec3))) {
      return false;
   }

   if (this->GLReturnedError("ChOpenGLMesh::PostInitialize - on exit"))
      return false;

   return true;
}
//This function recomputes normals in the case that we moves verticies around
void ChOpenGLMesh::RecomputeNormals() {
   //set all normals to 0
   for (unsigned int i = 0; i < this->normal.size(); i++) {
      this->normal[i] = vec3(0, 0, 0);
   }

   for (unsigned int i = 0; i < this->vertex_indices.size(); i += 3) {
      int i1 = this->vertex_indices[i];
      int i2 = this->vertex_indices[i + 1];
      int i3 = this->vertex_indices[i + 2];

      //erase triangles if two of the verticies are the same
      if (i1 == i2 || i1 == i3 || i2 == i3) {
         this->vertex_indices.erase(this->vertex_indices.begin() + i + 2);
         this->vertex_indices.erase(this->vertex_indices.begin() + i + 1);
         this->vertex_indices.erase(this->vertex_indices.begin() + i + 0);
      }
   }

   for (unsigned int i = 0; i < this->vertex_indices.size(); i += 3) {
      int i1 = this->vertex_indices[i];
      int i2 = this->vertex_indices[i + 1];
      int i3 = this->vertex_indices[i + 2];

      vec3 v1 = this->position[i1];
      vec3 v2 = this->position[i2];
      vec3 v3 = this->position[i3];

      vec3 d1 = v2 - v1;
      vec3 d2 = v1 - v3;
      //this protects againt the case where we have a 0 d1 or d2
      if (v2 == v1) {
         d1 = vec3(0, 1, 0);
      }
      if (v1 == v3) {
         d2 = vec3(0, 1, 0);
      }
      //determine the normal for this triangle
      vec3 norm = normalize(cross(d1, d2));
      //add the normal to the vertex
      this->normal[i1] += norm;
      this->normal[i2] += norm;
      this->normal[i3] += norm;
   }
   //normalize the normals!
   for (unsigned int i = 0; i < this->normal.size(); i++) {
      vec3 normal = this->normal[i];
      if (normal != vec3(0, 0, 0)) {
         this->normal[i] = -normalize(normal);
      }
   }
}

void ChOpenGLMesh::TakeDown() {
   //Clean up the vertex arrtibute data
   this->position.clear();
   this->normal.clear();
   this->color.clear();
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

   glEnable(GL_DEPTH_TEST);
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

   glEnableVertexAttribArray(0);
   glBindBuffer(GL_ARRAY_BUFFER, vertex_position_handle);
   glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);  // Position

   glEnableVertexAttribArray(1);
   glBindBuffer(GL_ARRAY_BUFFER, vertex_normal_handle);
   glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);  // Normal

   glEnableVertexAttribArray(2);
   glBindBuffer(GL_ARRAY_BUFFER, vertex_color_handle);
   glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);  // Color Ambient

   glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->vertex_element_handle);

   this->GLReturnedError("ChOpenGLMesh::Draw - before draw");
   glDrawElements(GL_TRIANGLES, this->vertex_indices.size(), GL_UNSIGNED_INT, (void*) 0);
   this->GLReturnedError("ChOpenGLMesh::Draw - after draw");
   //unbind everything and cleanup
   glDisableVertexAttribArray(0);
   glDisableVertexAttribArray(1);
   glDisableVertexAttribArray(2);
   glBindVertexArray(0);
   glBindBuffer(GL_ARRAY_BUFFER, 0);

   glUseProgram(0);


   if (this->GLReturnedError("ChOpenGLMesh::Draw - on exit"))
      return;
}

//This is a convenience function to allow the user to use a custom shader with this geometry
void ChOpenGLMesh::Bind() {
   if (this->GLReturnedError("ChOpenGLMesh::Bind - on entry"))
      return;

   glBindVertexArray(this->vertex_array_handle);

   glEnableVertexAttribArray(0);
   glBindBuffer(GL_ARRAY_BUFFER, vertex_position_handle);
   glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);  // Position

   glEnableVertexAttribArray(1);
   glBindBuffer(GL_ARRAY_BUFFER, vertex_normal_handle);
   glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);  // Normal

   glEnableVertexAttribArray(2);
   glBindBuffer(GL_ARRAY_BUFFER, vertex_color_handle);
   glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);  // Color Ambient

   glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->vertex_element_handle);
   this->GLReturnedError("ChOpenGLMesh::Draw - before draw");

   glDrawElements(GL_TRIANGLES, this->vertex_indices.size(), GL_UNSIGNED_INT, (void*) 0);
   this->GLReturnedError("ChOpenGLMesh::Draw - after draw");
   glDisableVertexAttribArray(0);
   glDisableVertexAttribArray(1);
   glDisableVertexAttribArray(2);
   glBindVertexArray(0);
   glBindBuffer(GL_ARRAY_BUFFER, 0);

   if (this->GLReturnedError("ChOpenGLMesh::Bind - on exit"))
      return;
}
