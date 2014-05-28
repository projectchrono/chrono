/*  Perry Kivolowitz - University of Wisconsin - Madison
 Computer Sciences Department

 A sample hello world like program demonstrating modern
 OpenGL techniques.

 Created:    2/25/13
 Updates:
 */

#include <iostream>
#include "ChOpenGLFrustum.h"

using namespace std;
using namespace glm;
using namespace chrono;
using namespace chrono::utils;

ChOpenGLFrustum::ChOpenGLFrustum()
      :
        ChOpenGLObject() {
}

//This function take a position, normal and size as input and generates a plane
void ChOpenGLFrustum::InitPlane(
      const glm::vec3 pos,
      const glm::vec3 normal,
      const vec2 size,
      ChOpenGLMaterial mat) {
   vec3 u, v;
   vec3 N = normalize(normal);
   //Determine our basis
   if (N == vec3(0, 0, 1)) {
      u = vec3(1, 0, 0);
      v = vec3(0, 1, 0);
   } else if (N == vec3(0, 0, -1)) {
      u = vec3(0, 1, 0);
      v = vec3(1, 0, 0);
   } else {
      u = cross(N, vec3(0, 0, 1));
      v = cross(N, u);
   }
   //Find horizontal and vertical size
   vec3 P0 = pos;
   vec3 fu = u * size.x;
   vec3 fv = v * size.y;
   //Move from center out to the required points
   vec3 P1 = P0 - fu - fv;
   vec3 P2 = P0 + fu - fv;
   vec3 P3 = P0 + fu + fv;
   vec3 P4 = P0 - fu + fv;
   //generate our verticies with color and normal data
   this->position.push_back(P1);
   this->position.push_back(P2);
   this->position.push_back(P3);
   this->position.push_back(P4);

   this->normal.push_back(N);
   this->normal.push_back(N);
   this->normal.push_back(N);
   this->normal.push_back(N);

   this->color.push_back(mat.ambient_color);
   this->color.push_back(mat.ambient_color);
   this->color.push_back(mat.ambient_color);
   this->color.push_back(mat.ambient_color);

   //Generate connectivity info
   this->vertex_indices.push_back(this->position.size() - 4);
   this->vertex_indices.push_back(this->position.size() - 3);
   this->vertex_indices.push_back(this->position.size() - 2);
   this->vertex_indices.push_back(this->position.size() - 1);

}

//Simpler form of init plane. Provide the 4 corner points
void ChOpenGLFrustum::InitPlane(
      glm::vec3 P1,
      glm::vec3 P2,
      glm::vec3 P3,
      glm::vec3 P4,
      ChOpenGLMaterial mat) {

   //compute normal from the 4 points and normalize
   vec3 v1 = P2 - P1;
   vec3 v2 = P3 - P2;
   vec3 N = normalize(cross(v1, v2));

   //generate our verticies with color and normal data

   this->position.push_back(P1);
   this->position.push_back(P2);
   this->position.push_back(P3);
   this->position.push_back(P4);

   this->normal.push_back(N);
   this->normal.push_back(N);
   this->normal.push_back(N);
   this->normal.push_back(N);

   this->color.push_back(mat.ambient_color);
   this->color.push_back(mat.ambient_color);
   this->color.push_back(mat.ambient_color);
   this->color.push_back(mat.ambient_color);

   //Generate connectivity info
   this->vertex_indices.push_back(this->position.size() - 4);
   this->vertex_indices.push_back(this->position.size() - 3);
   this->vertex_indices.push_back(this->position.size() - 2);
   this->vertex_indices.push_back(this->position.size() - 1);

}

bool ChOpenGLFrustum::Initialize(
      const glm::vec2 base,
      const glm::vec2 top,
      const float height,
      ChOpenGLMaterial mat) {
   if (this->GLReturnedError("Frustum::Initialize - on entry"))
      return false;
   if (!super::Initialize())
      return false;
   //Generate top and bottom with the specified sizes
   InitPlane(vec3(0, height, 0), vec3(0, 1, 0), top, mat);
   InitPlane(vec3(0, -height, 0), vec3(0, -1, 0), base, mat);

   //Create 4 planes to complete frustum
   {
      vec3 p1 = vec3(-top.x, height, -top.y);
      vec3 p2 = vec3(top.x, height, -top.y);

      vec3 p3 = vec3(base.x, -height, -base.y);
      vec3 p4 = vec3(-base.x, -height, -base.y);

      InitPlane(p1, p2, p3, p4, mat);
   }
   {
      vec3 p1 = vec3(-top.x, height, top.y);
      vec3 p2 = vec3(top.x, height, top.y);

      vec3 p3 = vec3(base.x, -height, base.y);
      vec3 p4 = vec3(-base.x, -height, base.y);

      InitPlane(p4, p3, p2, p1, mat);
   }
   {
      vec3 p1 = vec3(top.x, height, -top.y);
      vec3 p2 = vec3(top.x, height, top.y);

      vec3 p3 = vec3(base.x, -height, base.y);
      vec3 p4 = vec3(base.x, -height, -base.y);

      InitPlane(p1, p2, p3, p4, mat);
   }
   {
      vec3 p1 = vec3(-top.x, height, -top.y);
      vec3 p2 = vec3(-top.x, height, top.y);

      vec3 p3 = vec3(-base.x, -height, base.y);
      vec3 p4 = vec3(-base.x, -height, -base.y);

      InitPlane(p4, p3, p2, p1, mat);
   }

   //Generation complete bind everything!
   if (!this->PostGLInitialize((GLuint*) (&this->position[0]), (GLuint*) (&this->normal[0]), (GLuint*) (&this->color[0]), this->position.size() * sizeof(vec3))) {
      return false;
   }

   if (this->GLReturnedError("Frustum::Initialize - on exit"))
      return false;

   return true;
}

void ChOpenGLFrustum::TakeDown() {
   //Cleanup vertex attrib data
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

void ChOpenGLFrustum::Draw(
      const mat4 & projection,
      const mat4 & modelview) {
   if (this->GLReturnedError("Frustum::Draw - on entry"))
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

   this->GLReturnedError("Frustum::Draw - before draw");
   glDrawElements(GL_PATCHES, this->vertex_indices.size(), GL_UNSIGNED_INT, (void*) 0);
   this->GLReturnedError("Frustum::Draw - after draw");
   //unbind everything and cleanup
   glDisableVertexAttribArray(0);
   glDisableVertexAttribArray(1);
   glDisableVertexAttribArray(2);
   glBindVertexArray(0);
   glBindBuffer(GL_ARRAY_BUFFER, 0);

   glUseProgram(0);

   if (this->GLReturnedError("Frustum::Draw - on exit"))
      return;
}
