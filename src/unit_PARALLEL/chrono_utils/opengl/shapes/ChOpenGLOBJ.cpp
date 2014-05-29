/*  Perry Kivolowitz - University of Wisconsin - Madison
 Computer Sciences Department

 A sample hello world like program demonstrating modern
 OpenGL techniques.

 Created:    2/25/13
 Updates:
 */

#include <iostream>
#include "ChOpenGLOBJ.h"

using namespace std;
using namespace glm;
using namespace chrono::utils;

ChOpenGLOBJ::ChOpenGLOBJ() {
}

bool ChOpenGLOBJ::Initialize(
      string filename,
      ChOpenGLMaterial mat,
      ChOpenGLShader * shader) {
   if (this->GLReturnedError("ChOpenGLOBJ::Initialize - on entry")) {
      return false;
   }

   loader.LoadObject(filename, vertices, normals, texcoords, indices, names);
   meshes.resize(vertices.size());
   for (unsigned int i = 0; i < meshes.size(); i++) {
      meshes[i].Initialize(vertices[i], normals[i], texcoords[i], indices[i], mat);
      meshes[i].AttachShader(shader);
   }

   PostInitialize();

   if (this->GLReturnedError("ChOpenGLOBJ::Initialize - on exit")) {
      return false;
   }

   return true;

}

bool ChOpenGLOBJ::PostInitialize() {
   if (this->GLReturnedError("ChOpenGLOBJ::PostInitialize - on entry"))
      return false;
   //Generation complete bind everything!
//   if (!this->PostGLInitialize((GLuint*) (&this->position[0]), (GLuint*) (&this->normal[0]), (GLuint*) (&this->color[0]), this->position.size() * sizeof(vec3))) {
//      return false;
//   }

   if (this->GLReturnedError("ChOpenGLOBJ::PostInitialize - on exit"))
      return false;

   return true;
}

void ChOpenGLOBJ::TakeDown() {


}

void ChOpenGLOBJ::Draw(
      const mat4 & projection,
      const mat4 & modelview) {
   if (this->GLReturnedError("ChOpenGLOBJ::Draw - on entry"))
      return;

   for (unsigned int i = 0; i < meshes.size(); i++) {
      meshes[i].Draw(projection, modelview);
   }

   if (this->GLReturnedError("ChOpenGLOBJ::Draw - on exit"))
      return;
}

