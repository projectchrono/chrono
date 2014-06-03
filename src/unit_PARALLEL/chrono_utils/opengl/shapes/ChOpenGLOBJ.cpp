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

   if (this->GLReturnedError("ChOpenGLOBJ::Initialize - on exit")) {
      return false;
   }

   return true;
}


void ChOpenGLOBJ::TakeDown() {
   for (unsigned int i = 0; i < meshes.size(); i++) {
      meshes[i].TakeDown();
   }

   meshes.clear();
   vertices.clear();
   normals.clear();
   texcoords.clear();
   indices.clear();
   names.clear();

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

