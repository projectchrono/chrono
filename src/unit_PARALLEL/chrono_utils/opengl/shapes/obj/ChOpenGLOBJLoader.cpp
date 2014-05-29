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
// Uses the tiny_obj_loader library to load an OBJ file in the proper format
// Authors: Hammad Mazhar
// =============================================================================

#include <iostream>
#include "ChOpenGLOBJLoader.h"
#include <sstream>
#include <string>
using namespace std;
using namespace glm;
using namespace chrono::utils;
ChOpenGLOBJLoader::ChOpenGLOBJLoader() {
}

//load an obj mesh. Each mesh can have multiple sub meshes
void ChOpenGLOBJLoader::LoadObject(
      string fname,
      vector<vector<glm::vec3> > &vertices,
      vector<vector<glm::vec3> > &normals,
      vector<vector<glm::vec2> > &texcoords,
      vector<vector<GLuint> > &indices,
      vector<string> & names) {
   std::vector<tinyobj::shape_t> shapes;

   std::string err = tinyobj::LoadObj(shapes, fname.c_str());

   std::cout << "# of shapes : " << shapes.size() << std::endl;

   vertices.resize(shapes.size());
   normals.resize(shapes.size());
   texcoords.resize(shapes.size());
   indices.resize(shapes.size());
   names.resize(shapes.size());

   //convert between mesh loader data structure and vector data structure
   for (size_t i = 0; i < shapes.size(); i++) {

      vertices[i].resize(shapes[i].mesh.positions.size() / 3);
      normals[i].resize(shapes[i].mesh.normals.size() / 3);
      texcoords[i].resize(shapes[i].mesh.texcoords.size() / 2);
      indices[i].resize(shapes[i].mesh.indices.size());
      names[i] = shapes[i].name;

      cout << shapes[i].mesh.positions.size() / 3 << " " << shapes[i].mesh.normals.size() / 3 << " " << shapes[i].mesh.texcoords.size() / 2 << " " << shapes[i].mesh.indices.size()
           << endl;

      for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++) {
         vertices[i][v] = vec3(shapes[i].mesh.positions[3 * v + 0], shapes[i].mesh.positions[3 * v + 1], shapes[i].mesh.positions[3 * v + 2]);
      }
      for (size_t n = 0; n < shapes[i].mesh.normals.size() / 3; n++) {
         normals[i][n] = vec3(shapes[i].mesh.normals[3 * n + 0], shapes[i].mesh.normals[3 * n + 1], shapes[i].mesh.normals[3 * n + 2]);
      }
      for (size_t t = 0; t < shapes[i].mesh.texcoords.size() / 2; t++) {
         texcoords[i][t] = vec2(shapes[i].mesh.texcoords[2 * t + 0], shapes[i].mesh.texcoords[2 * t + 1] * -1);
      }
      for (size_t f = 0; f < shapes[i].mesh.indices.size(); f++) {
         indices[i][f] = shapes[i].mesh.indices[f];
      }

   }
}
