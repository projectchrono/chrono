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
// Authors: Hammad Mazhar
// =============================================================================
// OBJ object class, use this to render an obj file
// Stores one mesh object per obj object in the file
// =============================================================================

#include <iostream>

#include "chrono_opengl/shapes/ChOpenGLOBJ.h"

using namespace glm;

namespace chrono {
namespace opengl {

ChOpenGLOBJ::ChOpenGLOBJ() {}
ChOpenGLOBJ::~ChOpenGLOBJ() {}
bool ChOpenGLOBJ::Initialize(std::string filename, ChOpenGLMaterial mat, ChOpenGLShader* shader) {
    if (GLReturnedError("ChOpenGLOBJ::Initialize - on entry")) {
        return false;
    }

    std::ifstream ifs(filename);

    std::ifstream in(filename.c_str(), std::ios::in);
    if (!in) {
        std::cout << "Cannot open file [" << filename << "]" << std::endl;
    }
    std::string contents;
    in.seekg(0, std::ios::end);
    contents.resize(in.tellg());
    in.seekg(0, std::ios::beg);
    in.read(&contents[0], contents.size());
    in.close();

    loader.LoadObject(contents.c_str(), vertices, normals, texcoords, indices, names);
    meshes.resize(vertices.size());
    for (unsigned int i = 0; i < meshes.size(); i++) {
        meshes[i].Initialize(vertices[i], normals[i], texcoords[i], indices[i], mat);
        meshes[i].AttachShader(shader);
    }

    if (GLReturnedError("ChOpenGLOBJ::Initialize - on exit")) {
        return false;
    }

    return true;
}

bool ChOpenGLOBJ::InitializeString(const char* mesh_data, ChOpenGLMaterial mat, ChOpenGLShader* shader) {
    if (GLReturnedError("ChOpenGLOBJ::Initialize - on entry")) {
        return false;
    }

    loader.LoadObject(mesh_data, vertices, normals, texcoords, indices, names);
    meshes.resize(vertices.size());
    for (unsigned int i = 0; i < meshes.size(); i++) {
        meshes[i].Initialize(vertices[i], normals[i], texcoords[i], indices[i], mat);
        meshes[i].AttachShader(shader);
    }

    if (GLReturnedError("ChOpenGLOBJ::Initialize - on exit")) {
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
void ChOpenGLOBJ::Update(std::vector<glm::mat4>& model) {
    for (unsigned int i = 0; i < meshes.size(); i++) {
        meshes[i].Update(model);
    }
}
void ChOpenGLOBJ::Draw(const mat4& projection, const mat4& view) {
    if (GLReturnedError("ChOpenGLOBJ::Draw - on entry"))
        return;

    for (unsigned int i = 0; i < meshes.size(); i++) {
        meshes[i].Draw(projection, view);
    }

    if (GLReturnedError("ChOpenGLOBJ::Draw - on exit"))
        return;
}
}
}
