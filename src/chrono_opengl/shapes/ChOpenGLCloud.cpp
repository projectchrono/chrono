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
// Generic renderable point cloud.
// =============================================================================

#include <iostream>
#include <glm/gtc/type_ptr.hpp>

#include "chrono_opengl/shapes/ChOpenGLCloud.h"

using namespace glm;

namespace chrono {
namespace opengl {

ChOpenGLCloud::ChOpenGLCloud() : ChOpenGLObject() {
    point_size = .04f;
    point_size_handle = BAD_GL_VALUE;
    color_handle = BAD_GL_VALUE;
}

bool ChOpenGLCloud::Initialize(const std::vector<glm::vec3>& data, ChOpenGLMaterial mat, ChOpenGLShader* _shader) {
    if (GLReturnedError("Background::Initialize - on entry"))
        return false;

    if (!super::Initialize()) {
        return false;
    }

    this->vertices = data;
    this->vertex_indices.resize(data.size());
    for (int i = 0; i < data.size(); i++) {
        this->vertex_indices[i] = i;
    }

    if (!this->PostGLInitialize((GLuint*)(&this->vertices[0]), this->vertices.size() * sizeof(vec3))) {
        return false;
    }

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), 0);  // Position

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    if (GLReturnedError("Cloud::Initialize - on exit"))
        return false;

    this->AttachShader(_shader);
    color_handle = this->shader->GetUniformLocation("color");
    point_size_handle = this->shader->GetUniformLocation("point_size");
    color = glm::vec4(mat.diffuse_color, 1);
    return true;
}
void ChOpenGLCloud::Update(const std::vector<glm::vec3>& data) {
    this->vertices = data;
    this->vertex_indices.resize(data.size());
    for (int i = 0; i < data.size(); i++) {
        this->vertex_indices[i] = i;
    }
}
void ChOpenGLCloud::TakeDown() {
    vertices.clear();
    super::TakeDown();
}

void ChOpenGLCloud::Draw(const mat4& projection, const mat4& view) {
    if (GLReturnedError("ChOpenGLCloud::Draw - on entry"))
        return;
    if (this->vertices.size() == 0)
        return;
    glEnable(GL_DEPTH_TEST);
    // compute the mvp matrix and normal matricies
    // mat4 mvp = projection * modelview;
    // mat3 nm = inverse(transpose(mat3(modelview)));

    // Enable the shader
    shader->Use();
    GLReturnedError("ChOpenGLCloud::Draw - after use");
    // Send our common uniforms to the shader
    shader->CommonSetup(value_ptr(projection), value_ptr(view));
    glUniform4fv(color_handle, 1, glm::value_ptr(color));
    glUniform1fv(point_size_handle, 1, &point_size);
    GLReturnedError("ChOpenGLCloud::Draw - after common setup");
    // Bind and draw! (in this case we draw as triangles)
    glBindVertexArray(this->vertex_array_handle);

    glBindBuffer(GL_ARRAY_BUFFER, vertex_data_handle);
    glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(vec3), &this->vertices[0], GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vertex_element_handle);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, vertex_indices.size() * sizeof(GLuint), &vertex_indices[0], GL_DYNAMIC_DRAW);

    glDrawElements(GL_POINTS, (GLsizei)this->vertex_indices.size(), GL_UNSIGNED_INT, (void*)0);

    GLReturnedError("ChOpenGLCloud::Draw - after draw");
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glUseProgram(0);

    if (GLReturnedError("ChOpenGLCloud::Draw - on exit"))
        return;
}

void ChOpenGLCloud::SetPointSize(const float& pointsize) {
    point_size = pointsize;
}
}
}
