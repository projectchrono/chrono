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
// Renders a wireframe view for triangles
// =============================================================================

#include <iostream>
#include <glm/gtc/type_ptr.hpp>

#include "chrono_opengl/shapes/ChOpenGLBars.h"

using namespace glm;

namespace chrono {
namespace opengl {

ChOpenGLBars::ChOpenGLBars() : ChOpenGLObject() {}

bool ChOpenGLBars::Initialize(ChOpenGLShader* _shader) {
    if (this->GLReturnedError("ChOpenGLBars::Initialize - on entry"))
        return false;

    if (!super::Initialize()) {
        return false;
    }
    PostInitialize();
    this->AttachShader(_shader);

    if (this->GLReturnedError("ChOpenGLBars::Initialize - on exit"))
        return false;

    return true;
}

void ChOpenGLBars::AddBar(double left, double right, double top, double bottom, glm::vec3 color) {
    vec3 A(left, bottom, 0);
    vec3 B(left, top, 0);
    vec3 C(right, top, 0);
    vec3 D(right, bottom, 0);
    int index = (int)this->data.size();

    this->data.push_back(ChOpenGLVertexAttributesPCN(C, color, glm::vec3(1, 0, 0)));
    this->data.push_back(ChOpenGLVertexAttributesPCN(B, color, glm::vec3(1, 0, 0)));
    this->data.push_back(ChOpenGLVertexAttributesPCN(A, color, glm::vec3(1, 0, 0)));

    this->data.push_back(ChOpenGLVertexAttributesPCN(D, color, glm::vec3(1, 0, 0)));
    this->data.push_back(ChOpenGLVertexAttributesPCN(C, color, glm::vec3(1, 0, 0)));
    this->data.push_back(ChOpenGLVertexAttributesPCN(A, color, glm::vec3(1, 0, 0)));

    this->vertex_indices.push_back(index + 0);
    this->vertex_indices.push_back(index + 1);
    this->vertex_indices.push_back(index + 2);
    this->vertex_indices.push_back(index + 3);
    this->vertex_indices.push_back(index + 4);
    this->vertex_indices.push_back(index + 5);
}

void ChOpenGLBars::Clear() {
    this->data.clear();
    this->vertex_indices.clear();
}

void ChOpenGLBars::Update() {
    std::size_t pcn_size = sizeof(ChOpenGLVertexAttributesPCN);

    glBindVertexArray(vertex_array_handle);

    glBindBuffer(GL_ARRAY_BUFFER, vertex_data_handle);
    glBufferData(GL_ARRAY_BUFFER, this->data.size() * pcn_size, &this->data[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vertex_element_handle);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, vertex_indices.size() * sizeof(GLuint), &vertex_indices[0], GL_STATIC_DRAW);
}

bool ChOpenGLBars::PostInitialize() {
    std::size_t pcn_size = sizeof(ChOpenGLVertexAttributesPCN);

    if (this->GLReturnedError("ChOpenGLBars::PostInitialize - on entry"))
        return false;
    // Generation complete bind everything!
    if (this->data.size() == 0) {
        return false;
    }

    if (!this->PostGLInitialize((GLuint*)(&this->data[0]), this->data.size() * pcn_size)) {
        return false;
    }
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, (GLsizei)pcn_size, (GLvoid*)(sizeof(vec3) * 0));  // Position
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, (GLsizei)pcn_size, (GLvoid*)(sizeof(vec3) * 1));  // Color
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, (GLsizei)pcn_size, (GLvoid*)(sizeof(vec3) * 2));  // Normal

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    if (this->GLReturnedError("ChOpenGLBars::PostInitialize - on exit"))
        return false;

    return true;
}

void ChOpenGLBars::TakeDown() {
    data.clear();
    super::TakeDown();
}

void ChOpenGLBars::Draw(const mat4& projection, const mat4& view) {
    if (this->GLReturnedError("ChOpenGLBars::Draw - on entry"))
        return;

    if (data.size() == 0) {
        return;
    }

    glEnable(GL_DEPTH_TEST);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // Enable the shader
    shader->Use();
    this->GLReturnedError("ChOpenGLBars::Draw - after use");
    // Send our common uniforms to the shader
    shader->CommonSetup(value_ptr(projection), value_ptr(view));

    this->GLReturnedError("ChOpenGLBars::Draw - after common setup");
    // Bind and draw! (in this case we draw as triangles)
    glBindVertexArray(this->vertex_array_handle);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_data_handle);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->vertex_element_handle);

    glDrawElements(GL_TRIANGLES, (GLsizei)this->vertex_indices.size(), GL_UNSIGNED_INT, (void*)0);
    this->GLReturnedError("ChOpenGLBars::Draw - after draw");
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glUseProgram(0);

    if (this->GLReturnedError("ChOpenGLBars::Draw - on exit"))
        return;
}
}
}
