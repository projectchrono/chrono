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
// Object() is a sample base class upon which drawable objects might
// be derived. It assumes that all drawable objects have some geometry
// to buffer. Based on code provided by Perry Kivolowitz
// =============================================================================

#include "chrono_opengl/core/ChOpenGLObject.h"

using namespace glm;

namespace chrono {
namespace opengl {

/*  Notice the destructor in this case asserts that all resources
 that don't go away by themselves have ALREADY been released. This
 is because the destructor might be called after a GL context has
 been destroyed, so I force the user of this class to call the
 TakeDown() purposefully.
 */

ChOpenGLObject::ChOpenGLObject() {
    this->InternalInitialize();
    shader = 0;
}

ChOpenGLObject::~ChOpenGLObject() {
    assert(this->vertex_array_handle == GLuint(-1));
    assert(this->vertex_data_handle == GLuint(-1));
    assert(this->vertex_element_handle == GLuint(-1));
    assert(this->vertex_ambient_handle == GLuint(-1));
    assert(this->vertex_diffuse_handle == GLuint(-1));
    assert(this->vertex_specular_handle == GLuint(-1));
    assert(this->vertex_model_handle == GLuint(-1));
}
// clear all of the internal data structures used by object
void ChOpenGLObject::TakeDown() {
    // clear the indicies
    this->vertex_indices.clear();

    // check if handle is valid, if it is delete the associated vertex arrays
    if (this->vertex_array_handle != GLuint(-1))
        glDeleteVertexArrays(1, &this->vertex_array_handle);

    if (this->vertex_data_handle != GLuint(-1))
        glDeleteBuffers(1, &this->vertex_data_handle);

    if (this->vertex_element_handle != GLuint(-1))
        glDeleteBuffers(1, &this->vertex_element_handle);

    if (this->vertex_ambient_handle != GLuint(-1))
        glDeleteBuffers(1, &this->vertex_ambient_handle);

    if (this->vertex_diffuse_handle != GLuint(-1))
        glDeleteBuffers(1, &this->vertex_diffuse_handle);

    if (this->vertex_specular_handle != GLuint(-1))
        glDeleteBuffers(1, &this->vertex_specular_handle);

    if (this->vertex_model_handle != GLuint(-1))
        glDeleteBuffers(1, &this->vertex_model_handle);

    this->vertex_array_handle = GLuint(-1);
    this->vertex_data_handle = GLuint(-1);
    this->vertex_element_handle = GLuint(-1);
    this->vertex_ambient_handle = GLuint(-1);
    this->vertex_diffuse_handle = GLuint(-1);
    this->vertex_specular_handle = GLuint(-1);
    this->vertex_model_handle = GLuint(-1);

    this->InternalInitialize();
}

bool ChOpenGLObject::PostGLInitialize(const GLvoid* ptr, GLsizeiptr size) {
    // once all of out data is ready, generate the vertex buffers and bind them
    glGenVertexArrays(1, &vertex_array_handle);
    glBindVertexArray(vertex_array_handle);

    glGenBuffers(1, &vertex_data_handle);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_data_handle);
    glBufferData(GL_ARRAY_BUFFER, size, ptr, GL_STATIC_DRAW);

    glGenBuffers(1, &vertex_element_handle);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vertex_element_handle);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, vertex_indices.size() * sizeof(GLuint), &vertex_indices[0], GL_STATIC_DRAW);

    // glBindBuffer(GL_ARRAY_BUFFER, 0);
    // glBindVertexArray(0);

    return !GLReturnedError("ChOpenGLObject::PostGLInitialize - on exit");
}

bool ChOpenGLObject::Initialize() {
    this->InternalInitialize();
    return true;
}
// Attaches a shader to the object. This can be used in the draw call of any
// inherited classes
void ChOpenGLObject::AttachShader(ChOpenGLShader* new_shader) {
    shader = new_shader;
}

void ChOpenGLObject::InternalInitialize() {
    this->vertex_element_handle = GLuint(-1);

    this->vertex_array_handle = GLuint(-1);
    this->vertex_data_handle = GLuint(-1);
    this->vertex_ambient_handle = GLuint(-1);
    this->vertex_diffuse_handle = GLuint(-1);
    this->vertex_specular_handle = GLuint(-1);
    this->vertex_model_handle = GLuint(-1);
}
}
}
