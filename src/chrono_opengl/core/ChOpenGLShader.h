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
// Shader() is a sample shader class that loads and compiles the vertex and
// fragment shaders. Based on code provided by Perry Kivolowitz.
// This Shader() class implements or assumes a basic set of uniforms will be
// provided to all shaders derived from it. These are listed below.
// ChOpenGLShader::CommonSetup() can be used by call derived classes to send the
// common values to the shader.
// =============================================================================

#ifndef CHOPENGLSHADER_H
#define CHOPENGLSHADER_H

#include "chrono_opengl/core/ChOpenGLBase.h"
#include <glm/glm.hpp>

namespace chrono {
namespace opengl {

/// @addtogroup opengl_module
/// @{

/// Sample shader class that loads and compiles the vertex and fragment shaders.
class CH_OPENGL_API ChOpenGLShader : public ChOpenGLBase {
  public:
    ChOpenGLShader();
    void TakeDown();
    void Use();
    bool CompileStrings(std::string shader_name, const char* vertex_shader, const char* fragment_shader);
    bool CompileFiles(std::string vertex_shader_file, std::string fragment_shader_file);
    void CompleteInit();

    virtual bool InitializeFiles(std::string vertex_shader_file, std::string fragment_shader_file);
    virtual bool InitializeStrings(std::string shader_name,
                                   const char* vertex_shader_file,
                                   const char* fragment_shader_file);
    virtual void CustomSetup();
    void CommonSetup(const GLfloat* projection, const GLfloat* view);
    void SetTime(const float& _time);
    void SetCamera(const glm::vec3& campos);
    void SetViewport(const glm::ivec2& viewport);
    GLuint GetUniformLocation(std::string name);
    GLuint view_matrix_handle;
    GLuint projection_matrix_handle;
    GLuint time_handle, camera_handle, viewport_size_handle;
    GLuint vertex_shader_id;
    GLuint fragment_shader_id;
    GLuint program_id;
    bool LoadShader(const std::string file_name, GLuint shader_id);
    bool LoadShaderString(const char* shader_string, GLuint shader_id);
    std::string GetShaderLog(GLuint shader_id);
    void CheckGlProgram(GLuint prog);
    float time;
    glm::vec3 camera_pos;
    glm::ivec2 viewport_size;
};

/// @} opengl_module

}
}

#endif
