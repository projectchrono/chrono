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

#include "chrono_opengl/core/ChOpenGLShader.h"

#include <glm/gtc/type_ptr.hpp>
#include <sstream>

using namespace glm;

namespace chrono {
namespace opengl {

ChOpenGLShader::ChOpenGLShader() {
    this->vertex_shader_id = BAD_GL_VALUE;
    this->fragment_shader_id = BAD_GL_VALUE;
    this->program_id = BAD_GL_VALUE;
    this->view_matrix_handle = BAD_GL_VALUE;
    this->projection_matrix_handle = BAD_GL_VALUE;
    this->time_handle = BAD_GL_VALUE;
    this->camera_handle = BAD_GL_VALUE;
    this->viewport_size_handle = BAD_GL_VALUE;
    time = 0;
}

/*  This Shader() class implements or assumes a basic set of uniforms will
 be
 provided to all shaders derived from it. These are listed below.
 ChOpenGLShader::CommonSetup() can be used by call derived classes to send the
 common values to the shader. Values unique to the derived class can be
 loaded with the CustomShader() function.
 */

void ChOpenGLShader::CommonSetup(const GLfloat* projection, const GLfloat* view) {
    if (this->projection_matrix_handle != BAD_GL_VALUE) {
        glUniformMatrix4fv(this->projection_matrix_handle, 1, GL_FALSE, projection);
    }
    if (this->view_matrix_handle != BAD_GL_VALUE) {
        glUniformMatrix4fv(this->view_matrix_handle, 1, GL_FALSE, view);
    }
    if (this->viewport_size_handle != BAD_GL_VALUE) {
        glUniform2iv(viewport_size_handle, 1, glm::value_ptr(viewport_size));
    }
    GLReturnedError("Draw - after common setup");
}

void ChOpenGLShader::SetTime(const float& _time) {
    time = _time;
}
void ChOpenGLShader::SetCamera(const vec3& campos) {
    camera_pos = campos;
}
void ChOpenGLShader::SetViewport(const glm::ivec2& viewport) {
    viewport_size = viewport;
}
void ChOpenGLShader::Use() {
    assert(this->program_id != BAD_GL_VALUE);
    glUseProgram(this->program_id);

    if (this->time_handle != BAD_GL_VALUE) {
        glUniform1f(this->time_handle, time);
    }
    if (this->camera_handle != BAD_GL_VALUE) {
        glUniform3fv(this->camera_handle, 1, glm::value_ptr(camera_pos));
    }
    GLReturnedError("ChOpenGLShader::Use - exit");
}

bool ChOpenGLShader::CompileStrings(std::string shader_name, const char* vertex_shader, const char* fragment_shader) {
    GLint check_value;

    std::stringstream ss;
    ss << shader_name << "Initialize - on entrance";

    if (GLReturnedError(ss.str().c_str()))
        return false;
    // Create our vertex shader, read it from the file and compile it.
    // Check if compilation was successful
    this->vertex_shader_id = glCreateShader(GL_VERTEX_SHADER);
    this->LoadShaderString(vertex_shader, this->vertex_shader_id);
    glCompileShader(this->vertex_shader_id);
    glGetShaderiv(this->vertex_shader_id, GL_COMPILE_STATUS, &check_value);
    if (check_value != GL_TRUE) {
        std::cerr << this->GetShaderLog(vertex_shader_id);
        std::cerr << "GLSL compilation failed - vertex shader: " << shader_name << std::endl;
        return false;
    }

    if (GLReturnedError("ChOpenGLShader::Initialize - after processing vertex shader"))
        return false;
    // Create our fragment shader, read it from the file and compile it.
    // Check if compilation was successful
    this->fragment_shader_id = glCreateShader(GL_FRAGMENT_SHADER);
    this->LoadShaderString(fragment_shader, this->fragment_shader_id);
    glCompileShader(this->fragment_shader_id);
    glGetShaderiv(this->fragment_shader_id, GL_COMPILE_STATUS, &check_value);
    if (check_value != GL_TRUE) {
        std::cerr << this->GetShaderLog(fragment_shader_id);
        std::cerr << "GLSL compilation failed - fragment shader: " << shader_name << std::endl;
        return false;
    }
    return true;
}
bool ChOpenGLShader::CompileFiles(std::string vertex_shader_file, std::string fragment_shader_file) {
    GLint check_value;

    std::stringstream ss;
    ss << vertex_shader_file << "Initialize - on entrance";

    if (GLReturnedError(ss.str().c_str()))
        return false;
    // Create our vertex shader, read it from the file and compile it.
    // Check if compilation was successful
    this->vertex_shader_id = glCreateShader(GL_VERTEX_SHADER);
    this->LoadShader(vertex_shader_file, this->vertex_shader_id);
    glCompileShader(this->vertex_shader_id);
    glGetShaderiv(this->vertex_shader_id, GL_COMPILE_STATUS, &check_value);
    if (check_value != GL_TRUE) {
        std::cerr << this->GetShaderLog(vertex_shader_id);
        std::cerr << "GLSL compilation failed - vertex shader: " << vertex_shader_file << std::endl;
        return false;
    }

    if (GLReturnedError("ChOpenGLShader::Initialize - after processing vertex shader"))
        return false;
    // Create our fragment shader, read it from the file and compile it.
    // Check if compilation was successful
    this->fragment_shader_id = glCreateShader(GL_FRAGMENT_SHADER);
    this->LoadShader(fragment_shader_file, this->fragment_shader_id);
    glCompileShader(this->fragment_shader_id);
    glGetShaderiv(this->fragment_shader_id, GL_COMPILE_STATUS, &check_value);
    if (check_value != GL_TRUE) {
        std::cerr << this->GetShaderLog(fragment_shader_id);
        std::cerr << "GLSL compilation failed - fragment shader: " << fragment_shader_file << std::endl;
        return false;
    }
    return true;
}

void ChOpenGLShader::CompleteInit() {
    // Attach the shaders to the program and link
    this->program_id = glCreateProgram();
    glAttachShader(this->program_id, this->vertex_shader_id);
    glAttachShader(this->program_id, this->fragment_shader_id);
    glLinkProgram(program_id);

    // Individual shaders not needed anymore
    glDeleteShader(vertex_shader_id);
    glDeleteShader(fragment_shader_id);

    glUseProgram(this->program_id);

    CheckGlProgram(this->program_id);
    // Locate all of the possible default handles that we can have
    this->projection_matrix_handle = GetUniformLocation("projection_matrix");
    this->view_matrix_handle = GetUniformLocation("view_matrix");
    this->time_handle = GetUniformLocation("time");
    this->camera_handle = GetUniformLocation("camera_position");
    this->viewport_size_handle = GetUniformLocation("viewport");

    glUseProgram(0);
}

// The shader initialization code is lifted liberally from the GLSL 4.0
// Cookbook.
// Added extra handles as needed
bool ChOpenGLShader::InitializeFiles(std::string vertex_shader_file, std::string fragment_shader_file) {
    if (CompileFiles(vertex_shader_file, fragment_shader_file)) {
        CompleteInit();
    } else {
        std::cerr << "Unable to compile shader: " << vertex_shader_file << " " << fragment_shader_file;
        exit(0);
    }
    return !GLReturnedError("ChOpenGLShader::Initialize - on exit");
}
// The shader initialization code is lifted liberally from the GLSL 4.0
// Cookbook.
// Added extra handles as needed
bool ChOpenGLShader::InitializeStrings(std::string name, const char* vertex_shader, const char* fragment_shader) {
    if (CompileStrings(name, vertex_shader, fragment_shader)) {
        CompleteInit();
    } else {
        std::cerr << "Unable to compile shader: " << name;
        exit(0);
    }

    return !GLReturnedError("ChOpenGLShader::Initialize - on exit");
}
void ChOpenGLShader::CustomSetup() {}

void ChOpenGLShader::TakeDown() {
    // Cleanup the shader

    GLint temp;
    GLsizei size;
    // If already taken down, do not do again
    if (this->program_id == (GLuint)-1)
        return;
    // Get the shader program
    glGetProgramiv(this->program_id, GL_ATTACHED_SHADERS, &temp);
    if (temp > 0) {
        // Get the attached shaders
        GLuint* shader_list = new GLuint[temp];
        glGetAttachedShaders(this->program_id, temp, &size, shader_list);
        for (GLsizei i = 0; i < size; i++) {
            // detach each shader
            glDetachShader(this->program_id, shader_list[i]);
            // The shaders were marked for deletion
            // immediately after they were created.
        }
        delete[] shader_list;
    }
    // finally delete the shader program and then finish
    glDeleteProgram(this->program_id);
    this->program_id = (GLuint)-1;
}

// This function is adapted from OpenGL 4.0 Shading Language Cookbook by David
// Wolff.
bool ChOpenGLShader::LoadShader(const std::string file_name, GLuint shader_id) {
    if (GLReturnedError("ChOpenGLShader::LoadShader - on entrance"))
        return false;

    FILE* file_handle = fopen(file_name.c_str(), "rb");
    if (file_handle == NULL) {
        std::cerr << "Cannot open shader: " << file_name << std::endl;
        return false;
    }
    fseek(file_handle, 0, SEEK_END);
    int length = ftell(file_handle);
    fseek(file_handle, 0, SEEK_SET);
    GLubyte* buffer = new GLubyte[length + 1];
    fread(buffer, sizeof(GLubyte), length, file_handle);
    fclose(file_handle);
    buffer[length] = 0;
    glShaderSource(shader_id, 1, (const char**)&buffer, NULL);
    delete[] buffer;

    return !GLReturnedError("ChOpenGLShader::LoadShader - on exit");
}

// This function is adapted from OpenGL 4.0 Shading Language Cookbook by David
// Wolff.
bool ChOpenGLShader::LoadShaderString(const char* shader_string, GLuint shader_id) {
    if (GLReturnedError("ChOpenGLShader::LoadShader - on entrance"))
        return false;

    glShaderSource(shader_id, 1, &shader_string, NULL);

    return !GLReturnedError("ChOpenGLShader::LoadShader - on exit");
}

// This function is adapted from OpenGL 4.0 Shading Language Cookbook by David
// Wolff.
std::string ChOpenGLShader::GetShaderLog(GLuint shader_id) {
    std::stringstream s;
    GLint log_length;
    glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &log_length);
    if (log_length <= 0)
        s << "No shader log information available." << std::endl;
    else {
        GLchar* buffer = new GLchar[log_length];
        glGetShaderInfoLog(shader_id, log_length, NULL, buffer);
        s << "Shader log:" << std::endl;
        s << buffer << std::endl;
        delete[] buffer;
    }
    return s.str();
}

void ChOpenGLShader::CheckGlProgram(GLuint prog) {
    GLint status;
    glGetProgramiv(prog, GL_LINK_STATUS, &status);
    if (status == GL_FALSE) {
        int loglen;
        char logbuffer[1000];
        glGetProgramInfoLog(prog, sizeof(logbuffer), &loglen, logbuffer);
        fprintf(stderr, "OpenGL Program Linker Error at \n%.*s", loglen, logbuffer);
    } else {
        int loglen;
        char logbuffer[1000];
        glGetProgramInfoLog(prog, sizeof(logbuffer), &loglen, logbuffer);
        if (loglen > 0) {
            fprintf(stderr, "OpenGL Program Link OK at \n%.*s", loglen, logbuffer);
        }
        glValidateProgram(prog);
        glGetProgramInfoLog(prog, sizeof(logbuffer), &loglen, logbuffer);
        if (loglen > 0) {
            fprintf(stderr, "OpenGL Program Validation results at \n%.*s", loglen, logbuffer);
        }
    }
}

GLuint ChOpenGLShader::GetUniformLocation(std::string name) {
    return glGetUniformLocation(program_id, (const GLchar*)name.c_str());
}
}
}
