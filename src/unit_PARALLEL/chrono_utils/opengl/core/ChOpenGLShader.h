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
// Shader() is a sample shader class that loads and compiles the vertex and
// fragment shaders. Based on code provided by Perry Kivolowitz.
// This Shader() class implements or assumes a basic set of uniforms will be
// provided to all shaders derived from it. These are listed below.
// ChOpenGLShader::CommonSetup() can be used by call derived classes to send the
// common values to the shader.
// Authors: Hammad Mazhar
// =============================================================================

#ifndef CHOPENGLSHADER_H
#define CHOPENGLSHADER_H

#include "chrono_utils/opengl/core/ChOpenGLBase.h"

namespace chrono {
namespace utils {

class CH_UTILS_OPENGL_API ChOpenGLShader : public ChOpenGLBase {
 public:
   ChOpenGLShader();
   void TakeDown();
   void Use();
   bool CompileStrings(
         std::string shader_name,
         const char * vertex_shader,
         const char * fragment_shader);
   bool CompileFiles(
         std::string vertex_shader_file,
         std::string fragment_shader_file);
   void CompleteInit();

   virtual bool InitializeFiles(
         std::string vertex_shader_file,
         std::string fragment_shader_file);
   virtual bool InitializeStrings(
         std::string shader_name,
         const char * vertex_shader_file,
         const char * fragment_shader_file);
   virtual void CustomSetup();
   void CommonSetup(
         const GLfloat * projection,
         const GLfloat * view);
   void SetTime(
         float _time);
   void SetCamera(
         glm::vec3 campos);
   GLuint GetUniformLocation(
         std::string name);
   GLuint view_matrix_handle;
   GLuint projection_matrix_handle;
   GLuint time_handle, camera_handle;
   GLuint vertex_shader_id;
   GLuint fragment_shader_id;
   GLuint program_id;
   bool LoadShader(
         const std::string file_name,
         GLuint shader_id);
   bool LoadShaderString(
         const char * shader_string,
         GLuint shader_id);
   std::string GetShaderLog(
         GLuint shader_id);
   void CheckGlProgram(
         GLuint prog);
   float time;
   glm::vec3 camera_pos;
};

}
}
#endif  // END of CHOPENGLSHADER_H
