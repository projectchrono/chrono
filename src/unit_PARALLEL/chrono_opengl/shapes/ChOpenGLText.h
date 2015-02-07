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
// Generic renderable text.
// Authors: Hammad Mazhar
// =============================================================================

#ifndef CHOPENGLTEXT_H
#define CHOPENGLTEXT_H

#include "chrono_opengl/core/ChOpenGLObject.h"
#include "chrono_opengl/core/ChOpenGLMaterial.h"
#include <map>

namespace chrono {
namespace opengl {
class CH_OPENGL_API ChOpenGLText : public ChOpenGLObject {
 public:
  ChOpenGLText();
  virtual bool Initialize(ChOpenGLMaterial mat, ChOpenGLShader* shader);
  virtual void Draw(const glm::mat4& projection, const glm::mat4& view);
  void TakeDown();
  void Update();
  void GenerateFontIndex();
  void Render(const std::string& str, float x, float y, float sx, float sy);

 private:
  glm::vec4 color;
  GLuint color_handle, texture_handle;
  std::vector<glm::vec4> text_data;
  std::map<char, int> char_index;
  GLuint vbo, vao;

  GLuint texture, sampler;
  typedef ChOpenGLObject super;
};
}
}
#endif    // END of CHOPENGLTEXT_H