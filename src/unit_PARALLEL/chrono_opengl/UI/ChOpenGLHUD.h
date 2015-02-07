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
// Class that renders the text and other UI elements
// Authors: Hammad Mazhar
// =============================================================================

#ifndef CHOPENGLHUD_H
#define CHOPENGLHUD_H

#include "chrono_opengl/core/ChOpenGLBase.h"
#include "chrono_opengl/core/ChOpenGLMaterial.h"
#include "chrono_opengl/shapes/ChOpenGLText.h"

namespace chrono {
namespace opengl {
class CH_OPENGL_API ChOpenGLHUD : public ChOpenGLBase {
 public:
  ChOpenGLHUD();
  bool Initialize();
  void DrawHelp();
  void DrawStats();
  void TakeDown();
  void Update(const glm::ivec2& window_size, const float & dpi);

 private:
  ChOpenGLText help_text;
  ChOpenGLShader font_shader;
  float sx, sy, spacing;

};
}
}
#endif    // END of CHOPENGLHUD_H