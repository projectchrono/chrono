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

#include <iostream>
#include "chrono_opengl/shapes/ChOpenGLText.h"
#include <glm/gtc/type_ptr.hpp>
#include "chrono_opengl/FontData.h"

// Includes that are generated at compile time
#include "resources/text_frag.h"
#include "resources/text_vert.h"

using namespace glm;
using namespace chrono::opengl;

ChOpenGLHUD::ChOpenGLHUD() : ChOpenGLBase() { spacing = 0.055; }

bool ChOpenGLHUD::Initialize() {
  if (this->GLReturnedError("ChOpenGLHUD::Initialize - on entry"))
    return false;

  if (!font_shader.InitializeStrings("text", text_vert, text_frag)) {
    return 0;
  }

  return true;
}
void ChOpenGLHUD::Update(const glm::ivec2& window_size, const float& dpi) {

  sx = (2 * dpi / 147.782) / window_size.x;
  sy = (2 * dpi / 147.782) / window_size.y;
}

void ChOpenGLHUD::TakeDown() {

  font_shader.TakeDown();
  help_text.TakeDown();
}

void ChOpenGLHUD::DrawHelp() {
  help_text.Update();

  help_text.Render("Press h to exit help", -.95, 0.925 - spacing * 0, sx, sy);
  help_text.Render("W: Forward", -.95, 0.925 - spacing * 1, sx, sy);
  help_text.Render("A: Strafe Left", -.95, 0.925 - spacing * 2, sx, sy);
  help_text.Render("S: Back", -.95, 0.925 - spacing * 3, sx, sy);
  help_text.Render("D: Strafe Right", -.95, 0.925 - spacing * 4, sx, sy);
  help_text.Render("Q: Down", -.95, 0.925 - spacing * 5, sx, sy);
  help_text.Render("E: Up", -.95, 0.925 - spacing * 6, sx, sy);

  help_text.Render("Mouse Look (Click and hold left mouse button)", -.95, 0.925 - spacing * 7, sx, sy);

  help_text.Render("1: Point Cloud (default)", -.95, 0.925 - spacing * 9, sx, sy);
  help_text.Render("2: Wireframe (slow)", -.95, 0.925 - spacing * 10, sx, sy);
  help_text.Render("3: Solid", -.95, 0.925 - spacing * 11, sx, sy);

  help_text.Render("C: Show/Hide Contacts (DVI only)", -.95, 0.925 - spacing * 13, sx, sy);

  help_text.Render("Space: Pause Simulation (not rendering)", -.95, 0.925 - spacing * 15, sx, sy);
  help_text.Render("P: Pause Rendering (not simulating)", -.95, 0.925 - spacing * 16, sx, sy);
  help_text.Render(".: Single Step ", -.95, 0.925 - spacing * 18, sx, sy);
  help_text.Render("B: Enable/Disable AABB ", -.95, 0.925 - spacing * 20, sx, sy);

  help_text.Render("Escape: Exit ", -.95, 0.925 - spacing * 30, sx, sy);


}
void ChOpenGLHUD::DrawStats() {}