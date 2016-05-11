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
// Class that renders the text and other UI elements
// =============================================================================

#ifndef CHOPENGLHUD_H
#define CHOPENGLHUD_H

#include "chrono_opengl/core/ChOpenGLBase.h"
#include "chrono_opengl/core/ChOpenGLMaterial.h"
#include "chrono_opengl/shapes/ChOpenGLText.h"
#include "chrono_opengl/ChOpenGLCamera.h"

#include "physics/ChSystem.h"
#include "core/ChTimer.h"
namespace chrono {
namespace opengl {

/// @addtogroup opengl
/// @{

/// Class that renders the text and other UI elements.
class CH_OPENGL_API ChOpenGLHUD : public ChOpenGLBase {
  public:
    ChOpenGLHUD();
    bool Initialize(ChOpenGLCamera* camera);
    void GenerateHelp();
    void GenerateCamera();
    void GenerateSystem(ChSystem* physics_system);
    void GenerateSolver(ChSystem* physics_system);
    void GenerateCD(ChSystem* physics_system);
    void GenerateRenderer();
    void GenerateStats(ChSystem* physics_system);
    void GenerateExtraStats(ChSystem* physics_system);
    void TakeDown();
    void Update(const glm::ivec2& window_size,
                const float& dpi,
                const float& frame_per_sec,
                const float& t_geometry,
                const float& t_text,
                const float& t_total);
    void Draw();

  private:
    ChOpenGLText text;
    ChOpenGLShader font_shader;
    float sx, sy;
    float aspect, z_x, z_y;
    char buffer[50];
    ChOpenGLCamera* render_camera;

    float time_geometry, time_text, time_total, fps;

    ChTimer<double> timer_text, timer_render, timer_geometry;
};

/// @} opengl

}
}
#endif  // END of CHOPENGLHUD_H
