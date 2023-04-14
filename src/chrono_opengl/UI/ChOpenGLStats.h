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
#include "chrono_opengl/shapes/ChOpenGLBars.h"

#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace opengl {

/// @addtogroup opengl_module
/// @{

/// Base class for an OpenGL stats overlay
class CH_OPENGL_API ChOpenGLStats : public ChOpenGLBase {
  public:
    virtual ~ChOpenGLStats();

    virtual bool Initialize(ChOpenGLCamera* camera);

    virtual void GenerateStats(ChSystem& sys) = 0;
    virtual void GenerateHelp() {}

    virtual void Render();

    virtual void TakeDown() override;

  protected:
    /// Screen coordinates for text placement
    struct Screen {
        float LEFT;
        float TOP;
        float BOTTOM;
        float RIGHT;
        float CENTER;
        float SPACING;
        float SCALE;
        float SX;
        float SY;
    } screen;

    ChOpenGLStats();

    double time_geometry, time_text, time_total, fps;

    ChOpenGLText text;
    ChOpenGLBars bars;
    ChOpenGLShader font_shader;
    ChOpenGLShader bar_shader;

    ChOpenGLCamera* render_camera;

  private:
    void Update(const glm::ivec2& window_size,
                const double& dpi,
                const double& frame_per_sec,
                const double& t_geometry,
                const double& t_text,
                const double& t_total);

    friend class ChOpenGLViewer;
};

/// Class that renders the text and other UI elements.
class CH_OPENGL_API ChOpenGLStatsDefault : public ChOpenGLStats {
  public:
    ChOpenGLStatsDefault();
    ~ChOpenGLStatsDefault();
    virtual bool Initialize(ChOpenGLCamera* camera) override;
    virtual void GenerateStats(ChSystem& sys) override;
    virtual void GenerateHelp() override;

  private:
    void GenerateCamera();
    void GenerateSystem(ChSystem& sys);
    void GenerateSolver(ChSystem& sys);
    void GenerateCD(ChSystem& sys);
    void GenerateRenderer();

    char buffer[500];
};

/// @} opengl_module

}  // namespace opengl
}  // namespace chrono

#endif
