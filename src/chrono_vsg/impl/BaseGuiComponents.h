// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
//
// Implementation of the base Chrono::VSG GUI components.
//
// =============================================================================
// Radu Serban
// =============================================================================

#pragma once

#include <string>

#include "chrono/assets/ChColormap.h"
#include "chrono/core/ChVector2.h"

#include "chrono_vsg/ChGuiComponentVSG.h"

namespace chrono {
namespace vsg3d {

class ChVisualSystemVSG;

class ChBaseGuiComponentVSG : public ChGuiComponentVSG {
  public:
    ChBaseGuiComponentVSG(ChVisualSystemVSG* app);
    virtual void render(vsg::CommandBuffer& cb) override;
    ChVisualSystemVSG* m_app;
};

class ChCameraGuiComponentVSG : public ChGuiComponentVSG {
  public:
    ChCameraGuiComponentVSG(ChVisualSystemVSG* app);
    virtual void render(vsg::CommandBuffer& cb) override;
    ChVisualSystemVSG* m_app;
};

class ChColorbarGuiComponentVSG : public ChGuiComponentVSG {
  public:
    ChColorbarGuiComponentVSG(const std::string& title,
                              const ChVector2d& range,
                              ChColormap::Type type,
                              bool bimodal,
                              float width = 400);
    virtual void Initialize() override;
    virtual void render(vsg::CommandBuffer& cb) override;

  private:
    std::string m_title;
    ChColormap::Type m_type;
    bool m_bimodal;
    vsg::ref_ptr<vsgImGui::Texture> m_texture;
    float m_width;
    ChVector2d m_range;
};

}  // namespace vsg3d
}  // namespace chrono
