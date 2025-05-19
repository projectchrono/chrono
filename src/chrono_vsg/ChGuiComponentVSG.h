// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Radu Serban
// =============================================================================

#ifndef CH_GUI_COMPONENT_VSG_H
#define CH_GUI_COMPONENT_VSG_H

#include <vsg/all.h>
#include <vsgImGui/imgui.h>
#include <vsgImGui/Texture.h>

#include "chrono/core/ChVector2.h"

#include "chrono_vsg/ChApiVSG.h"

namespace chrono {
namespace vsg3d {

class ChVisualSystemVSG;

/// @addtogroup vsg_module
/// @{

/// Base class for a GUI component for the VSG run-time visualization system.
class CH_VSG_API ChGuiComponentVSG {
  public:
    ChGuiComponentVSG();
    virtual ~ChGuiComponentVSG() {}

    /// Allow the GUI component to initialize itself.
    /// This function is called after the main GUI container is created.
    virtual void Initialize() {}

    /// Specify the ImGui elements to be rendered for this GUI component.
    virtual void render(vsg::CommandBuffer& cb) = 0;

    /// Set visibility for this GUI component.
    void SetVisibility(bool visible) { m_visible = visible; }

    /// Toggle GUI visibility for this GUI component.
    void ToggleVisibility() { m_visible = !m_visible; }

    /// Return boolean indicating whether or not this GUI component visible.
    bool IsVisible() const { return m_visible; }

    /// Utility function to draw a gauge.
    static void DrawGauge(float val, float v_min, float v_max);

    /// Utility function to draw a colorbar (colormap legend).
    static void Colorbar(vsg::ref_ptr<vsgImGui::Texture> texture,
                         const ChVector2d& range,
                         bool bimodal,
                         float width,
                         uint32_t deviceID);

  protected:
    bool m_visible;
    ChVisualSystemVSG* m_vsys;

    friend class ChVisualSystemVSG;
};

/// @} vsg_module

}  // namespace vsg3d
}  // namespace chrono

#endif
