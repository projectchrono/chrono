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

#include <vsgImGui/imgui.h>

#include "chrono_vsg/ChApiVSG.h"

namespace chrono {
namespace vsg3d {

/// @addtogroup vsg_module
/// @{

/// Base class for a GUI component for the VSG run-time visualization system.
class CH_VSG_API ChGuiComponentVSG {
  public:
    ChGuiComponentVSG() : m_visible(true) {}
    virtual ~ChGuiComponentVSG() {}

    /// Specify the ImGui elements to be rendered for this GUI component.
    virtual void render() = 0;

    /// Set visibility for this GUI component.
    void SetVisibility(bool visible) { m_visible = visible; }

    /// Toggle GUI visibility for this GUI component.
    void ToggleVisibility() { m_visible = !m_visible; }

    /// Return boolean indicating whether or not this GUI component visible.
    bool IsVisible() const { return m_visible; }

    /// Utility function to draw a gauge.
    static void DrawGauge(float val, float v_min, float v_max) {
        ImGui::PushItemWidth(150.0f);
        ImGui::PushStyleColor(ImGuiCol_SliderGrab, (ImVec4)ImColor(200, 100, 20));
        ImGui::SliderFloat("", &val, v_min, v_max, "%.2f");
        ImGui::PopStyleColor();
        ImGui::PopItemWidth();
    }

  protected:
    bool m_visible;
};

/// @} vsg_module

}  // namespace vsg3d
}  // namespace chrono

#endif
