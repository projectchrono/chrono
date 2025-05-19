// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#pragma once

#include <string>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChParticleCloud.h"
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_fsi/ChApiFsi.h"

#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_vis
/// @{

/// VSG-based run-time visualization system for SCM terrain.
/// Used as a plugin to a Chrono::VSG visualization system.
class CH_VEHICLE_API ChScmVisualizationVSG : public vsg3d::ChVisualSystemVSGPlugin {
  public:
    /// Rendering mode for particles and mesh objects.
    enum class RenderMode { POINTS, WIREFRAME, SOLID };

    ChScmVisualizationVSG(SCMTerrain* scm);
    ~ChScmVisualizationVSG();

    virtual void OnAttach() override;
    virtual void OnInitialize() override;
    virtual void OnBindAssets() override;
    virtual void OnRender() override;

    /// Set the visibility of active boxes with specified tag to the provided value.
    /// A tag value of -1 indicates that the visibility flag should be applied to all boxes.
    void SetActiveBoxVisibility(bool vis, int tag);

    /// Set output directory for saving frame snapshots (default: ".").
    void SetImageOutputDirectory(const std::string& dir) { m_image_dir = dir; }

    /// Enable/disable writing of frame snapshots to file.
    void SetImageOutput(bool val) { m_write_images = val; }

    /// Add additional proxy body to supplemental system.
    /// Must be called before Initialize().
    /// The provided body is set fixed to ground and it is the caller's responsibility to update the position of
    /// this body before a call to Render().
    void AddProxyBody(std::shared_ptr<ChBody> body) {
        body->SetFixed(true);
        m_sys->AddBody(body);
    }

    /// Return the internal Chrono system that holds visualization shapes.
    ChSystem* GetSystem() const { return m_sys; }

  private:
    enum ParticleCloudTag { SPH = 0, BCE_WALL = 1, BCE_RIGID = 2, BCE_FLEX = 3 };

    void BindActiveBox(SCMLoader::ActiveDomainInfo& domain);
    void BindDefaultActiveBox();

    SCMTerrain* m_scm;  ///< associated SCM terrain
    ChSystem* m_sys;    ///< internal Chrono system (holds proxies)

    double m_scm_resolution;                       ///< SCM grid resolution
    bool m_user_active_boxes;                      ///< active domains enabled?
    SCMLoader::ActiveDomainInfo m_default_domain;  ///< default active domain

    SCMTerrain::DataPlotType m_plot_type;  ///< type of SCM false coloring
    std::string m_plot_label;
    double m_plot_min;
    double m_plot_max;

    bool m_active_boxes;                         ///< render active boxes?
    ChColor m_active_box_color;                  ///< color for active boxes
    vsg::ref_ptr<vsg::Switch> m_activeBoxScene;  ///< VSG scene containing FSI body active boxes

    bool m_write_images;      ///< if true, save snapshots
    std::string m_image_dir;  ///< directory for image files

    friend class SCMStatsVSG;
};

/// @} vehicle_vis

}  // namespace vehicle
}  // namespace chrono
