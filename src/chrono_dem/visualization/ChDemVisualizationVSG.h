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

#include "chrono_dem/ChApiDem.h"
#include "chrono_dem/ChConfigDem.h"
#include "chrono_dem/physics/ChSystemDem.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

namespace chrono {
namespace dem {

/// @addtogroup dem_visualization
/// @{

/// VSG-based run-time visualization system for SPH-based FSI systems.
/// Used as a plugin to a Chrono::VSG visualization system.
/// Note that using run-time visualization for an FSI system incurs the penalty of collecting positions of all
/// particles every time the Render() function is invoked.
class CH_DEM_API ChDemVisualizationVSG : public vsg3d::ChVisualSystemVSGPlugin {
  public:
    ChDemVisualizationVSG(ChSystemDem* sysDEM);

    ~ChDemVisualizationVSG();

    virtual void OnAttach() override;
    virtual void OnInitialize() override;
    virtual void OnBindAssets() override;
    virtual void OnRender() override;

    /// Set default color for granular material (default: [0.10, 0.40, 0.65]).
    void SetDefaultColor(const ChColor& col) { m_default_color = col; }

    /// Get the type of the colormap currently in use.
    ChColormap::Type GetColormapType() const;

    /// Get the colormap object in current use.
    const ChColormap& GetColormap() const;

    /// Class to be used as a callback interface for dynamic coloring of SPH particles.
    class CH_DEM_API ParticleColorCallback : public ChParticleCloud::ColorCallback {
      public:
        virtual std::string GetTile() const = 0;
        virtual ChVector2d GetDataRange() const = 0;
        virtual ChColor GetColor(unsigned int n) const = 0;
        virtual bool IsBimodal() const { return false; }

        ChDemVisualizationVSG* m_vsys;
        ChVector3f* pos;
        ChVector3f* vel;

      private:
        virtual ChColor get(unsigned int n, const ChParticleCloud& cloud) const override final { return GetColor(n); }
    };

    /// Set a callback for dynamic coloring of SPH particles.
    /// If none provided, SPH particles are rendered with a default color.
    void SetColorCallback(std::shared_ptr<ParticleColorCallback> functor,
                          ChColormap::Type type = ChColormap::Type::JET);

    /// Class to be used as a callback interface for dynamic visibility of SPH particles or BCE markers.
    class CH_DEM_API ParticleVisibilityCallback : public ChParticleCloud::VisibilityCallback {
      public:
        virtual bool get(unsigned int n) const = 0;

        ChVector3f* pos;

      private:
        virtual bool get(unsigned int n, const ChParticleCloud& cloud) const override final { return get(n); }
    };

    /// Set a callback for dynamic visibility of SPH particles.
    /// If none provided, all SPH particles are visible.
    void SetVisibilityCallback(std::shared_ptr<ParticleVisibilityCallback> functor) { m_vis_fun = functor; }

    /// Set output directory for saving frame snapshots (default: ".").
    void SetImageOutputDirectory(const std::string& dir) { m_image_dir = dir; }

    /// Enable/disable writing of frame snapshots to file.
    void SetImageOutput(bool val) { m_write_images = val; }

    /// Enable/disable rendering of fluid SPH particles (default: true).
    void EnableParticles(bool val) { m_show_particles = val; }

    /// Add additional proxy body to supplemental system.
    /// Must be called before Initialize().
    /// The provided body is set fixed to ground and it is the caller's responsibility to update the position of
    /// this body before a call to Render().
    void AddProxyBody(std::shared_ptr<ChBody> body) {
        body->SetFixed(true);
        m_sysMBS->AddBody(body);
    }

    /// Return the internal Chrono system that holds visualization shapes.
    ChSystem* GetSystem() const { return m_sysMBS; }

  private:
    ChSystemDem* m_sysDEM;  ///< associated DEM system
    ChSystem* m_sysMBS;     ///< internal Chrono system (holds proxies)

    bool m_show_particles;  ///< render particles?

    std::shared_ptr<ChParticleCloud> m_particle_cloud;  ///< particle cloud proxy for granular material

    ChColor m_default_color;  ///< color for SPH particles

    std::unique_ptr<ChColormap> m_colormap;  ///< colormap for SPH particle false coloring
    ChColormap::Type m_colormap_type;        ///< colormap type

    std::shared_ptr<ParticleColorCallback> m_color_fun;     ///< color functor for particles
    std::shared_ptr<ParticleVisibilityCallback> m_vis_fun;  ///< visibility functor for particles

    // Data for color and visibility functors
    std::vector<ChVector3f> m_pos;  ///< SPH and BCE positions
    std::vector<ChVector3f> m_vel;  ///< SPH and BCE positions

    bool m_write_images;      ///< if true, save snapshots
    std::string m_image_dir;  ///< directory for image files

    friend class DEMStatsVSG;
};

// -----------------------------------------------------------------------------

/// Predefined particle coloring based on particle height.
class CH_DEM_API ParticleHeightColorCallback : public ChDemVisualizationVSG::ParticleColorCallback {
  public:
    ParticleHeightColorCallback(double hmin, double hmax, const ChVector3d& up = ChVector3d(0, 0, 1));

    virtual std::string GetTile() const override;
    virtual ChVector2d GetDataRange() const override;
    virtual ChColor GetColor(unsigned int n) const override;

  private:
    ChColor m_base_color;
    double m_hmin;
    double m_hmax;
    ChVector3d m_up;
};

/// Predefined particle coloring based on particle velocity.
class CH_DEM_API ParticleVelocityColorCallback : public ChDemVisualizationVSG::ParticleColorCallback {
  public:
    enum class Component { X, Y, Z, NORM };

    ParticleVelocityColorCallback(double vmin, double vmax, Component component = Component::NORM);

    virtual std::string GetTile() const override;
    virtual ChVector2d GetDataRange() const override;
    virtual ChColor GetColor(unsigned int n) const override;

  private:
    Component m_component;
    ChColor m_base_color;
    double m_vmin;
    double m_vmax;
};

/// @} dem_visualization

}  // namespace dem
}  // namespace chrono
