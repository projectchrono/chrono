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

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChParticleCloud.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/geometry/ChAABB.h"

#include "chrono_dem/ChApiDem.h"
#include "chrono_dem/ChConfigDem.h"
#include "chrono_dem/physics/ChSystemDem.h"

#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_vsg/utils/ChForceNetworkVSG.h"

namespace chrono {
namespace dem {

/// @addtogroup dem_visualization
/// @{

/// VSG-based run-time visualization system for Chrono::DEM systems.
/// Used as a plugin to a Chrono::VSG visualization system.
/// Note that using run-time visualization for a DEM system incurs the penalty of collecting positions of all
/// particles every time the Render() function is invoked.
///
/// The plugin can also draw the contact force network (see vsg3d::ChForceNetworkVSG): a segment between the particle
/// centers for each particle-particle contact, and a segment from the particle center to the contact point for each
/// contact with a boundary condition (such as the walls of the domain box). The network is colored by the normal force,
/// the Coulomb ratio (tangential force over its Coulomb limit), or the magnitude of the stored tangential displacement
/// (MULTI_STEP friction mode). A force threshold and a region filter (slab and box) select the contacts that are drawn;
/// a slab gives section cuts. The network requires a frictional simulation with contact recording enabled, and it
/// includes the contacts with boundary conditions only with the material-based contact model (see
/// ChSystemDem::GetContacts). Contacts with meshes are not included.
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

    /// Class to be used as a callback interface for dynamic coloring of particles.
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

    /// Set a callback for dynamic coloring of particles.
    /// If none provided, particles are rendered with a default color.
    void SetColorCallback(std::shared_ptr<ParticleColorCallback> functor,
                          ChColormap::Type type = ChColormap::Type::JET);

    /// Class to be used as a callback interface for particle dynamic visibility.
    class CH_DEM_API ParticleVisibilityCallback : public ChParticleCloud::VisibilityCallback {
      public:
        virtual bool get(unsigned int n) const = 0;

        ChVector3f* pos;

      private:
        virtual bool get(unsigned int n, const ChParticleCloud& cloud) const override final { return get(n); }
    };

    /// Set a callback for dynamic visibility of particles.
    /// If none provided, all particles are visible.
    void SetVisibilityCallback(std::shared_ptr<ParticleVisibilityCallback> functor) { m_vis_fun = functor; }

    /// Set output directory for saving frame snapshots (default: ".").
    void SetImageOutputDirectory(const std::string& dir) { m_image_dir = dir; }

    /// Enable/disable writing of frame snapshots to file.
    void SetImageOutput(bool val) { m_write_images = val; }

    /// Enable/disable rendering of particles (default: true).
    void EnableParticles(bool val) { m_show_particles = val; }

    // ---- Force network

    /// Quantity used to color the force network.
    enum class NetworkColoring {
        NORMAL_FORCE,            ///< normal contact force
        COULOMB_RATIO,           ///< tangential contact force over its Coulomb limit
        TANGENTIAL_DISPLACEMENT  ///< magnitude of the stored tangential displacement (MULTI_STEP friction mode)
    };

    /// Show or hide the force network (default: hidden).
    void SetNetworkVisibility(bool visible) { m_network_visible = visible; }

    /// Set the rendering mode of the force network (default: CYLINDERS).
    void SetNetworkMode(vsg3d::ChForceNetworkVSG::Mode mode) { m_network_mode = mode; }

    /// Set the quantity used to color the force network (default: NORMAL_FORCE).
    void SetNetworkColoring(NetworkColoring quantity) { m_coloring = quantity; }

    /// Set the quantity used to color the force network and the range of values mapped to colors.
    /// Without an explicit range, the range is [0, 1] for the Coulomb ratio and, for the other quantities, from 0 to the
    /// largest value at the current frame.
    void SetNetworkColoring(NetworkColoring quantity, double vmin, double vmax);

    /// Map network values to colors with the specified colormap (default: JET).
    void SetNetworkColormap(ChColormap::Type type);

    /// Map network values to colors with a linear ramp between two colors.
    void SetNetworkColorRamp(const ChColor& color_min, const ChColor& color_max);

    /// Set a constant radius for network segments drawn as cylinders (default: 0.2 times the particle radius).
    void SetNetworkRadius(double radius);

    /// Scale the radius of network segments with the normal force, from rmin at fmin to rmax at fmax (clamped).
    void SetNetworkRadius(double rmin, double rmax, double fmin, double fmax);

    /// Set the force threshold of the network (default: 0).
    /// Only contacts with a normal force larger than the threshold are drawn. If relative is true, the threshold is a
    /// multiple of the mean normal force over all contacts with a positive normal force.
    void SetNetworkForceThreshold(double threshold, bool relative = false);

    /// Set the maximum number of network segments (default: 6 times the number of particles, at most 2e6).
    /// Segments beyond this capacity are dropped and counted. This function must be called before the visual system is
    /// initialized.
    void SetNetworkCapacity(unsigned int num_segments) { m_capacity = num_segments; }

    /// Get the number of network segments drawn at the current frame.
    unsigned int GetNumNetworkSegments() const;

    /// Get the number of network segments dropped at the current frame because they exceeded the capacity.
    unsigned int GetNumNetworkSegmentsDropped() const;

    // ---- Region filter

    /// Restrict drawing to a slab of the specified half-thickness, centered at a point and normal to a direction.
    /// A contact is drawn if its contact point is in the slab. With SetRegionParticleFiltering(true), a particle is
    /// drawn if its center is in the slab.
    void SetRegionSlab(const ChVector3d& point, const ChVector3d& normal, double half_thickness);

    /// Restrict drawing to an axis-aligned box.
    /// A contact is drawn if its contact point is in the box. With SetRegionParticleFiltering(true), a particle is drawn
    /// if its center is in the box. A box and a slab can be combined.
    void SetRegionBox(const ChAABB& box);

    /// Remove the slab and box region filters.
    void ClearRegion();

    /// Apply the region filter to particles as well as to the force network (default: false).
    void SetRegionParticleFiltering(bool val) { m_region_particles = val; }

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
    /// Return true if the given point is in the region filter.
    bool InRegion(const ChVector3d& p) const;

    /// Update the force network from the current contact data.
    void UpdateNetwork();

    ChSystemDem* m_sysDEM;  ///< associated DEM system
    ChSystem* m_sysMBS;     ///< internal Chrono system (holds proxies)

    bool m_show_particles;  ///< render particles?

    std::shared_ptr<ChParticleCloud> m_particle_cloud;  ///< particle cloud proxy for granular material

    ChColor m_default_color;  ///< default particle color

    std::unique_ptr<ChColormap> m_colormap;  ///< colormap for particle false coloring
    ChColormap::Type m_colormap_type;        ///< colormap type

    std::shared_ptr<ParticleColorCallback> m_color_fun;     ///< color functor for particles
    std::shared_ptr<ParticleVisibilityCallback> m_vis_fun;  ///< visibility functor for particles

    // Data for color and visibility functors
    std::vector<ChVector3f> m_pos;  ///< particle positions
    std::vector<ChVector3f> m_vel;  ///< particle velocities

    bool m_write_images;      ///< if true, save snapshots
    std::string m_image_dir;  ///< directory for image files

    // Force network
    std::unique_ptr<vsg3d::ChForceNetworkVSG> m_network;  ///< force network renderer
    unsigned int m_capacity;                              ///< maximum number of segments (0: automatic)
    bool m_network_visible;                               ///< draw the force network?
    vsg3d::ChForceNetworkVSG::Mode m_network_mode;        ///< rendering mode
    NetworkColoring m_coloring;                           ///< coloring quantity
    std::array<ChVector2d, 3> m_ranges;                   ///< ranges of the coloring quantities (automatic if empty)
    bool m_use_colormap;                                  ///< map values with the colormap (else with the ramp)
    ChColormap::Type m_network_colormap_type;             ///< colormap of the network
    ChColor m_color_min;                                  ///< ramp color at the low end of the range
    ChColor m_color_max;                                  ///< ramp color at the high end of the range
    double m_radius;                                      ///< constant radius (0: automatic)
    bool m_scale_radius;                                  ///< scale the radius with the normal force?
    double m_rmin, m_rmax;                                ///< radius range
    double m_fmin, m_fmax;                                ///< normal force range for the radius scaling
    double m_threshold;                                   ///< normal force threshold
    bool m_relative_threshold;                            ///< threshold relative to the mean normal force?
    ChVector2d m_current_range;                           ///< range of values mapped to colors at the current frame
    unsigned int m_num_contacts;                          ///< number of contacts at the current frame
    unsigned int m_num_contacts_drawn;                    ///< number of contacts drawn at the current frame
    bool m_contacts_warning;                              ///< warning about unavailable contacts issued?

    // Region filter
    bool m_use_slab;               ///< slab filter active?
    ChVector3d m_slab_point;       ///< point on the slab mid-plane
    ChVector3d m_slab_normal;      ///< slab normal
    double m_slab_half_thickness;  ///< slab half-thickness
    bool m_use_box;                ///< box filter active?
    ChAABB m_box;                  ///< box
    bool m_region_particles;       ///< filter the particles too?

    // Work arrays for the force network
    std::vector<ChSystemDem::Contact> m_contacts;
    std::vector<float> m_points1;
    std::vector<float> m_points2;
    std::vector<float> m_values;
    std::vector<float> m_radii;

    friend class DEMStatsVSG;
    friend class DEMParticleVisibility;
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
