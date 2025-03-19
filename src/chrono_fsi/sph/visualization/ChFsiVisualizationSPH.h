// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
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
#include "chrono_fsi/sph/ChFsiSystemSPH.h"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_visualization
/// @{

/// Base class for a run-time visualization system for SPH-based FSI systems.
/// Note that using run-time visualization for an FSI system incurs the penalty of collecting positions of all
/// particles every time the Render() function is invoked.
class CH_FSI_API ChFsiVisualizationSPH {
  public:
    /// Rendering mode for particles and mesh objects.
    enum class RenderMode { POINTS, WIREFRAME, SOLID };

    virtual ~ChFsiVisualizationSPH();

    /// Enable/disable information terminal output during initialization (default: false).
    void SetVerbose(bool verbose);

    /// Set title of the visualization window (default: "").
    virtual void SetTitle(const std::string& title);

    /// Set window dimensions (default: 1280x720).
    virtual void SetSize(int width, int height);

    /// Add a camera initially at the specified position and target (look at) point.
    virtual void AddCamera(const ChVector3d& pos, const ChVector3d& target);

    /// Set camera position and target (look at) point.
    virtual void UpdateCamera(const ChVector3d& pos, const ChVector3d& target);

    /// Set camera up vector (default: Z).
    virtual void SetCameraVertical(CameraVerticalDir up);

    /// Set scale for camera movement increments (default: 0.1).
    virtual void SetCameraMoveScale(float scale);

    /// Set directional light intensity (default: 1).
    /// The light intensity must be in [0,1].
    virtual void SetLightIntensity(double intensity);

    /// Set azimuth and elevation of directional light in radians (default: 0, 0).
    /// The azimuth must be in [-pi,+pi], with 0 corresponding to the positive x direction.
    /// The elevation must be in [0, pi/2] with 0 corresponding to the horizontal.
    virtual void SetLightDirection(double azimuth, double elevation);

    /// Set rendering mode for SPH particles.
    /// Must be called before Initialize().
    virtual void SetParticleRenderMode(RenderMode mode);

    /// Set rendering mode for mesh objects (default: WIREFRAME).
    virtual void SetRenderMode(RenderMode mode);

    /// Set default color for fluid SPH particles (default: [0.10, 0.40, 0.65]).
    void SetColorFluidMarkers(const ChColor& col) { m_sph_color = col; }

    /// Set default color for boundary BCE markers (default: [0.65, 0.30, 0.03]).
    void SetColorBoundaryMarkers(const ChColor& col) { m_bndry_bce_color = col; }

    /// Set default color for rigid body BCE markers (default: [0.10, 0.60, 0.30]).
    void SetColorRigidBodyMarkers(const ChColor& col) { m_rigid_bce_color = col; }

    /// Set default color for flex body BCE markers (default: [0.40, 0.10, 0.65]).
    void SetColorFlexBodyMarkers(const ChColor& col) { m_flex_bce_color = col; }

    /// Class to be used as a callback interface for dynamic coloring of SPH particles.
    class CH_FSI_API ParticleColorCallback : public ChParticleCloud::ColorCallback {
      public:
        virtual ChColor get(unsigned int n) const = 0;

        Real3* pos;
        Real3* vel;
        Real3* prop;

      private:
        virtual ChColor get(unsigned int n, const ChParticleCloud& cloud) const override final { return get(n); }
    };

    /// Set a callback for dynamic coloring of SPH particles.
    /// If none provided, SPH particles are rendered with a default color.
    void SetSPHColorCallback(std::shared_ptr<ParticleColorCallback> functor) { m_color_fun = functor; }

    /// Class to be used as a callback interface for dynamic visibility of SPH particles or BCE markers.
    class CH_FSI_API MarkerVisibilityCallback : public ChParticleCloud::VisibilityCallback {
      public:
        virtual bool get(unsigned int n) const = 0;

        Real3* pos;

      private:
        virtual bool get(unsigned int n, const ChParticleCloud& cloud) const override final { return get(n); }
    };

    /// Set a callback for dynamic visibility of SPH particles.
    /// If none provided, all SPH particles are visible.
    void SetSPHVisibilityCallback(std::shared_ptr<MarkerVisibilityCallback> functor) { m_vis_sph_fun = functor; }

    /// Set a callback for dynamic visibility of boundary BCE markers.
    /// If none provided, all boundary BCE markers are visible.
    void SetBCEVisibilityCallback(std::shared_ptr<MarkerVisibilityCallback> functor) { m_vis_bndry_fun = functor; }

    /// Set output directory for saving frame snapshots (default: ".").
    void SetImageOutputDirectory(const std::string& dir) { m_image_dir = dir; }

    /// Enable/disable writing of frame snapshots to file.
    void SetImageOutput(bool val) { m_write_images = val; }

    /// Enable/disable information overlay (default: true).
    virtual void EnableInfoOverlay(bool val);

    /// Enable/disable rendering of fluid SPH particles (default: true).
    void EnableFluidMarkers(bool val) { m_sph_markers = val; }

    /// Enable/disable rendering of rigid-body BCE markers (default: true).
    void EnableRigidBodyMarkers(bool val) { m_rigid_bce_markers = val; }

    /// Enable/disable rendering of flex-body BCE markers (default: true).
    void EnableFlexBodyMarkers(bool val) { m_flex_bce_markers = val; }

    /// Enable/disable rendering of boundary BCE markers (default: false).
    void EnableBoundaryMarkers(bool val) { m_bndry_bce_markers = val; }

    /// Attach a user-provided Chrono system for rendering.
    /// By default, the FSI run-time visualization renders SPH particles and BCE markers.
    /// This function can be used to also render the mechanical system interacting with the SPH system.
    void AttachSystem(ChSystem* system) { m_user_system = system; }

    /// Add additional proxy body to supplemental system.
    /// Must be called before Initialize().
    /// The provided body is set fixed to ground and it is the caller's responsibility to update the position of this
    /// body before a call to Render().
    void AddProxyBody(std::shared_ptr<ChBody> body);

    /// Initialize the run-time visualization system.
    virtual void Initialize();

    /// Render the current state of the Chrono::Fsi system. This function, typically invoked from within the main
    /// simulation loop, can only be called after construction of the FSI system was completed (i.e., the system was
    /// initialized). This function querries the positions of all particles in the FSI system in order to update the
    /// positions of the proxy bodies.
    /// Returns false if the visualization window was closed.
    virtual bool Render() = 0;

    /// Return the internal Chrono system that holds visualization shapes.
    ChSystem* GetSystem() const { return m_sysMBS; }

    /// Return the underlying visualization system.
    virtual ChVisualSystem* GetVisualSystem() const = 0;

  protected:
    /// Create a run-time FSI visualization object associated with a given Chrono::Fsi system.
    ChFsiVisualizationSPH(ChFsiSystemSPH* sysFSI);

    /// Create a run-time FSI visualization object associated with a given SPH fluid system.
    ChFsiVisualizationSPH(ChFsiFluidSystemSPH* sysSPH);

    ChFsiSystemSPH* m_sysFSI;    ///< associated FSI system
    ChFsiFluidSystemSPH* m_sysSPH;  ///< associated SPH system
    ChSystem* m_sysMBS;          ///< internal Chrono system (holds proxies)
    ChSystem* m_user_system;     ///< optional user-provided system

    bool m_sph_markers;        ///< render fluid SPH particles?
    bool m_bndry_bce_markers;  ///< render boundary BCE markers?
    bool m_rigid_bce_markers;  ///< render rigid-body BCE markers?
    bool m_flex_bce_markers;   ///< render flex-body markers?

    std::shared_ptr<ChParticleCloud> m_sph_cloud;        ///< particle cloud proxy for SPH particles
    std::shared_ptr<ChParticleCloud> m_bndry_bce_cloud;  ///< particle cloud proxy for boundary BCE markers
    std::shared_ptr<ChParticleCloud> m_rigid_bce_cloud;  ///< particle cloud proxy for BCE markers on rigid bodies
    std::shared_ptr<ChParticleCloud> m_flex_bce_cloud;   ///< particle cloud proxy for BCE markers on flex bodies

    ChColor m_sph_color;        ///< color for SPH particles
    ChColor m_bndry_bce_color;  ///< color for boundary BCE markers
    ChColor m_rigid_bce_color;  ///< color for BCE markers on rigid bodies
    ChColor m_flex_bce_color;   ///< color for BCE markers on flex bodies

    std::shared_ptr<ParticleColorCallback> m_color_fun;         ///< color functor for SPH particles
    std::shared_ptr<MarkerVisibilityCallback> m_vis_sph_fun;    ///< visibility functor for SPH particles
    std::shared_ptr<MarkerVisibilityCallback> m_vis_bndry_fun;  ///< visibility functor for bndry BCE markers

    bool m_write_images;      ///< if true, save snapshots
    std::string m_image_dir;  ///< directory for image files
};

// -----------------------------------------------------------------------------

/// Predefined SPH coloring based on particle height.
class CH_FSI_API ParticleHeightColorCallback : public ChFsiVisualizationSPH::ParticleColorCallback {
  public:
    ParticleHeightColorCallback(double hmin, double hmax, const ChVector3d& up = ChVector3d(0, 0, 1));
    ParticleHeightColorCallback(const ChColor& base_color,
                                double hmin,
                                double hmax,
                                const ChVector3d& up = ChVector3d(0, 0, 1));

    virtual ChColor get(unsigned int n) const override;

  private:
    bool m_monochrome;
    ChColor m_base_color;
    double m_hmin;
    double m_hmax;
    Real3 m_up;
};

/// Predefined SPH coloring based on particle velocity.
class CH_FSI_API ParticleVelocityColorCallback : public ChFsiVisualizationSPH::ParticleColorCallback {
  public:
    enum class Component { X, Y, Z, NORM };

    ParticleVelocityColorCallback(double vmin, double vmax, Component component = Component::NORM);
    ParticleVelocityColorCallback(const ChColor& base_color,
                                  double vmin,
                                  double vmax,
                                  Component component = Component::NORM);

    virtual ChColor get(unsigned int n) const override;

  private:
    Component m_component;
    bool m_monochrome;
    ChColor m_base_color;
    double m_vmin;
    double m_vmax;
};

/// Predefined SPH coloring based on particle density.
class CH_FSI_API ParticleDensityColorCallback : public ChFsiVisualizationSPH::ParticleColorCallback {
  public:
    ParticleDensityColorCallback(double dmin, double dmax);
    ParticleDensityColorCallback(const ChColor& base_color, double dmin, double dmax);

    virtual ChColor get(unsigned int n) const override;

  private:
    bool m_monochrome;
    ChColor m_base_color;
    double m_dmin;
    double m_dmax;
};

/// Predefined SPH coloring based on particle pressure.
class CH_FSI_API ParticlePressureColorCallback : public ChFsiVisualizationSPH::ParticleColorCallback {
  public:
    ParticlePressureColorCallback(double pmin, double pmax);
    ParticlePressureColorCallback(const ChColor& base_color, double pmin, double pmax);
    ParticlePressureColorCallback(const ChColor& base_color_neg,
                                  const ChColor& base_color_pos,
                                  double pmin,
                                  double pmax);

    virtual ChColor get(unsigned int n) const override;

  private:
    bool m_monochrome;
    bool m_bichrome;
    ChColor m_base_color;
    ChColor m_base_color_pos;
    ChColor m_base_color_neg;
    double m_pmin;
    double m_pmax;
};

/// @} fsisph_visualization

}  // namespace sph
}  // namespace fsi
}  // namespace chrono
