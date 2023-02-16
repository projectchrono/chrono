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

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChParticleCloud.h"
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChSystemFsi.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_utils
/// @{

/// Run-time visualization support for Chrono::FSI systems.
/// Note that using run-time visualization for a Chrono::FSI system incurs the penalty of collecting positions of all
/// particles every time the Render() function is invoked.
class CH_FSI_API ChFsiVisualization {
  public:
    /// Rendering mode for particles and mesh objects.
    enum class RenderMode {POINTS, WIREFRAME, SOLID};

    /// Create a run-time FSI visualization object associated with a given Chrono::Fsi system.
    ChFsiVisualization(ChSystemFsi* sysFSI);

    virtual ~ChFsiVisualization();

    /// Set title of the visualization window (default: "").
    virtual void SetTitle(const std::string& title);

    /// Set window dimensions (default: 1280x720).
    virtual void SetSize(int width, int height);

    /// Add a camera initially at the specified position and target (look at) point.
    virtual void AddCamera(const ChVector<>& pos, const ChVector<>& target);

    /// Set camera position and target (look at) point.
    virtual void UpdateCamera(const ChVector<>& pos, const ChVector<>& target);

    /// Set camera up vector (default: Z).
    virtual void SetCameraVertical(CameraVerticalDir up);

    /// Set scale for camera movement increments (default: 0.1).
    virtual void SetCameraMoveScale(float scale);

    /// Set rendering mode for SPH particles.
    /// Must be called before Initialize().
    virtual void SetParticleRenderMode(RenderMode mode);

    /// Set rendering mode for mesh objects (default: WIREFRAME).
    virtual void SetRenderMode(RenderMode mode);

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

  protected:
    ChSystemFsi* m_systemFSI;  ///< associated Chrono::FSI system
    ChSystem* m_system;        ///< internal Chrono system (holds proxy bodies)
    ChSystem* m_user_system;   ///< optional user-provided system

    bool m_sph_markers;        ///< render fluid SPH particles?
    bool m_bndry_bce_markers;  ///< render boundary BCE markers?
    bool m_rigid_bce_markers;  ///< render rigid-body BCE markers?
    bool m_flex_bce_markers;   ///< render flex-body markers?

    std::shared_ptr<ChParticleCloud> m_sph_cloud;        ///< particle cloud proxy for SPH particles
    std::shared_ptr<ChParticleCloud> m_bndry_bce_cloud;  ///< particle cloud proxy for boundary BCE markers
    std::shared_ptr<ChParticleCloud> m_rigid_bce_cloud;  ///< particle cloud proxy for BCE markers on rigid bodies
    std::shared_ptr<ChParticleCloud> m_flex_bce_cloud;   ///< particle cloud proxy for BCE markers on flex bodies
};

/// @} fsi_utils

}  // namespace fsi
}  // namespace chrono
