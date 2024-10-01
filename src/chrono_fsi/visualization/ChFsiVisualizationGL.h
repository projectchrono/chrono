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

#include "chrono_fsi/visualization/ChFsiVisualization.h"

#include "chrono_opengl/ChVisualSystemOpenGL.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_utils
/// @{

/// Run-time visualization support for Chrono::FSI systems.
/// Requires the Chrono::OpenGL module.
/// Note that using run-time visualization for a Chrono::FSI system incurs the penalty of collecting positions of all
/// particles every time the Render() function is invoked.
class CH_FSI_API ChFsiVisualizationGL : public ChFsiVisualization {
  public:
    /// Create a run-time FSI visualization object associated with a given Chrono::Fsi system.
    ChFsiVisualizationGL(ChSystemFsi* sysFSI);

    ~ChFsiVisualizationGL();

    /// Set title of the visualization window (default: "").
    virtual void SetTitle(const std::string& title) override;

    /// Set window dimensions (default: 1280x720).
    virtual void SetSize(int width, int height) override;

    /// Add a camera initially at the specified position and target (look at) point.
    virtual void AddCamera(const ChVector3d& pos, const ChVector3d& target) override;

    /// Set camera position and target (look at) point.
    virtual void UpdateCamera(const ChVector3d& pos, const ChVector3d& target) override;

    /// Set camera up vector (default: Z).
    virtual void SetCameraVertical(CameraVerticalDir up) override;

    /// Set scale for camera movement increments (default: 0.1).
    virtual void SetCameraMoveScale(float scale) override;

    /// Set rendering mode for SPH particles (default: SOLID).
    /// Must be called before Initialize().
    virtual void SetParticleRenderMode(RenderMode mode) override;

    /// Set rendering mode for mesh objects (default: WIREFRAME).
    virtual void SetRenderMode(RenderMode mode) override;

    /// Enable/disable information overlay (default: false).
    virtual void EnableInfoOverlay(bool val) override;

    /// Initialize the run-time visualization system.
    virtual void Initialize() override;

    /// Render the current state of the Chrono::Fsi system. This function, typically invoked from within the main
    /// simulation loop, can only be called after construction of the FSI system was completed (i.e., the system was
    /// initialized). This function querries the positions of all particles in the FSI system in order to update the
    /// positions of the proxy bodies.
    /// Returns false if the visualization window was closed.
    virtual bool Render() override;

    /// Return the underlying OpenGL visualization system.
    virtual ChVisualSystem* GetVisualSystem() const override { return m_vsys; }

  private:
    opengl::ChVisualSystemOpenGL* m_vsys;  ///< OpenGL visualization system
    unsigned int m_bce_start_index;        ///< start index of BCE proxy bodies in m_system's body list
};

/// @} fsi_utils

}  // namespace fsi
}  // namespace chrono
