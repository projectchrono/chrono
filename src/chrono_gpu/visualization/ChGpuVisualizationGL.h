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

#include "chrono_gpu/visualization/ChGpuVisualization.h"

#include "chrono_opengl/ChVisualSystemOpenGL.h"

namespace chrono {
namespace gpu {

/// @addtogroup gpu_visualization
/// @{

/// OpenGL-based run-time visualization system for GPU systems.
/// Requires the Chrono::OpenGL module.
///
/// Visualization is based on a separate Chrono system which carries proxies for all particles in the Chrono::Gpu
/// system. This separate system can be provided by the user or else created automatically. Note that using run-time
/// visualization for a Chrono::Gpu system incurs the penalty of collecting positions of all particles every time the
/// Render() function is invoked.
class CH_GPU_API ChGpuVisualizationGL : public ChGpuVisualization {
  public:
    /// Create a run-time visualization object associated with a given Chrono::Gpu system.
    ChGpuVisualizationGL(ChSystemGpu* sysGPU);

    ~ChGpuVisualizationGL();

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
    /// Must be called before Initialize().
    virtual void SetCameraMoveScale(float scale) override;

    /// Initialize the run-time visualization system.
    /// If the Chrono::OpenGL module is not available, this function is no-op.
    virtual void Initialize() override;

    /// Render the current state of the Chrono::Gpu system. This function, typically invoked from within the main
    /// simulation loop, can only be called after construction of the Gpu system was completed (i.e., the system was
    /// initialized). This funtion querries the positions of all particles in the Gpu system in order to update the
    /// positions of the proxy bodies.
    /// Returns false if the visualization window was closed.
    /// If the Chrono::OpenGL module is not available, this function is no-op.
    virtual bool Render() override;

    /// Return the underlying OpenGL visualization system.
    virtual ChVisualSystem* GetVisualSystem() const override { return m_vsys; }

  private:
    opengl::ChVisualSystemOpenGL* m_vsys;  ///< OpenGL visualization system
};

/// @} gpu_visualization

}  // namespace gpu
}  // namespace chrono
