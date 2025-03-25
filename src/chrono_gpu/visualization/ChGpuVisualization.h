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

#include "chrono_gpu/ChConfigGpu.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChParticleCloud.h"
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_gpu/ChApiGpu.h"
#include "chrono_gpu/physics/ChSystemGpu.h"

namespace chrono {
namespace gpu {

/// @addtogroup gpu_visualization
/// @{

/// Base class for a run-time visualization system for SPH-based FSI systems.
class CH_GPU_API ChGpuVisualization {
  public:
    virtual ~ChGpuVisualization();

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
    /// Must be called before Initialize().
    virtual void SetCameraMoveScale(float scale);

    /// Attach a user-provided Chrono system for rendering.
    /// By default, the GPU run-time visualization renders granular particles.
    /// This function can be used to also render the mechanical system interacting with the granular system.
    void AttachSystem(ChSystem* system) { m_user_system = system; }

    /// Add additional proxy body to supplemental system.
    /// Must be called before Initialize().
    /// The provided body is set fixed to ground and it is the caller's responsibility to update the position of this
    /// body before a call to Render().
    void AddProxyBody(std::shared_ptr<ChBody> body);

    /// Initialize the run-time visualization system.
    virtual void Initialize();

    /// Render the current state of the Chrono::Gpu system. This function, typically invoked from within the main
    /// simulation loop, can only be called after construction of the Gpu system was completed (i.e., the system was
    /// initialized). This funtion querries the positions of all particles in the Gpu system in order to update the
    /// positions of the proxy bodies.
    /// Returns false if the visualization window was closed.
    virtual bool Render() = 0;

    /// Return the underlying visualization system.
    virtual ChVisualSystem* GetVisualSystem() const = 0;

  protected:
    /// Create a run-time visualization object associated with a given Chrono::Gpu system.
    /// If a supplemental Chrono system is not provided (default), one will be created internally.
    ChGpuVisualization(ChSystemGpu* sysGPU);

    ChSystemGpu* m_systemGPU;  ///< associated Chrono::Gpu system
    ChSystem* m_system;        ///< internal Chrono system (holds proxy bodies)
    ChSystem* m_user_system;   ///< optional user-provided system

    std::shared_ptr<ChParticleCloud> m_particles;  ///< particle cloud proxy for particles
};

/// @} gpu_visualization

}  // namespace gpu
}  // namespace chrono
