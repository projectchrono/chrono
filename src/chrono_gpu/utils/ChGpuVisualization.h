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

#include "chrono_gpu/ChApiGpu.h"
#include "chrono_gpu/physics/ChSystemGpu.h"

namespace chrono {
namespace gpu {

/// @addtogroup gpu_utils
/// @{

/// Run-time visualization support for Chrono::Gpu systems.
/// Requires the Chrono::OpenGL module; if not available, most ChGpuVisualization functions are no-op.
///
/// Visualization is based on a separate Chrono system which carries proxies for all particles in the Chrono::Gpu
/// system. This separate system can be provided by the user or else created automatically. Note that using run-time
/// visualization for a Chrono::Gpu system incurs the penalty of collecting positions of all particles every time the
/// Render() function is invoked.
///
/// To implement a moving camera (i.e., prescribe the camera position), get the current instance of the active
/// Chrono::OpenGL window and use the function ChOpenGLWindow::SetCamera().
class CH_GPU_API ChGpuVisualization {
  public:
    /// <summary>
    /// Create a run-time visualization object associated with a given Chrono::Gpu system.
    /// If a supplemental Chrono system is not provided (default), one will be created internally.
    /// </summary>
    /// <param name="sysGPU">Associated Chrono::Gpu system</param>
    /// <param name="sys">Supplemental Chrono system containing visualization proxies</param>
    ChGpuVisualization(ChSystemGpu* sysGPU);

    ~ChGpuVisualization();

    /// Set title of the visualization window (default: "").
    void SetTitle(const std::string& title) { m_title = title; }

    /// Set camera position and target (look at) point.
    /// Must be called before Initialize().
    void SetCameraPosition(const ChVector<>& pos, const ChVector<>& target);

    /// Set camera up vector (default: Z).
    /// Must be called before Initialize().
    void SetCameraUpVector(const ChVector<>& up);

    /// Set scale for camera movement increments (default: 0.1).
    /// Must be called before Initialize().
    void SetCameraMoveScale(float scale);

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
    /// If the Chrono::OpenGL module is not available, this function is no-op.
    void Initialize();

    /// Render the current state of the Chrono::Gpu system. This function, typically invoked from within the main
    /// simulation loop, can only be called after construction of the Gpu system was completed (i.e., the system was
    /// initialized). This funtion querries the positions of all particles in the Gpu system in order to update the
    /// positions of the proxy bodies.
    /// Returns false if the visualization window was closed.
    /// If the Chrono::OpenGL module is not available, this function is no-op.
    bool Render();

  private:
    ChSystemGpu* m_systemGPU;  ///< associated Chrono::Gpu system
    ChSystem* m_system;        ///< supplemental Chrono system (holds proxy bodies)
    ChSystem* m_user_system;   ///< optional user-provided system

    unsigned int m_part_start_index;  ///< start index of particles in m_system's body list

    std::string m_title;      ///< visualization window title
    ChVector<> m_cam_pos;     ///< current camera position
    ChVector<> m_cam_target;  ///< current camera look at point
    ChVector<> m_cam_up;      ///< camera up vector
    float m_cam_scale;        ///< camera move increment scale
};

/// @} gpu_utils

}  // namespace gpu
}  // namespace chrono
