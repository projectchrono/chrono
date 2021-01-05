// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Contributors: Dan Negrut, Nic Olsen, Radu Serban
// =============================================================================

#pragma once

#include <vector>

#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_gpu/physics/ChSystemGpu_impl.h"
#include "chrono_gpu/physics/ChSystemGpuMesh_impl.h"

namespace chrono {
namespace gpu {

/// @addtogroup gpu_physics
/// @{

// -----------------------------------------------------------------------------

/// Interface to a Chrono::Gpu mesh system.
class CH_GPU_API ChSystemGpuMesh {
  public:
    enum class MeshVerbosity { QUIET = 0, INFO = 1 };

    /// Construct system with given sphere radius, density, and big domain dimensions.
    ChSystemGpuMesh(float sphere_rad, float density, float3 boxDims);
    ~ChSystemGpuMesh();

    /// Load triangle meshes into granular system. MUST happen before initialize is called.
    void LoadMeshes(std::vector<std::string> objfilenames,
                     std::vector<ChMatrix33<float>> rotscale,
                     std::vector<float3> translations,
                     std::vector<float> masses);

    ChSystemGpuMesh_impl& getSystem() { return *m_sys_trimesh; }

    /// Set particle positions.
    void SetParticlePositions(
        const std::vector<ChVector<float>>& points,
        const std::vector<ChVector<float>>& vels = std::vector<ChVector<float>>(),
        const std::vector<ChVector<float>>& ang_vels = std::vector<ChVector<float>>());

    /// Return particle position.
    ChVector<float> GetParticlePosition(int nSphere) const;

    /// Return particle angular velocity.
    ChVector<float> GetParticleAngVelocity(int nSphere) const;

    /// Return particle velocity.
    ChVector<float> GetParticleVelocity(int nSphere) const;

    /// Set simulation verbose level.
    void SetVerbosity(MeshVerbosity level) { mesh_verbosity = level; }

  private:
    MeshVerbosity mesh_verbosity; ///< verbose level

    /// Setup data structures associated with triangle mesh.
    void SetMeshes(const std::vector<geometry::ChTriangleMeshConnected>& all_meshes, std::vector<float> masses);

    /// Clean copy of mesh soup interacting with granular material in unified memory. Stored in UU
    ChSystemGpuMesh_impl* m_sys_trimesh;
};

// -----------------------------------------------------------------------------

/// Interface to a Chrono::Gpu system.
class CH_GPU_API ChSystemGpu {
  public:
    /// Construct system with given sphere radius, density, and big domain dimensions.
    ChSystemGpu(float sphere_rad, float density, float3 boxDims);
    ~ChSystemGpu();

    ChSystemGpu_impl& getSystem() { return *m_sys; }

    /// Set particle positions.
    void SetParticlePositions(
        const std::vector<ChVector<float>>& points,
        const std::vector<ChVector<float>>& vels = std::vector<ChVector<float>>(),
        const std::vector<ChVector<float>>& ang_vels = std::vector<ChVector<float>>());

    /// Return particle position.
    ChVector<float> GetParticlePosition(int nSphere) const;

    /// Return particle angular velocity.
    ChVector<float> GetParticleAngVelocity(int nSphere) const;

    /// Return particle linear velocity.
    ChVector<float> GetParticleVelocity(int nSphere) const;

    /// Return position of BC plane.
    ChVector<float> GetBCplanePosition(size_t plane_id) const;

    /// Return number of particle-particle contacts.
    int GetNumContacts() const;

  private:
    ChSystemGpu_impl* m_sys;
};

/// @} gpu_physics

}  // namespace gpu
}  // namespace chrono
