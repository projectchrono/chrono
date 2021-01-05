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
    /// Construct system with given sphere radius, density, and big domain dimensions.
    ChSystemGpuMesh(float sphere_rad, float density, float3 boxDims);
    ~ChSystemGpuMesh();

    /// Load triangle meshes into granular system. MUST happen before initialize is called.
    void LoadMeshes(std::vector<std::string> objfilenames,
                    std::vector<ChMatrix33<float>> rotscale,
                    std::vector<float3> translations,
                    std::vector<float> masses);

    /// Enable/disable mesh collision (for all defined meshes).
    void EnableMeshCollision(bool val);

    /// Apply rigid body motion to specified mesh.
    void ApplyMeshMotion(unsigned int mesh,
                         const ChVector<>& pos,
                         const ChQuaternion<>& rot,
                         const ChVector<>& lin_vel,
                         const ChVector<>& ang_vel);

    /// Return the number of meshes in the system.
    unsigned int GetNumMeshes() const;

    ChSystemGpuMesh_impl& getSystem() { return *m_sys_trimesh; }

    /// Set particle positions.
    void SetParticlePositions(const std::vector<ChVector<float>>& points,
                              const std::vector<ChVector<float>>& vels = std::vector<ChVector<float>>(),
                              const std::vector<ChVector<float>>& ang_vels = std::vector<ChVector<float>>());

    /// Set flags indicating whether or not a particle is fixed.
    /// MUST be called only once and MUST be called before Initialize.
    void SetParticleFixed(const std::vector<bool>& fixed);

    /// Set simulation verbosity level.
    void SetVerbosity(CHGPU_VERBOSITY level);

    /// Set verbosity level of mesh operations.
    void SetMeshVerbosity(CHGPU_MESH_VERBOSITY level);

    /// Return particle position.
    ChVector<float> GetParticlePosition(int nSphere) const;

    /// Return particle angular velocity.
    ChVector<float> GetParticleAngVelocity(int nSphere) const;

    /// Return particle velocity.
    ChVector<float> GetParticleVelocity(int nSphere) const;

    /// Initialize simulation so that it can be advanced.
    /// Must be called before AdvanceSimulation and after simulation parameters are set.
    /// This function initializes both the granular material and any existing trimeshes.
    void Initialize();

    /// Advance simulation by duration in user units, return actual duration elapsed.
    /// Requires Initialize() to have been called.
    double AdvanceSimulation(float duration);

    /// Collect contact forces exerted on all meshes by the granular system.
    void CollectMeshContactForces(std::vector<ChVector<>>& forces, std::vector<ChVector<>>& torques);

    /// Collect contact forces exerted on the specified meshe by the granular system.
    void CollectMeshContactForces(int mesh, ChVector<>& force, ChVector<>& torque);

    /// Write particle positions according to the system output mode.
    void WriteFile(std::string ofile) const;

    /// Write visualization files for triangle meshes with current positions.
    void WriteMeshes(std::string outfilename) const;

  private:
    CHGPU_MESH_VERBOSITY mesh_verbosity;  ///< mesh operations verbosity level

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
    void SetParticlePositions(const std::vector<ChVector<float>>& points,
                              const std::vector<ChVector<float>>& vels = std::vector<ChVector<float>>(),
                              const std::vector<ChVector<float>>& ang_vels = std::vector<ChVector<float>>());

    /// Set flags indicating whether or not a particle is fixed.
    /// MUST be called only once and MUST be called before Initialize.
    void SetParticleFixed(const std::vector<bool>& fixed);

    /// Set simualtion verbosity level.
    void SetVerbosity(CHGPU_VERBOSITY level);

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

    /// Initialize simulation so that it can be advanced.
    /// Must be called before AdvanceSimulation and after simulation parameters are set.
    void Initialize();

    /// Advance simulation by duration in user units, return actual duration elapsed.
    /// Requires Initialize() to have been called.
    double AdvanceSimulation(float duration);

    /// Write particle positions according to the system output mode.
    void WriteFile(std::string ofile) const;

  private:
    ChSystemGpu_impl* m_sys;
};

/// @} gpu_physics

}  // namespace gpu
}  // namespace chrono
