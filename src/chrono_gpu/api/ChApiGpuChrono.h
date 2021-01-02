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
// Contributors: Dan Negrut, Nic Olsen
// =============================================================================

#pragma once

#include <vector>

#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_gpu/physics/ChGpu.h"
#include "chrono_gpu/physics/ChGpuTriMesh.h"

namespace chrono {
namespace gpu {

// -----------------------------------------------------------------------------

class CH_GPU_API ChGpuSMCtrimesh_API {
  public:
    enum class MeshVerbosity { QUIET = 0, INFO = 1 };

    ChGpuSMCtrimesh_API(float sphere_rad, float density, float3 boxDims);
    ~ChGpuSMCtrimesh_API() { delete m_sys_trimesh; }

    /// Load triangle meshes into granular system. MUST happen before initialize is called.
    void load_meshes(std::vector<std::string> objfilenames,
                     std::vector<chrono::ChMatrix33<float>> rotscale,
                     std::vector<float3> translations,
                     std::vector<float> masses);

    /// Setup data structures associated with triangle mesh.
    void set_meshes(const std::vector<chrono::geometry::ChTriangleMeshConnected>& all_meshes,
                    std::vector<float> masses);

    chrono::gpu::ChSystemGpuSMC_trimesh& getSystem() { return *m_sys_trimesh; }

    /// Set particle positions in UU.
    void setElemsPositions(
        const std::vector<chrono::ChVector<float>>& points,
        const std::vector<chrono::ChVector<float>>& vels = std::vector<chrono::ChVector<float>>(),
        const std::vector<chrono::ChVector<float>>& ang_vels = std::vector<chrono::ChVector<float>>());

    /// Return particle position in UU.
    chrono::ChVector<float> getPosition(int nSphere);

    /// Return particle angular velocity in UU.
    chrono::ChVector<float> getAngularVelo(int nSphere);

    /// Return particle velocity in UU.
    chrono::ChVector<float> getVelo(int nSphere);

    /// Set simulation verbose level.
    /// Used to check on very large, slow simulations or for debugging.
    void setVerbosity(MeshVerbosity level) { mesh_verbosity = level; }

  private:
    MeshVerbosity mesh_verbosity;

    /// Clean copy of mesh soup interacting with granular material in unified memory. Stored in UU
    chrono::gpu::ChSystemGpuSMC_trimesh* m_sys_trimesh;
};

// -----------------------------------------------------------------------------

class CH_GPU_API ChGpuSMC_API {
  public:
    ChGpuSMC_API() : m_sys(nullptr) {}

    /// Set particle positions in UU.
    void setElemsPositions(
        const std::vector<chrono::ChVector<float>>& points,
        const std::vector<chrono::ChVector<float>>& vels = std::vector<chrono::ChVector<float>>(),
        const std::vector<chrono::ChVector<float>>& ang_vels = std::vector<chrono::ChVector<float>>());

    /// Set the GPU systems that the user talks to.
    /// Note that the system is built through the API, instead of passing a system pointer to the API.
    void setSystem(chrono::gpu::ChSystemGpuSMC* sys) { m_sys = sys; }

    chrono::ChVector<float> getPosition(int nSphere);
    chrono::ChVector<float> getAngularVelo(int nSphere);
    chrono::ChVector<float> getVelo(int nSphere);
    chrono::ChVector<float> getBCPlanePos(size_t plane_id);
    int getNumContacts();

  private:
    chrono::gpu::ChSystemGpuSMC* m_sys;
};

}  // namespace gpu
}  // namespace chrono
