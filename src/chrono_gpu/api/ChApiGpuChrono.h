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

inline void convertChVector2Float3Vec(const std::vector<chrono::ChVector<float>>& points,
                                      std::vector<float3>& pointsFloat3) {
    size_t nPoints = points.size();
    pointsFloat3.resize(nPoints);
    for (size_t index = 0; index < nPoints; index++) {
        pointsFloat3.at(index).x = points.at(index)[0];
        pointsFloat3.at(index).y = points.at(index)[1];
        pointsFloat3.at(index).z = points.at(index)[2];
    }
}

class CH_GPU_API ChGpuSMCtrimesh_API {
  public:
    enum class MeshVerbosity { QUIET = 0, INFO = 1 };

    ChGpuSMCtrimesh_API(float sphere_rad, float density, float3 boxDims);
    ~ChGpuSMCtrimesh_API() { delete m_sys_trimesh; }

    /// Load triangle meshes into granular system. MUST happen before initialize is called
    void load_meshes(std::vector<std::string> objfilenames,
                     std::vector<chrono::ChMatrix33<float>> rotscale,
                     std::vector<float3> translations,
                     std::vector<float> masses);

    /// Setup data structures associated with triangle mesh
    void set_meshes(const std::vector<chrono::geometry::ChTriangleMeshConnected>& all_meshes,
                    std::vector<float> masses);

    chrono::gpu::ChSystemGpuSMC_trimesh& getSystem() { return *m_sys_trimesh; }

    // Set particle positions in UU
    void setElemsPositions(const std::vector<chrono::ChVector<float>>& points,
                           const std::vector<chrono::ChVector<float>>& vels = std::vector<chrono::ChVector<float>>(),
                           const std::vector<chrono::ChVector<float>>& ang_vels = 
                           std::vector<chrono::ChVector<float>>());

    // return particle position in UU 
    chrono::ChVector<float> getPosition(int nSphere);
    chrono::ChVector<float> getAngularVelo(int nSphere);
    chrono::ChVector<float> getVelo(int nSphere);

    /// Set simulation verbosity -- used to check on very large, slow simulations or debug
    void setVerbosity(MeshVerbosity level) { mesh_verbosity = level; }

  private:
    MeshVerbosity mesh_verbosity;

    /// Clean copy of mesh soup interacting with granular material in unified memory. Stored in UU
    chrono::gpu::ChSystemGpuSMC_trimesh* m_sys_trimesh;
};

class CH_GPU_API ChGpuSMC_API {
  public:
    ChGpuSMC_API() : m_sys(NULL) {}
    // Set particle positions in UU
    void setElemsPositions(
        const std::vector<chrono::ChVector<float>>& points,
        const std::vector<chrono::ChVector<float>>& vels = std::vector<chrono::ChVector<float>>(),
        const std::vector<chrono::ChVector<float>>& ang_vels = std::vector<chrono::ChVector<float>>());

    // set the GPU systems that the user talks to; beef up the API so that the system is built through the API, instead
    // of passing a system pointer to the API (the API builds the  system; not the API coming in after system is up)
    void setSystem(chrono::gpu::ChSystemGpuSMC* sys) { m_sys = sys; }

    chrono::ChVector<float> getPosition(int nSphere);
    chrono::ChVector<float> getAngularVelo(int nSphere);
    chrono::ChVector<float> getVelo(int nSphere);
    chrono::ChVector<float> getBCPlanePos(size_t plane_id);
    int getNumContacts();

  private:
    chrono::gpu::ChSystemGpuSMC* m_sys;
};
