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
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#define MESH_INFO_PRINTF(...)     \
    if (mesh_verbosity == INFO) { \
        printf(__VA_ARGS__);      \
    }

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

class CH_GRANULAR_API ChGranularChronoTriMeshAPI {
  public:
    ChGranularChronoTriMeshAPI(float sphere_rad, float density, float3 boxDims);
    enum MESH_VERBOSITY { QUIET = 0, INFO = 1 };
    ~ChGranularChronoTriMeshAPI() { delete pGranSystemSMC_TriMesh; }

    /// Load triangle meshes into granular system. MUST happen before initialize is called
    void load_meshes(std::vector<std::string> objfilenames,
                     std::vector<chrono::ChMatrix33<float>> rotscale,
                     std::vector<float3> translations,
                     std::vector<float> masses);

    chrono::granular::ChSystemGranularSMC_trimesh& getGranSystemSMC_TriMesh() { return *pGranSystemSMC_TriMesh; }

    // Set particle positions in UU
    void setElemsPositions(const std::vector<chrono::ChVector<float>>& points,
                           const std::vector<chrono::ChVector<float>>& vels = std::vector<chrono::ChVector<float>>(),
                           const std::vector<chrono::ChVector<float>>& ang_vels = 
                           std::vector<chrono::ChVector<float>>());

    // return particle position in UU 
    chrono::ChVector<float> getPosition(int nSphere);
    chrono::ChVector<float> getAngularVelo(int nSphere);
    chrono::ChVector<float> getVelo(int nSphere);


    /// Set simualtion verbosity -- used to check on very large, slow simulations or debug
    void setVerbosity(MESH_VERBOSITY level) { mesh_verbosity = level; }

  private:
    MESH_VERBOSITY mesh_verbosity;
    /// Clean copy of mesh soup interacting with granular material in unified memory. Stored in UU
    chrono::granular::ChSystemGranularSMC_trimesh* pGranSystemSMC_TriMesh;

    /// Setup data structures associated with triangle mesh
    void setupTriMesh(const std::vector<chrono::geometry::ChTriangleMeshConnected>& all_meshes,
                      unsigned int nTriangles,
                      std::vector<float> masses);
};

class CH_GRANULAR_API ChGranularSMC_API {
  public:
    ChGranularSMC_API() : gran_sys(NULL) {}
    // Set particle positions in UU
    void setElemsPositions(const std::vector<chrono::ChVector<float>>& points,
                           const std::vector<chrono::ChVector<float>>& vels = std::vector<chrono::ChVector<float>>(),
                           const std::vector<chrono::ChVector<float>>& ang_vels = 
                           std::vector<chrono::ChVector<float>>());

    // set the gran systems that the user talks to; beef up the API so that the gran system is built through the API,
    // instead of passing a gran system pointer to the API (the API builds the gran system; not the API coming in after
    // gran system is up
    void setGranSystem(chrono::granular::ChSystemGranularSMC* granSystem) { gran_sys = granSystem; }

    chrono::ChVector<float> getPosition(int nSphere);
    chrono::ChVector<float> getAngularVelo(int nSphere);
    chrono::ChVector<float> getVelo(int nSphere);
    chrono::ChVector<float> getBCPlanePos(size_t plane_id);
    int getNumContacts();





  private:
    chrono::granular::ChSystemGranularSMC* gran_sys;
};