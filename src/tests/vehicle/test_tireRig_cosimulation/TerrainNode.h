// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Antonio Recuero
// =============================================================================
//
// Mechanism for testing tires over granular terrain.  The mechanism + tire
// system is co-simulated with a Chrono::Parallel system for the granular terrain.
//
// Definition of the TERRAIN NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

//// TODO:
////    better approximation of mass / inertia? (CreateFaceProxies)
////    angular velocity (UpdateFaceProxies)
////    implement (PrintFaceProxiesContactData)
////    mesh connectivity doesn't need to be communicated every time (modify Chrono?)  

#ifndef TESTRIG_TERRAINNODE_H
#define TESTRIG_TERRAINNODE_H

#include <omp.h>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>

#include "chrono/core/ChTimer.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

// =============================================================================

class TerrainNode {
  public:
    enum Type { RIGID, GRANULAR };

    TerrainNode(PhaseType phase,                                      ///< problem type (SETTLING or TESTING)
                Type type,                                            ///< terrain type (RIGID or GRANULAR)
                chrono::ChMaterialSurfaceBase::ContactMethod method,  ///< contact method (penalty or complementatiry)
                int num_threads                                       ///< number of OpenMP threads
                );
    ~TerrainNode();

    void SetOutputFile(const std::string& name);

    void Settle();
    void Initialize();
    void Synchronize(int step_number, double time);
    void Advance(double step_size);

    double GetSimTime() { return m_timer.GetTimeSeconds(); }
    double GetTotalSimTime() { return m_cumm_sim_time; }
    void OutputData(int frame);
    void WriteCheckpoint();

  private:
    /// Triangle vertex indices.
    struct Triangle {
        int v1;
        int v2;
        int v3;
    };

    /// Mesh vertex state.
    struct VertexState {
        chrono::ChVector<> pos;
        chrono::ChVector<> vel;
    };

    /// Association between a proxy body and a mesh index.
    /// The body can be associated with either a mesh vertex or a mesh triangle.
    struct ProxyBody {
        ProxyBody(std::shared_ptr<chrono::ChBody> body, int index) : m_body(body), m_index(index) {}
        std::shared_ptr<chrono::ChBody> m_body;
        int m_index;
    };

    PhaseType m_phase;  ///< problem type (SETTLING or TESTING)

    Type m_type;  ///< terrain type (RIGID or GRANULAR)

    chrono::ChMaterialSurfaceBase::ContactMethod m_method;  ///< contact method (penalty or complementarity)
    chrono::ChSystemParallel* m_system;                     ///< containing system

    std::shared_ptr<chrono::ChMaterialSurfaceBase> m_material_tire;  ///< material properties for proxy bodies
    std::vector<ProxyBody> m_proxies;  ///< list of proxy bodies with associated mesh index
    bool m_fixed_proxies;              ///< flag indicating whether or not proxy bodies are fixed to ground

    double m_hdimX;   ///< container half-length (X direction)
    double m_hdimY;   ///< container half-width (Y direction)
    double m_hdimZ;   ///< container half-height (Z direction)
    double m_hthick;  ///< container wall half-thickness

    double m_mass_pN;    ///< mass of a spherical proxy body
    double m_radius_pN;  ///< radius of a spherical proxy body
    double m_mass_pF;    ///< mass of a triangular proxy body

    double m_init_height;  ///< initial terrain height (after optional settling)

    int m_Id_g;                    ///< first identifier for granular material bodies
    unsigned int m_num_particles;  ///< number of granular material bodies
    double m_radius_g;             ///< radius of one particle of granular material

    unsigned int m_num_vert;  ///< number of tire mesh vertices
    unsigned int m_num_tri;   ///< number of tire mesh triangles

    std::vector<VertexState> m_vertex_states;  ///< mesh vertex states
    std::vector<Triangle> m_triangles;         ///< tire mesh connectivity

    int m_particles_start_index;       ///< start index for granular material bodies in system body list
    unsigned int m_proxy_start_index;  ///< start index for proxy contact shapes in global arrays

    std::ofstream m_outf;  ///< output file stream
    chrono::ChTimer<double> m_timer;
    double m_cumm_sim_time;

    static const std::string m_checkpoint_filename;  ///< name of checkpointing file

    // Private methods
    void CreateNodeProxies();
    void CreateFaceProxies();

    void UpdateNodeProxies();
    void UpdateFaceProxies();

    void ForcesNodeProxies(std::vector<double>& vert_forces, std::vector<int>& vert_indices);
    void ForcesFaceProxies(std::vector<double>& vert_forces, std::vector<int>& vert_indices);

    void PrintMeshUpdateData();
    void PrintNodeProxiesUpdateData();
    void PrintFaceProxiesUpdateData();

    void PrintNodeProxiesContactData();
    void PrintFaceProxiesContactData();

    bool vertex_height_comparator(const ProxyBody& a, const ProxyBody& b);

    static chrono::ChVector<> CalcBarycentricCoords(const chrono::ChVector<>& v1,
                                                    const chrono::ChVector<>& v2,
                                                    const chrono::ChVector<>& v3,
                                                    const chrono::ChVector<>& vP);
};

#endif