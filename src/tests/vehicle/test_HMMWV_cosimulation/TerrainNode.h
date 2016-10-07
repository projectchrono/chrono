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
// Definition of the TERRAIN NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef HMMWV_COSIM_TERRAINNODE_H
#define HMMWV_COSIM_TERRAINNODE_H

#include <vector>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "BaseNode.h"

// =============================================================================

class TerrainNode : public BaseNode {
  public:
    enum Type { RIGID, GRANULAR };

    TerrainNode(Type type,                                            ///< terrain type (RIGID or GRANULAR)
                chrono::ChMaterialSurfaceBase::ContactMethod method,  ///< contact method (penalty or complementatiry)
                int num_tires,                                        ///< number of vehicle tires
                bool use_checkpoint,                                  ///< initialize granular terrain from checkpoint
                bool render,                                          ///< use OpenGL rendering
                int num_threads                                       ///< number of OpenMP threads
                );
    ~TerrainNode();

    /// Set container dimensions.
    void SetContainerDimensions(double length,    ///< length in X direction (default: 2)
                                double width,     ///< width in Y direction (default: 0.5)
                                double height,    ///< height in Z direction (default: 1)
                                double thickness  ///< wall thickness (default: 0.2)
                                );

    /// Set rear platform length.
    void SetPlatformLength(double length  ///< length in X direction (default: 0)
                           );

    /// Set properties of granular material.
    /// Note that this settings are only relevant when using GRANULAR terrain.
    void SetGranularMaterial(double radius,   ///< particle radius (default: 0.01)
                             double density,  ///< particle material density (default: 2000)
                             int num_layers   ///< number of generated particle layers (default: 5)
                             );

    /// Set properties of proxy bodies (rigid terrain).
    /// When using rigid terrain, the proxy bodies are contact spheres.
    void SetProxyProperties(double mass,    ///< mass of a proxy body (default: 1)
                            double radius,  ///< contact radius of a proxy body (default: 0.01)
                            bool fixed      ///< proxies fixed to ground? (default: false)
                            );

    /// Set properties of proxy bodies (granular terrain).
    /// When using granular terrain, the proxy bodies are contact triangles.
    void SetProxyProperties(double mass,  ///< mass of a proxy body (default: 1)
                            bool fixed    ///< proxies fixed to ground? (default: false)
                            );

    /// Set the material properties for terrain.
    /// The type of material must be consistent with the contact method (penalty or complementarity)
    /// specified at construction. These parameters characterize the material for the container and
    /// (if applicable) the granular material.  Tire contact material is received from one of the tire nodes.
    void SetMaterialSurface(const std::shared_ptr<chrono::ChMaterialSurfaceBase>& mat);

    /// Specify whether contact coefficients are based on material properties (default: true).
    /// Note that this setting is only relevant when using the penalty method.
    void UseMaterialProperties(bool flag);

    /// Set the normal contact force model (default: Hertz)
    /// Note that this setting is only relevant when using the penalty method.
    void SetContactForceModel(chrono::ChSystemDEM::ContactForceModel model);

    /// Set simulation length for settling of granular material (default: 0.4).
    void SetSettlingTime(double time) { m_time_settling = time; }

    /// Enable/disable output during settling (default: false).
    /// If enabled, output files are generated with a frequency of 100 FPS.
    void EnableSettlingOutput(bool val) { m_settling_output = val; }

    /// Initialize this node.
    /// This function allows the node to initialize itself and, optionally, perform an
    /// initial data exchange with any other node.
    virtual void Initialize() override;

    /// Synchronize this node.
    /// This function is called at every co-simulation synchronization time to
    /// allow the node to exchange information with any other node.
    virtual void Synchronize(int step_number, double time) override;

    /// Advance simulation.
    /// This function is called after a synchronization to allow the node to advance
    /// its state by the specified time step.  A node is allowed to take as many internal
    /// integration steps as required, but no inter-node communication should occur.
    virtual void Advance(double step_size) override;

    /// Output logging and debugging data.
    virtual void OutputData(int frame) override;

    /// Obtain settled terrain configuration.
    /// For granular terrain, this can be obtained either through simulation or by initializing
    /// particles from a previously generated checkpointing file.
    void Settle();

    /// Write checkpointing file.
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

    ///
    struct TireData {
        std::shared_ptr<chrono::ChMaterialSurfaceBase> m_material_tire;  ///< material properties for proxy bodies
        std::vector<ProxyBody> m_proxies;          ///< list of proxy bodies with associated mesh index
        std::vector<VertexState> m_vertex_states;  ///< mesh vertex states
        std::vector<Triangle> m_triangles;         ///< tire mesh connectivity
        unsigned int m_num_vert;                   ///< number of tire mesh vertices
        unsigned int m_num_tri;                    ///< number of tire mesh triangles
        unsigned int m_start_vert;                 ///< start vertex index for proxy body identifiers
        unsigned int m_start_tri;                  ///< start triangle index for proxy body identifiers
    };

    Type m_type;  ///< terrain type (RIGID or GRANULAR)

    chrono::ChSystemParallel* m_system;  ///< containing system
    bool m_constructed;                  ///< system construction completed?

    chrono::ChMaterialSurfaceBase::ContactMethod m_method;              ///< contact method (penalty or complementarity)
    std::shared_ptr<chrono::ChMaterialSurfaceBase> m_material_terrain;  ///< material properties for terrain bodies

    int m_num_tires;                    ///< number of vehicle tires
    bool m_fixed_proxies;               ///< flag indicating whether or not proxy bodies are fixed to ground
    std::vector<TireData> m_tire_data;  ///< data for the vehicle tire proxies

    std::shared_ptr<chrono::ChBody> m_platform;  ///< platform rigid body

    double m_hlenX;   ///< rear platform half-length (X direction)
    double m_hdimX;   ///< container half-length (X direction)
    double m_hdimY;   ///< container half-width (Y direction)
    double m_hdimZ;   ///< container half-height (Z direction)
    double m_hthick;  ///< container wall half-thickness

    double m_mass_pN;    ///< mass of a spherical proxy body
    double m_radius_pN;  ///< radius of a spherical proxy body
    double m_mass_pF;    ///< mass of a triangular proxy body

    double m_init_height;  ///< initial terrain height (after optional settling)

    bool m_use_checkpoint;         ///< initialize granular terrain from checkpoint file
    int m_Id_g;                    ///< first identifier for granular material bodies
    int m_num_layers;              ///< number of generated particle layers
    unsigned int m_num_particles;  ///< number of granular material bodies
    double m_radius_g;             ///< radius of one particle of granular material
    double m_rho_g;                ///< particle material density

    double m_time_settling;  ///< simulation length for settling of granular material
    bool m_settling_output;  ///< output files during settling?

    int m_particles_start_index;       ///< start index for granular material bodies in system body list
    unsigned int m_proxy_start_index;  ///< start index for proxy contact shapes in global arrays

    bool m_render;  ///< if true, use OpenGL rendering

    static const std::string m_checkpoint_filename;  ///< name of checkpointing file

    // Private methods

    void Construct();

    void CreateNodeProxies(int which);
    void CreateFaceProxies(int which);

    void UpdateNodeProxies(int which);
    void UpdateFaceProxies(int which);

    void ForcesNodeProxies(int which, std::vector<double>& vert_forces, std::vector<int>& vert_indices);
    void ForcesFaceProxies(int which, std::vector<double>& vert_forces, std::vector<int>& vert_indices);

    void WriteParticleInformation(chrono::utils::CSV_writer& csv);

    void PrintNodeProxiesUpdateData(int which);
    void PrintFaceProxiesUpdateData(int which);

    void PrintNodeProxiesContactData(int which);
    void PrintFaceProxiesContactData(int which);

    static chrono::ChVector<> CalcBarycentricCoords(const chrono::ChVector<>& v1,
                                                    const chrono::ChVector<>& v2,
                                                    const chrono::ChVector<>& v3,
                                                    const chrono::ChVector<>& vP);
};

#endif
