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
// Authors: Nic Olsen, Radu Serban
// =============================================================================
//
// Definition of the TERRAIN NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef HMMWV_COSIM_TERRAINNODEDISTR_H
#define HMMWV_COSIM_TERRAINNODEDISTR_H

#include <unordered_map>
#include <vector>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_granular/physics/ChGranular.h"

#include "BaseNode.h"

// =============================================================================

class TerrainNodeGran : public BaseNode {
  public:
    TerrainNodeGran(int num_tires  ///< number of vehicle tires
    );
    ~TerrainNodeGran();

    /// Set up output directories for distributed terrain.
    /// Must be called on all ranks in the terrain intra-communicator.
    virtual void SetOutDir(const std::string& dir_name, const std::string& suffix) override;

    /// Set container dimensions.
    void SetContainerDimensions(double length,  ///< length in X direction (default: 2)
                                double width,   ///< width in Y direction (default: 0.5)
                                double height   ///< height in Z direction (default: 1)
    );

    /// Set properties of granular material.
    /// Note that this settings are only relevant when using GRANULAR terrain.
    void SetGranularMaterial(double radius,   ///< particle radius (default: 0.01)
                             double density,  ///< particle material density (default: 2000)
                             int num_layers   ///< number of generated particle layers (default: 5)
    );

    /// Set properties of proxy bodies.
    void SetProxyProperties(double mass,  ///< mass of a proxy body (default: 1)
                            bool fixed    ///< proxies fixed to ground? (default: false)
    );

    /// Set the material properties for terrain.
    /// These parameters characterize the material for the container and the granular material.
    /// Tire contact material is received from one of the tire nodes.
    void SetMaterialSurface(const std::shared_ptr<chrono::ChMaterialSurfaceSMC>& mat);

    /// Set simulation length for settling of granular material (default: 0.4).
    void SetSettlingTime(double time) { m_time_settling = time; }

    /// Enable/disable output during settling (default: false).
    /// If enabled, output files are generated with the specified frequency.
    void EnableSettlingOutput(bool val, double fps) {
        m_settling_output = val;
        m_settling_output_fps = fps;
    }

    /// Enable/disable output of initial body information (default: false).
    /// Output includes body information after granular material creation (files init_particles_xxx.dat)
    /// and body information after proxy creation (files init_bodies_xxx.dat) on all terrain ranks.
    void EnableInitialOutput(bool val) { m_initial_output = val; }

    /// Enable rendering of the specified Bezier path.
    void SetPath(std::shared_ptr<chrono::ChBezierCurve> path);

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

    /// Write debugging proxy information for the specified tire after synchronization.
    void DumpProxyData(int which) const;

  private:
    /// Data for tire representation through proxy bodies.
    struct TireData {
        std::vector<uint> m_gids;                        ///< global indices of proxy bodies
        std::vector<chrono::ChVector<>> m_vertex_pos;    ///< mesh vertex positions
        std::vector<chrono::ChVector<>> m_vertex_vel;    ///< mesh vertex velocities
        std::vector<chrono::ChVector<int>> m_triangles;  ///< tire mesh connectivity
        std::unordered_map<uint, uint> m_map;            ///< map from global ID to triangle index
        unsigned int m_num_vert;                         ///< number of tire mesh vertices
        unsigned int m_num_tri;                          ///< number of tire mesh triangles
        unsigned int m_start_tri;                        ///< start triangle index for proxy body identifiers
    };

    int m_world_rank;  ///< process rank in MPI_COMM_WORLD

    chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless* m_system;  ///< containing system
    bool m_constructed;                                                         ///< system construction completed?

    std::shared_ptr<chrono::ChMaterialSurfaceSMC> m_material_terrain;  ///< material properties for terrain bodies

    int m_num_tires;                    ///< number of vehicle tires
    std::vector<TireData> m_tire_data;  ///< data for the vehicle tire proxies

    double m_hdimX;  ///< container half-length (X direction)
    double m_hdimY;  ///< container half-width (Y direction)
    double m_hdimZ;  ///< container half-height (Z direction)

    bool m_fixed_proxies;  ///< flag indicating whether or not proxy bodies are fixed to ground
    double m_mass_pF;      ///< mass of a triangular proxy body

    int m_Id_g;                    ///< first identifier for granular material bodies
    int m_num_layers;              ///< number of generated particle layers
    unsigned int m_num_particles;  ///< number of granular material bodies
    double m_radius_g;             ///< radius of one particle of granular material
    double m_rho_g;                ///< particle material density

    double m_time_settling;        ///< simulation length for settling of granular material
    bool m_settling_output;        ///< generate output files during settling?
    double m_settling_output_fps;  ///< output frequency during settling
    bool m_initial_output;         ///< generate output files with initial particle information?

    int m_particles_start_index;  ///< start index for granular material bodies in system body list
    double m_initial_height;      ///< highest particle Z coordinate when proxies are created

    std::shared_ptr<chrono::ChBezierCurve> m_path;  ///< path for closed-loop driver (for rendering only, may be empty)

    bool m_render;  ///< if true, use OpenGL rendering

    std::string m_rank_out_dir;  ///< rank-specific output directory

    static const std::string m_checkpoint_filename;  ///< name of checkpointing file

    // Private methods

    void Construct();

    void CreateFaceProxies(int which, std::shared_ptr<chrono::ChMaterialSurfaceSMC> material);

    void UpdateFaceProxies(int which);

    void ForcesFaceProxies(int which, std::vector<double>& vert_forces, std::vector<int>& vert_indices);

    void WriteParticleInformation(chrono::utils::CSV_writer& csv);

    void PrintFaceProxiesUpdateData(int which);

    void PrintFaceProxiesContactData(int which);

    static chrono::ChVector<> CalcBarycentricCoords(const chrono::ChVector<>& v1,
                                                    const chrono::ChVector<>& v2,
                                                    const chrono::ChVector<>& v3,
                                                    const chrono::ChVector<>& vP);
};

#endif