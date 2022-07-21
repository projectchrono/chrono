// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Definition of the MPI granular TERRAIN NODE (using Chrono::Distributed).
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef TESTRIG_TERRAIN_NODE_GRANULAR_MPI_H
#define TESTRIG_TERRAIN_NODE_GRANULAR_MPI_H

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_distributed/physics/ChSystemDistributed.h"

#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeChrono.h"

#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim_chrono
/// @{

/// Definition of the MPI granular terrain node (using Chrono::Distributed).
class CH_VEHICLE_API ChVehicleCosimTerrainNodeGranularMPI : public ChVehicleCosimTerrainNodeChrono {
  public:
    /// Create a Chrono::Distributed granular terrain node.
    ChVehicleCosimTerrainNodeGranularMPI(double length, double width);

    /// Create a Chrono::Distributed granular terrain node and set parameters from the provided JSON specfile.
    ChVehicleCosimTerrainNodeGranularMPI(const std::string& specfile);

    ~ChVehicleCosimTerrainNodeGranularMPI();

    virtual ChSystem* GetSystem() override { return m_system; }

    /// Set the number of OpenMP threads for each Chrono::Multicore terrain simulation (default: 1).
    void SetNumThreads(int num_threads);

    /// Set full terrain specification from JSON specfile.
    void SetFromSpecfile(const std::string& specfile);

    /// Set container wall thickness (default: 0.2)
    void SetWallThickness(double thickness);

    /// Set properties of granular material.
    void SetGranularMaterial(double radius,  ///< particle radius (default: 0.01)
                             double density  ///< particle material density (default: 2000)
    );

    /// Set the material properties for terrain.
    /// These parameters characterize the material for the container and the granular material.
    /// Tire contact material is received from the MBS node.
    void SetMaterialSurface(const std::shared_ptr<ChMaterialSurfaceSMC>& mat);

    /// Specify whether contact coefficients are based on material properties (default: true).
    void UseMaterialProperties(bool flag);

    /// Set the normal contact force model (default: Hertz).
    void SetContactForceModel(ChSystemSMC::ContactForceModel model);

    /// Set the tangential contact displacement model (default: OneStep).
    void SetTangentialDisplacementModel(ChSystemSMC::TangentialDisplacementModel model);

    /// Set sampling method for generation of granular material.
    /// The granular material is created in the volume defined by the x-y dimensions of the terrain patch and the
    /// specified initial height, using the specified sampling type, layer by layer or all at once.
    /// Note: for correct HCP, do not initialize in layers!
    void SetSamplingMethod(utils::SamplingType type,   ///< volume sampling type (default POISSON_DISK)
                           double init_height,         ///< height of granular material at initialization (default 0.2)
                           double sep_factor = 1.001,  ///< radius inflation factor for initial separation
                           bool in_layers = false      ///< initialize material in layers
    );

    /// Set sweeping sphere radius for proxy bodies (default 5e-3).
    /// This value is used as a "thickness" for collision meshes (a non-zero value can improve robustness of the
    /// collision detection algorithm).
    void SetProxyContactRadius(double radius) { m_radius_p = radius; }

    /// Write checkpoint to the specified file (which will be created in the output directory).
    virtual void WriteCheckpoint(const std::string& filename) const override;

    /// Estimate packing density (eta) of granular material in current configuration.
    /// Note that porosity is phi=1-eta and void ratio is e=(1-eta)/eta=phi/(1-phi).
    /// The function also returns the current depth of granular material.
    double CalculatePackingDensity(double& depth);

  private:
    /// Additional data for tire proxy bodies
    struct TireData {
        std::vector<uint> m_gids;              ///< global indices of proxy bodies
        std::unordered_map<uint, uint> m_map;  ///< map from global ID to triangle index
        unsigned int m_start_tri;              ///< start triangle index for proxy body identifiers
    };

    ChSystemDistributed* m_system;  ///< containing system
    bool m_constructed;             ///< system construction completed?
    bool m_proxies_constructed;     ///< proxy bodies created?

#ifdef CHRONO_OPENGL
    opengl::ChVisualSystemOpenGL* m_vsys;  ///< OpenGL visualization system
#endif

    int m_sub_rank;  ///< MPI rank within the terrain MPI intracommunicator

    double m_hthick;                    ///< container wall half-thickness
    double m_radius_p;                  ///< radius for a proxy body
    std::vector<TireData> m_tire_data;  ///< data for the vehicle tire proxies

    utils::SamplingType m_sampling_type;  ///< sampling method for generation of particles
    double m_init_depth;                  ///< height of granular maerial initialization volume
    double m_separation_factor;           ///< radius inflation factor for initial particle separation
    bool m_in_layers;                     ///< initialize material layer-by-layer (true) or all at once (false)

    unsigned int m_num_particles;  ///< number of granular material bodies
    double m_radius_g;             ///< radius of one particle of granular material
    double m_rho_g;                ///< particle material density

    virtual bool SupportsMeshInterface() const override { return true; }

    virtual void Construct() override;

    /// Return current total number of contacts.
    virtual int GetNumContacts() const override { return m_system->GetNcontacts(); }

    virtual void CreateMeshProxies(unsigned int i) override;
    virtual void UpdateMeshProxies(unsigned int i, MeshState& mesh_state) override;
    virtual void GetForcesMeshProxies(unsigned int i, MeshContact& mesh_contact) override;

    virtual void CreateWheelProxy(unsigned int i) override;
    virtual void UpdateWheelProxy(unsigned int i, BodyState& spindle_state) override;
    virtual void GetForceWheelProxy(unsigned int i, TerrainForce& wheel_contact) override;

    virtual void OnAdvance(double step_size) override;
    virtual void OnOutputData(int frame) override;
    virtual void Render(double time) override;

    /// Create the Chrono::Distributed system.
    void CreateSystem();

    /// Distribute tire mesh information from main terrain node to intra-communicator.
    void ScatterInitData(unsigned int i);

    /// Create the tire mesh proxy bodies.
    void CreateMeshProxiesInternal(unsigned int i);
    void CreateWheelProxyInternal(unsigned int i);

    /// Calculate current height of granular terrain.
    double CalcCurrentHeight();

    /// Calculate total kinetic energy of granular material.
    double CalcTotalKineticEnergy();

    static ChVector<> CalcBarycentricCoords(const ChVector<>& v1,
                                            const ChVector<>& v2,
                                            const ChVector<>& v3,
                                            const ChVector<>& vP);
};

/// @} vehicle_cosim_chrono

}  // end namespace vehicle
}  // end namespace chrono

#endif
