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
// Authors: Wei Hu, Radu Serban
// =============================================================================
//
// Definition of the SPH granular TERRAIN NODE (using Chrono::FSI).
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef TESTRIG_TERRAIN_NODE_GRANULAR_SPH_H
#define TESTRIG_TERRAIN_NODE_GRANULAR_SPH_H

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#include "chrono_vehicle/terrain/CRMTerrain.h"
#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeChrono.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim_chrono
/// @{

/// Definition of the SPH continuum representation of granular terrain node (using Chrono::FSI).
class CH_VEHICLE_API ChVehicleCosimTerrainNodeGranularSPH : public ChVehicleCosimTerrainNodeChrono {
  public:
    /// Create a Chrono::FSI granular SPH terrain node.
    /// No SPH parameters are set.
    ChVehicleCosimTerrainNodeGranularSPH(double length, double width, double depth);

    /// Create a Chrono::FSI granular SPH terrain node using parameters from the provided JSON specfile.
    /// See SetFromSpecfile.
    ChVehicleCosimTerrainNodeGranularSPH(const std::string& specfile);

    ~ChVehicleCosimTerrainNodeGranularSPH();

    virtual ChSystem* GetSystem() override { return m_system; }

    /// Set full terrain specification from the provided JSON specfile.
    /// This function is equivalent to calling SetDimensions + SetGranularMaterial + SetPropertiesSPH.
    void SetFromSpecfile(const std::string& specfile);

    /// Specify the SPH terrain properties.
    void SetPropertiesSPH(const std::string& specfile);

    /// Set properties of granular material.
    void SetGranularMaterial(double radius,   ///< particle radius (default: 0.01)
                             double density,  ///< particle material density (default: 2000)
                             double cohesion  ///< particle material cohesion (default: 0)
    );

    /// Set solid visualization.
    void SetSolidVisualization(bool show_geometry = true, bool show_bce = true);

    /// Initialize this Chrono terrain node.
    /// Construct the terrain system and the proxy bodies, then finalize the underlying FSI system.
    virtual void OnInitialize(unsigned int num_objects) override;

    /// Output post-processing visualization data.
    virtual void OutputVisualizationData(int frame) override final;

  private:
    enum class ConstructionMethod { PATCH, FILES };

    ChSystemSMC* m_system;   ///< containing system
    CRMTerrain* m_terrain;   ///< CRM terrain
    std::string m_specfile;  ///< CRM terrain specification file
#ifdef CHRONO_VSG
    std::shared_ptr<vsg3d::ChVisualSystemVSG> m_vsys;  ///< run-time visualization system
#endif

    ConstructionMethod m_terrain_type;  ///< construction method for CRMTerrain
    double m_depth;                     ///< SPH soil depth (PATCH type)
    std::string m_sph_filename;         ///< name of file with SPH particle positions (FILES type)
    std::string m_bce_filename;         ///< name of file with BCE marker positions (FILES type)

    double m_radius;    ///< radius of one particle of granular material
    double m_density;   ///< particle material density
    double m_cohesion;  ///< granular material cohesion

    double m_active_box_size;  ///< size of FSI active domain

    bool m_show_geometry;  ///< show 3D geometry of interacting solids
    bool m_show_bce;       ///< show BCE markers on interacting solids

    virtual ChSystem* GetSystemPostprocess() const override {
#ifdef CHRONO_VSG
        if (m_vsys)
            return &m_vsys->GetSystem(0);
#endif
        return nullptr;
    }

    virtual bool SupportsMeshInterface() const override { return true; }

    virtual void Construct() override;

    virtual void CreateMeshProxy(unsigned int i) override;
    virtual void UpdateMeshProxy(unsigned int i, MeshState& mesh_state) override;
    virtual void GetForceMeshProxy(unsigned int i, MeshContact& mesh_contact) override;
    void PrintMeshProxiesUpdateData(unsigned int i, const MeshState& mesh_state);

    virtual void CreateRigidProxy(unsigned int i) override;
    virtual void UpdateRigidProxy(unsigned int i, BodyState& rigid_state) override;
    virtual void GetForceRigidProxy(unsigned int i, TerrainForce& rigid_contact) override;

    virtual void OnOutputData(int frame) override;
    virtual void OnRender() override;

    /// Advance simulation.
    /// This function is called after a synchronization to allow the node to advance
    /// its state by the specified time step.  A node is allowed to take as many internal
    /// integration steps as required, but no inter-node communication should occur.
    virtual void OnAdvance(double step_size) override;
};

/// @} vehicle_cosim_chrono

}  // end namespace vehicle
}  // end namespace chrono

#endif
