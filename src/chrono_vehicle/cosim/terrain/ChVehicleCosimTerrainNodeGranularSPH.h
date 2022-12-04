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
#include "chrono_fsi/ChSystemFsi.h"

#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeChrono.h"

#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChVisualSystemOpenGL.h"
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
    ChVehicleCosimTerrainNodeGranularSPH(double length, double width);

    /// Create a Chrono::FSI granular SPH terrain node using parameters from the provided JSON specfile.
    /// See SetFromSpecfile.
    ChVehicleCosimTerrainNodeGranularSPH(const std::string& specfile);

    ~ChVehicleCosimTerrainNodeGranularSPH();

    virtual ChSystem* GetSystem() override { return m_system; }

    /// Set full terrain specification from the provided JSON specfile.
    /// This function is equivalent to calling SetDimensions + SetGranularMaterial + SetPropertiesSPH.
    void SetFromSpecfile(const std::string& specfile);

    /// Specify the SPH terrain properties.
    void SetPropertiesSPH(const std::string& filename, double depth);

    /// Set properties of granular material.
    void SetGranularMaterial(double radius,  ///< particle radius (default: 0.01)
                             double density  ///< particle material density (default: 2000)
    );

    /// Initialize this Chrono terrain node.
    /// Construct the terrain system and the proxy bodies, then finalize the underlying FSI system.
    virtual void OnInitialize(unsigned int num_objects) override;

    /// Output post-processing visualization data.
    virtual void OutputVisualizationData(int frame) override final;

  private:
    ChSystemSMC* m_system;          ///< containing system
    fsi::ChSystemFsi* m_systemFSI;  ///< containing FSI system

#ifdef CHRONO_OPENGL
    opengl::ChVisualSystemOpenGL* m_vsys;  ///< OpenGL visualization system
#endif

    double m_depth;  ///< SPH soil depth

    double m_radius_g;  ///< radius of one particle of granular material
    double m_rho_g;     ///< particle material density

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
    virtual void Render(double time) override;

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
