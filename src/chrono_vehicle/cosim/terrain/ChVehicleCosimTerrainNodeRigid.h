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
// Definition of the rigid TERRAIN NODE (using Chrono::Multicore).
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_TERRAIN_NODE_RIGID_H
#define CH_VEHCOSIM_TERRAIN_NODE_RIGID_H

#include "chrono/physics/ChSystem.h"
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeChrono.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim_chrono
/// @{

/// Definition of the rigid terrain node (using Chrono::Multicore).
class CH_VEHICLE_API ChVehicleCosimTerrainNodeRigid : public ChVehicleCosimTerrainNodeChrono {
  public:
    /// Create a rigid terrain node using the specified contact method (SMC or NSC).
    ChVehicleCosimTerrainNodeRigid(double length, double width, ChContactMethod method);

    /// Create a rigid terrain node using the specified contact method (SMC or NSC) and set parameters from the provided
    /// JSON specfile.
    ChVehicleCosimTerrainNodeRigid(const std::string& specfile, ChContactMethod method);

    ~ChVehicleCosimTerrainNodeRigid();

    virtual ChSystem* GetSystem() override { return m_system; }

    /// Set full terrain specification from JSON specfile.
    void SetFromSpecfile(const std::string& specfile);

    /// Set the material properties for terrain.
    /// The type of material must be consistent with the contact method (SMC or NSC)
    /// specified at construction. These parameters characterize the material for the container and
    /// (if applicable) the granular material.  Object contact material is received from the rig node.
    void SetMaterialSurface(const std::shared_ptr<ChMaterialSurface>& mat);

    /// Specify whether contact coefficients are based on material properties (default: true).
    /// Note that this setting is only relevant when using the SMC method.
    void UseMaterialProperties(bool flag);

    /// Set the normal contact force model (default: Hertz)
    /// Note that this setting is only relevant when using the SMC method.
    void SetContactForceModel(ChSystemSMC::ContactForceModel model);

    /// Set proxy contact radius (default: 0.01).
    /// When using a rigid object mesh, this is a "thickness" for the collision mesh (a non-zero value can improve
    /// robustness of the collision detection algorithm).  When using a flexible object, this is the radius of the proxy
    /// spheres attached to each FEA mesh node.
    void SetProxyContactRadius(double radius) { m_radius_p = radius; }

    /// Output post-processing visualization data.
    virtual void OutputVisualizationData(int frame) override final;

    /// Initialize this Chrono terrain node.
    /// Construct the terrain system and the proxy bodies, then finalize the underlying system.
    virtual void OnInitialize(unsigned int num_objects) override;

  private:
    ChSystem* m_system;                      ///< containing system
    double m_radius_p;                       ///< radius for a proxy body
    std::shared_ptr<ChVisualSystem> m_vsys;  ///< run-time visualization system

    virtual ChSystem* GetSystemPostprocess() const override { return m_system; }

    virtual bool SupportsMeshInterface() const override { return true; }

    virtual void Construct() override;

    /// Return current total number of contacts.
    virtual int GetNumContacts() const override { return m_system->GetNcontacts(); }

    virtual void CreateMeshProxy(unsigned int i) override;
    virtual void UpdateMeshProxy(unsigned int i, MeshState& mesh_state) override;
    virtual void GetForceMeshProxy(unsigned int i, MeshContact& mesh_contact) override;
    void PrintMeshProxiesUpdateData(unsigned int i, const MeshState& mesh_state);

    virtual void CreateRigidProxy(unsigned int i) override;
    virtual void UpdateRigidProxy(unsigned int i, BodyState& rigid_state) override;
    virtual void GetForceRigidProxy(unsigned int i, TerrainForce& rigid_contact) override;

    virtual void OnRender() override;
};

/// @} vehicle_cosim_chrono

}  // end namespace vehicle
}  // end namespace chrono

#endif
