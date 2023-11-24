// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Template for a rigid tire
//
// =============================================================================

#ifndef CH_RIGIDTIRE_H
#define CH_RIGIDTIRE_H

#include <vector>

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/assets/ChVisualShapeCylinder.h"

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Rigid tire model.
/// This tire is modeled as a rigid cylinder.  Requires a terrain system that
/// supports rigid contact with friction.
class CH_VEHICLE_API ChRigidTire : public ChTire {
  public:
    ChRigidTire(const std::string& name  ///< [in] name of this tire system
                );

    virtual ~ChRigidTire();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "RigidTire"; }

    /// Set Wavefront OBJ file for contact mesh.
    void SetMeshFilename(const std::string& mesh_file,   ///< [in] name of Wavefront file
                         double sweep_sphere_radius = 0  ///< [in] radius of sweeping sphere
                         );

    /// Check whether or not this tire uses a contact mesh.
    bool UseContactMesh() const { return m_use_contact_mesh; }

    /// Report the tire force and moment.
    /// This generalized force encapsulates the tire-terrain forces (i.e. the resultant
    /// of all contact forces acting on the tire). The force and moment are expressed
    /// in global frame, as applied to the center of the associated wheel.
    virtual TerrainForce ReportTireForce(ChTerrain* terrain) const override;

    /// Get the tire force and moment expressed in the tire frame.
    /// Currently *NOT IMPLEMENTED*.
    virtual TerrainForce ReportTireForceLocal(ChTerrain* terrain, ChCoordsys<>& tire_frame) const override;

    /// Get the tire contact material.
    /// Note that this is not set until after tire initialization.
    std::shared_ptr<ChMaterialSurface> GetContactMaterial() const { return m_material; }

    /// Add visualization assets for the rigid tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the rigid tire subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Get the contact mesh.
    std::shared_ptr<geometry::ChTriangleMeshConnected> GetContactMesh() const;

    /// Get the current state of the collision mesh.
    /// Mesh vertex positions and velocities are returned in the absolute frame.
    void GetMeshVertexStates(std::vector<ChVector<>>& pos,  ///< mesh vertex positions (expressed in absolute frame)
                             std::vector<ChVector<>>& vel   ///< mesh vertex velocities (expressed in absolute frame)
    ) const;

  protected:
    /// Create the contact material consistent with the specified contact method.
    virtual void CreateContactMaterial(ChContactMethod contact_method) = 0;

    std::shared_ptr<ChMaterialSurface> m_material;  ///< contact material;

    virtual void InitializeInertiaProperties() override final;
    virtual void UpdateInertiaProperties() override final;

    virtual double GetAddedMass() const override final;
    virtual ChVector<> GetAddedInertia() const override final;

    /// Get the tire force and moment.
    /// A ChRigidTire always returns zero force and moment since tire
    /// forces are automatically applied to the associated wheel through Chrono's
    /// frictional contact system.
    virtual TerrainForce GetTireForce() const override final;

    /// Initialize this tire by associating it to the specified wheel.
    virtual void Initialize(std::shared_ptr<ChWheel> wheel) override;

    /// Update the state of this tire system at the current time.
    virtual void Synchronize(double time,              ///< [in] current time
                             const ChTerrain& terrain  ///< [in] reference to the terrain system
                             ) override;

    bool m_use_contact_mesh;         ///< flag indicating use of a contact mesh
    std::string m_contact_meshFile;  ///< name of the OBJ file for contact mesh
    double m_sweep_sphere_radius;    ///< radius of sweeping sphere for mesh contact

    std::shared_ptr<geometry::ChTriangleMeshConnected> m_trimesh;  ///< contact mesh

    std::shared_ptr<ChVisualShape> m_cyl_shape;  ///< visualization cylinder asset
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
