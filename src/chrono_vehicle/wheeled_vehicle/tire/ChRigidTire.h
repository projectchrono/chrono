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
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"

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

    /// Get the tire width.
    virtual double GetWidth() const = 0;

    /// Report the tire force and moment.
    /// This generalized force encapsulates the tire-terrain forces (i.e. the resultant
    /// of all contact forces acting on the tire). The force and moment are expressed
    /// in global frame, as applied to the center of the associated wheel.
    virtual TerrainForce ReportTireForce(ChTerrain* terrain) const override;

    /// Add visualization assets for the rigid tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the rigid tire subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Return the number of vertices in the contact mesh.
    unsigned int GetNumVertices() const;

    /// Return the number of faces in the contact mesh.
    unsigned int GetNumTriangles() const;

    /// Get the contact mesh connectivity.
    const std::vector<ChVector<int>>& GetMeshConnectivity() const;

    /// Get the contact mesh vertices (in local frame).
    const std::vector<ChVector<>>& GetMeshVertices() const;

    /// Get the contact mesh vertex normals (in local frame).
    const std::vector<ChVector<>>& GetMeshNormals() const;

    /// Get the current state of the collision mesh.
    /// Mesh vertex positions and velocities are returned in the absolute frame.
    void GetMeshVertexStates(std::vector<ChVector<>>& pos,  ///< mesh vertex positions (expressed in absolute frame)
                             std::vector<ChVector<>>& vel   ///< mesh vertex velocities (expressed in absolute frame)
                             ) const;

  private:
    /// Get the tire force and moment.
    /// A ChRigidTire always returns zero force and moment since tire
    /// forces are automatically applied to the associated wheel through Chrono's
    /// frictional contact system.
    virtual TerrainForce GetTireForce() const override;

    /// Initialize this tire by associating it to the specified wheel.
    virtual void Initialize(std::shared_ptr<ChWheel> wheel) override;

    bool m_use_contact_mesh;         ///< flag indicating use of a contact mesh
    std::string m_contact_meshFile;  ///< name of the OBJ file for contact mesh
    double m_sweep_sphere_radius;    ///< radius of sweeping sphere for mesh contact

    std::shared_ptr<geometry::ChTriangleMeshConnected> m_trimesh;  ///< contact mesh

    std::shared_ptr<ChCylinderShape> m_cyl_shape;  ///< visualization cylinder asset
    std::shared_ptr<ChTexture> m_texture;          ///< visualization texture asset
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
