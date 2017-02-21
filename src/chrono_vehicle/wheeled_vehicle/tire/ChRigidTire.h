// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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

    /// Set coefficient of friction.
    /// The default value is 0.7
    void SetContactFrictionCoefficient(float friction_coefficient) { m_friction = friction_coefficient; }

    /// Set coefficient of restiturion.
    /// The default value is 0.1
    void SetContactRestitutionCoefficient(float restitution_coefficient) { m_restitution = restitution_coefficient; }

    /// Set contact material properties.
    /// These values are used to calculate contact material coefficients (if the containing
    /// system is so configured and if the DEM-P contact method is being used).
    /// The default values are: Y = 2e5 and nu = 0.3
    void SetContactMaterialProperties(float young_modulus,  ///< [in] Young's modulus of elasticity
                                      float poisson_ratio   ///< [in] Poisson ratio
                                      );

    /// Set contact material coefficients.
    /// These values are used directly to compute contact forces (if the containing system
    /// is so configured and if the DEM-P contact method is being used).
    /// The default values are: kn=2e5, gn=40, kt=2e5, gt=20
    void SetContactMaterialCoefficients(float kn,  ///< [in] normal contact stiffness
                                        float gn,  ///< [in] normal contact damping
                                        float kt,  ///< [in] tangential contact stiffness
                                        float gt   ///< [in] tangential contact damping
                                        );

    /// Set Wavefront OBJ file for contact mesh.
    void SetMeshFilename(const std::string& mesh_file,   ///< [in] name of Wavefront file
                         double sweep_sphere_radius = 0  ///< [in] radius of sweeping sphere
                         );

    /// Check whether or not this tire uses a contact mesh.
    bool UseContactMesh() const { return m_use_contact_mesh; }

    /// Get the tire width.
    virtual double GetWidth() const = 0;

    /// Get the tire force and moment.
    /// A ChRigidTire always returns zero forces and moments if the tire is
    /// simulated together with the associated vehicle (the tire forces are
    /// automatically applied to the associated wheel through Chrono's frictional
    /// contact system). If the tire is co-simulated, the tire force and moments
    /// encapsulate the tire-terrain forces (i.e. the resultant of all contact
    /// forces acting on the tire); in this case, the force and moment are expressed
    /// in global frame, as applied to the center of the associated wheel.
    virtual TireForce GetTireForce(bool cosim = false) const override;

    /// Initialize this tire system.
    /// This function creates the tire contact shape and attaches it to the
    /// associated wheel body.
    virtual void Initialize(std::shared_ptr<ChBody> wheel,  ///< handle to the associated wheel body
                            VehicleSide side                ///< left/right vehicle side
                            ) override;

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
    bool m_use_contact_mesh;         ///< flag indicating use of a contact mesh
    std::string m_contact_meshFile;  ///< name of the OBJ file for contact mesh
    double m_sweep_sphere_radius;    ///< radius of sweeping sphere for mesh contact

    float m_friction;
    float m_restitution;
    float m_young_modulus;
    float m_poisson_ratio;
    float m_kn;
    float m_gn;
    float m_kt;
    float m_gt;

    geometry::ChTriangleMeshConnected* m_trimesh;  ///< contact mesh

    std::shared_ptr<ChCylinderShape> m_cyl_shape;  ///< visualization cylinder asset
    std::shared_ptr<ChTexture> m_texture;          ///< visualization texture asset
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
