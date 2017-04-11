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
// Template for a rigid-body chassis vehicle subsystem.
//
// =============================================================================

#ifndef CH_RIGID_CHASSIS_H
#define CH_RIGID_CHASSIS_H

#include <vector>

#include "chrono_vehicle/ChChassis.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Template for a rigid-body chassis vehicle subsystem.
class CH_VEHICLE_API ChRigidChassis : public ChChassis {
  public:
    /// Construct a vehicle subsystem with the specified name.
    ChRigidChassis(const std::string& name,  ///< [in] name of the subsystem
                   bool fixed = false        ///< [in] is the chassis body fixed to ground?
                   );

    virtual ~ChRigidChassis() {}

    /// Specifies whether or not collision shapes were defined.
    bool HasCollision() const { return m_has_collision; }

    /// Specifies whether or not visualization primitives were defined.
    bool HasPrimitives() const { return m_has_primitives; }

    /// Specifies whether or not a visualization mesh was defined.
    bool HasMesh() const { return m_has_mesh; }

    /// Get the name of the Wavefront file with chassis visualization mesh.
    /// An empty string is returned if no mesh was specified.
    const std::string& GetMeshFilename() const { return m_vis_mesh_file; }

    /// Get the name of the chassis visualization mesh asset.
    /// An empty string is returned if no mesh was specified.
    const std::string& GetMeshName() const { return m_vis_mesh_name; }

    /// Set coefficient of friction.
    /// The default value is 0.7
    void SetContactFrictionCoefficient(float friction_coefficient) { m_friction = friction_coefficient; }

    /// Set coefficient of restitution.
    /// The default value is 0.1
    void SetContactRestitutionCoefficient(float restitution_coefficient) { m_restitution = restitution_coefficient; }

    /// Set contact material properties.
    /// These values are used to calculate contact material coefficients (if the containing
    /// system is so configured and if the DEM-P contact method is being used).
    /// The default values are: Y = 1e8 and nu = 0.3
    void SetContactMaterialProperties(float young_modulus,  ///< [in] Young's modulus of elasticity
                                      float poisson_ratio   ///< [in] Poisson ratio
                                      );

    /// Get coefficient of friction for contact material.
    float GetCoefficientFriction() const { return m_friction; }
    /// Get coefficient of restitution for contact material.
    float GetCoefficientRestitution() const { return m_restitution; }
    /// Get Young's modulus of elasticity for contact material.
    float GetYoungModulus() const { return m_young_modulus; }
    /// Get Poisson ratio for contact material.
    float GetPoissonRatio() const { return m_poisson_ratio; }
    /// Get normal stiffness coefficient for contact material.
    float GetKn() const { return m_kn; }
    /// Get tangential stiffness coefficient for contact material.
    float GetKt() const { return m_kt; }
    /// Get normal viscous damping coefficient for contact material.
    float GetGn() const { return m_gn; }
    /// Get tangential viscous damping coefficient for contact material.
    float GetGt() const { return m_gt; }

    /// Enable/disable contact for the chassis.
    virtual void SetCollide(bool state) override { m_body->SetCollide(state); }

    /// Set contact material coefficients.
    /// These values are used directly to compute contact forces (if the containing system
    /// is so configured and if the DEM-P contact method is being used).
    /// The default values are: kn=2e5, gn=40, kt=2e5, gt=20
    void SetContactMaterialCoefficients(float kn,  ///< [in] normal contact stiffness
                                        float gn,  ///< [in] normal contact damping
                                        float kt,  ///< [in] tangential contact stiffness
                                        float gt   ///< [in] tangential contact damping
                                        );

    /// Initialize the chassis at the specified global position and orientation.
    virtual void Initialize(ChSystem* system,                ///< [in] containing system
                            const ChCoordsys<>& chassisPos,  ///< [in] absolute chassis position
                            double chassisFwdVel             ///< [in] initial chassis forward velocity
                            ) override;

    /// Add visualization assets to this subsystem, for the specified visualization mode.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove all visualization assets from this subsystem.
    virtual void RemoveVisualizationAssets() override final;

  protected:
    struct BoxShape {
        BoxShape(const ChVector<>& pos, const ChQuaternion<>& rot, const ChVector<>& dims)
            : m_pos(pos), m_rot(rot), m_dims(dims) {}
        ChVector<> m_pos;
        ChQuaternion<> m_rot;
        ChVector<> m_dims;
    };

    struct SphereShape {
        SphereShape(const ChVector<>& pos, double radius) : m_pos(pos), m_radius(radius) {}
        ChVector<> m_pos;
        double m_radius;
    };

    struct CylinderShape {
        CylinderShape(const ChVector<>& pos, const ChQuaternion<>& rot, double radius, double length)
            : m_pos(pos), m_rot(rot), m_radius(radius), m_length(length) {}
        ChVector<> m_pos;
        ChQuaternion<> m_rot;
        double m_radius;
        double m_length;
    };

    bool m_has_collision;
    std::vector<BoxShape> m_coll_boxes;
    std::vector<SphereShape> m_coll_spheres;
    std::vector<CylinderShape> m_coll_cylinders;

    bool m_has_primitives;
    std::vector<BoxShape> m_vis_boxes;
    std::vector<SphereShape> m_vis_spheres;
    std::vector<CylinderShape> m_vis_cylinders;

    bool m_has_mesh;
    std::string m_vis_mesh_name;
    std::string m_vis_mesh_file;

    float m_friction;       ///< contact coefficient of friction
    float m_restitution;    ///< contact coefficient of restitution
    float m_young_modulus;  ///< contact material Young modulus
    float m_poisson_ratio;  ///< contact material Poisson ratio
    float m_kn;             ///< normal contact stiffness
    float m_gn;             ///< normal contact damping
    float m_kt;             ///< tangential contact stiffness
    float m_gt;             ///< tangential contact damping
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
