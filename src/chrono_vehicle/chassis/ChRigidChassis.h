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

    /// Enable/disable contact for the chassis. This function controls contact of
    /// the chassis with all other collision shapes in the simulation.
    virtual void SetCollide(bool state) override { m_body->SetCollide(state); }

    /// Initialize the chassis at the specified global position and orientation.
    virtual void Initialize(ChSystem* system,                ///< [in] containing system
                            const ChCoordsys<>& chassisPos,  ///< [in] absolute chassis position
                            double chassisFwdVel,            ///< [in] initial chassis forward velocity
                            int collision_family = 0         ///< [in] chassis collision family
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
    std::vector<std::string> m_coll_mesh_names;

    bool m_has_primitives;
    std::vector<BoxShape> m_vis_boxes;
    std::vector<SphereShape> m_vis_spheres;
    std::vector<CylinderShape> m_vis_cylinders;

    bool m_has_mesh;
    std::string m_vis_mesh_name;
    std::string m_vis_mesh_file;
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
