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
// Template for a rigid-body chassis vehicle subsystem.
//
// =============================================================================

#ifndef CH_RIGID_CHASSIS_H
#define CH_RIGID_CHASSIS_H

#include <vector>

#include "chrono_vehicle/ChChassis.h"
#include "chrono/assets/ChColor.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

// -----------------------------------------------------------------------------

/// Utility class defining geometry (visualization and collision) and contact materials for a rigid chassis.
class CH_VEHICLE_API ChRigidChassisGeometry {
  public:
    ChRigidChassisGeometry();

    struct BoxShape {
        BoxShape(const ChVector<>& pos, const ChQuaternion<>& rot, const ChVector<>& dims, int matID = -1)
            : m_pos(pos), m_rot(rot), m_dims(dims), m_matID(matID) {}
        ChVector<> m_pos;
        ChQuaternion<> m_rot;
        ChVector<> m_dims;
        int m_matID;
    };

    struct SphereShape {
        SphereShape(const ChVector<>& pos, double radius, int matID = -1)
            : m_pos(pos), m_radius(radius), m_matID(matID) {}
        ChVector<> m_pos;
        double m_radius;
        int m_matID;
    };

    struct CylinderShape {
        CylinderShape(const ChVector<>& pos, const ChQuaternion<>& rot, double radius, double length, int matID = -1)
            : m_pos(pos), m_rot(rot), m_radius(radius), m_length(length), m_matID(matID) {}
        ChVector<> m_pos;
        ChQuaternion<> m_rot;
        double m_radius;
        double m_length;
        int m_matID;
    };

    struct ConvexHullsShape {
        ConvexHullsShape(const std::string& filename, int matID = -1) : m_filename(filename), m_matID(matID) {}
        std::string m_filename;
        int m_matID;
    };

    bool m_has_collision;
    std::vector<std::shared_ptr<ChMaterialSurface>> m_materials;
    std::vector<BoxShape> m_coll_boxes;
    std::vector<SphereShape> m_coll_spheres;
    std::vector<CylinderShape> m_coll_cylinders;
    std::vector<ConvexHullsShape> m_coll_hulls;

    bool m_has_primitives;
    std::vector<BoxShape> m_vis_boxes;
    std::vector<SphereShape> m_vis_spheres;
    std::vector<CylinderShape> m_vis_cylinders;
    ChColor m_color;

    bool m_has_mesh;
    std::string m_vis_mesh_file;

    void AddVisualizationAssets(std::shared_ptr<ChBodyAuxRef> body, VisualizationType vis);
    void AddCollisionShapes(std::shared_ptr<ChBodyAuxRef> body, int collision_family);
};

// -----------------------------------------------------------------------------

/// Template for a rigid-body main chassis vehicle subsystem.
class CH_VEHICLE_API ChRigidChassis : public ChChassis {
  public:
    /// Construct a vehicle subsystem with the specified name.
    ChRigidChassis(const std::string& name,  ///< [in] name of the subsystem
                   bool fixed = false        ///< [in] is the chassis body fixed to ground?
    );

    virtual ~ChRigidChassis() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "RigidChassis"; }

    /// Specifies whether or not collision shapes were defined.
    bool HasCollision() const { return m_geometry.m_has_collision; }

    /// Specifies whether or not visualization primitives were defined.
    bool HasPrimitives() const { return m_geometry.m_has_primitives; }

    /// Specifies whether or not a visualization mesh was defined.
    bool HasMesh() const { return m_geometry.m_has_mesh; }

    /// Get the name of the Wavefront file with chassis visualization mesh.
    /// An empty string is returned if no mesh was specified.
    const std::string& GetMeshFilename() const { return m_geometry.m_vis_mesh_file; }

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
    ChRigidChassisGeometry m_geometry;

    /// Load contact materials. A derived class must implement this only if it sets m_has_collision to 'true' and should
    /// use contact materials consistent with the specified contact method.
    virtual void CreateContactMaterials(ChContactMethod contact_method) {}

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;
};

// -----------------------------------------------------------------------------

/// Template for a rigid-body rear chassis vehicle subsystem.
class CH_VEHICLE_API ChRigidChassisRear : public ChChassisRear {
  public:
    /// Construct a vehicle subsystem with the specified name.
    ChRigidChassisRear(const std::string& name);

    virtual ~ChRigidChassisRear() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "RigidChassisRear"; }

    /// Specifies whether or not collision shapes were defined.
    bool HasCollision() const { return m_geometry.m_has_collision; }

    /// Specifies whether or not visualization primitives were defined.
    bool HasPrimitives() const { return m_geometry.m_has_primitives; }

    /// Specifies whether or not a visualization mesh was defined.
    bool HasMesh() const { return m_geometry.m_has_mesh; }

    /// Get the name of the Wavefront file with chassis visualization mesh.
    /// An empty string is returned if no mesh was specified.
    const std::string& GetMeshFilename() const { return m_geometry.m_vis_mesh_file; }

    /// Enable/disable contact for the chassis. This function controls contact of
    /// the chassis with all other collision shapes in the simulation.
    virtual void SetCollide(bool state) override { m_body->SetCollide(state); }

    /// Initialize the rear chassis at the specified position
    /// (relative to and expressed in the reference frame of the front chassis).
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,  ///< [in] front chassis
                            const ChVector<>& location,          ///< [in] location relative to front chassis frame
                            int collision_family = 0             ///< [in] chassis collision family
                            ) override;

    /// Add visualization assets to this subsystem, for the specified visualization mode.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove all visualization assets from this subsystem.
    virtual void RemoveVisualizationAssets() override final;

  protected:
    ChRigidChassisGeometry m_geometry;

    /// Load contact materials. A derived class must implement this only if it sets m_has_collision to 'true' and should
    /// use contact materials consistent with the specified contact method.
    virtual void CreateContactMaterials(ChContactMethod contact_method) {}

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
