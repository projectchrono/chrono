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

    /// Initialize the chassis at the specified global position and orientation.
    virtual void Initialize(ChSystem* system,                ///< [in] containing system
                            const ChCoordsys<>& chassisPos,  ///< [in] absolute chassis position
                            double chassisFwdVel,            ///< [in] initial chassis forward velocity
                            int collision_family = 0         ///< [in] chassis collision family
                            ) override;

    /// Enable/disable contact for the chassis.
    /// This function controls contact of the chassis with all other collision shapes in the simulation. Must be called
    /// after initialization and has effect only if the derived object has defined some collision shapes.
    virtual void SetCollide(bool state) override { m_body->SetCollide(state); }

    /// Add visualization assets to this subsystem, for the specified visualization mode.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove all visualization assets from this subsystem.
    virtual void RemoveVisualizationAssets() override final;

  protected:
    ChVehicleGeometry m_geometry;  ///< collection of visualization and collision shapes

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

    /// Initialize the rear chassis relative to the specified front chassis.
    /// The orientation is set to be the same as that of the front chassis while the location is based on the connector
    /// position on the front and rear chassis.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,  ///< [in] front chassis
                            int collision_family = 0             ///< [in] chassis collision family
                            ) override;

    /// Add visualization assets to this subsystem, for the specified visualization mode.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove all visualization assets from this subsystem.
    virtual void RemoveVisualizationAssets() override final;

  protected:
    ChVehicleGeometry m_geometry;  ///< collection of visualization and collision shapes

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
