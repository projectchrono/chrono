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
// Base class for a deformable tire (i.e. modeled with an FEA mesh)
//
// =============================================================================

#ifndef CH_DEFORMABLETIRE_H
#define CH_DEFORMABLETIRE_H

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMate.h"

#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAbase.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Base class for a deformable tire model.
class CH_VEHICLE_API ChDeformableTire : public ChTire {
  public:
    /// Type of the mesh contact surface.
    enum class ContactSurfaceType { NODE_CLOUD, TRIANGLE_MESH };

    /// Construct a deformable tire with the specified name.
    ChDeformableTire(const std::string& name);

    virtual ~ChDeformableTire();

    /// Set the type of contact surface.
    void SetContactSurfaceType(ContactSurfaceType type) { m_contact_type = type; }
    ContactSurfaceType GetContactSurfaceType() const { return m_contact_type; }

    /// Set radius of contact nodes.
    /// This value is relevant only for NODE_CLOUD contact surface type.
    void SetContactNodeRadius(double radius) { m_contact_node_radius = radius; }
    double GetContactNodeRadius() const { return m_contact_node_radius; }

    /// Set thickness of contact faces (radius of swept sphere).
    /// This value is relevant only for TRIANGLE_MESH contact surface type.
    void SetContactFaceThickness(double thickness) { m_contact_face_thickness = thickness; }
    double GetContactFaceThickness() const { return m_contact_face_thickness; }

    /// Get the tire contact material.
    /// Note that this is not set until after tire initialization.
    std::shared_ptr<ChMaterialSurfaceSMC> GetContactMaterial() const { return m_contact_mat; }

    /// Enable/disable tire pressure (default: true).
    void EnablePressure(bool val) { m_pressure_enabled = val; }
    bool IsPressureEnabled() const { return m_pressure_enabled; }

    /// Enable/disable tire contact (default: true).
    void EnableContact(bool val) { m_contact_enabled = val; }
    bool IsContactEnabled() const { return m_contact_enabled; }

    /// Enable/disable tire-rim connection (default: true).
    void EnableRimConnection(bool val) { m_connection_enabled = val; }
    bool IsRimConnectionEnabled() const { return m_connection_enabled; }

    /// Get a handle to the mesh visualization.
    fea::ChVisualizationFEAmesh* GetMeshVisualization() const { return m_visualization.get(); }

    /// Get the underlying FEA mesh.
    std::shared_ptr<fea::ChMesh> GetMesh() const { return m_mesh; }

    /// Get the mesh contact surface.
    /// If contact is not enabled, an empty shared pointer is returned.
    std::shared_ptr<fea::ChContactSurface> GetContactSurface() const;

    /// Get the load container associated with this tire.
    std::shared_ptr<ChLoadContainer> GetLoadContainer() const { return m_load_container; }

    /// Set the tire pressure.
    void SetPressure(double pressure) {
        assert(m_pressure > 0);
        m_pressure = pressure;
    }

    /// Get the rim radius (inner tire radius).
    virtual double GetRimRadius() const = 0;

    /// Report the tire force and moment.
    /// This generalized force encapsulates the tire-terrain forces, as well as the weight
    /// of the tire itself and is calculated as the resultant of all reaction forces and
    /// torques in the tire-wheel connections, as applied at the wheel body center of mass.
    /// The force and moment are expressed in the global frame.
    virtual TerrainForce ReportTireForce(ChTerrain* terrain) const override;

    /// Add visualization assets for the rigid tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override final;

    /// Remove visualization assets for the rigid tire subsystem.
    virtual void RemoveVisualizationAssets() override final;

  protected:
    /// Return the default tire pressure.
    virtual double GetDefaultPressure() const = 0;

    /// Return list of nodes connected to the rim.
    virtual std::vector<std::shared_ptr<fea::ChNodeFEAbase>> GetConnectedNodes() const = 0;

    /// Create the FEA nodes and elements.
    /// The wheel rotational axis is assumed to be the Y axis.
    virtual void CreateMesh(const ChFrameMoving<>& wheel_frame,  ///< [in] frame of associated wheel
                            VehicleSide side                     ///< [in] left/right vehicle side
                            ) = 0;

    /// Create the ChLoad for applying pressure to the tire.
    /// A derived class must create a load and add it to the underlying load container.
    virtual void CreatePressureLoad() = 0;

    /// Create the contact surface for the tire mesh.
    /// A derived class must create a contact surface and add it to the underlying mesh.
    virtual void CreateContactSurface() = 0;

    /// Create the tire-rim connections.
    /// A derived class must create the various constraints between the tire and the
    /// provided wheel body and add them to the underlying system.
    virtual void CreateRimConnections(std::shared_ptr<ChBody> wheel  ///< [in] associated wheel body
                                      ) = 0;

    /// Create the SMC contact material.
    virtual void CreateContactMaterial() = 0;

    std::shared_ptr<fea::ChMesh> m_mesh;                                ///< tire mesh
    std::shared_ptr<ChLoadContainer> m_load_container;                  ///< load container (for pressure load)
    std::vector<std::shared_ptr<fea::ChLinkPointFrame>> m_connections;  ///< tire-wheel point connections
    std::vector<std::shared_ptr<fea::ChLinkDirFrame>> m_connectionsD;   ///< tire-wheel direction connections
    std::vector<std::shared_ptr<ChLinkMateFix>> m_connectionsF;         ///< tire-wheel fix connection (point+rotation)

    bool m_connection_enabled;  ///< enable tire connections to rim
    bool m_pressure_enabled;    ///< enable internal tire pressure
    bool m_contact_enabled;     ///< enable tire-terrain contact

    double m_pressure;  ///< internal tire pressure

    ContactSurfaceType m_contact_type;  ///< type of contact surface model (node cloud or mesh)
    double m_contact_node_radius;       ///< node radius (for node cloud contact surface)
    double m_contact_face_thickness;    ///< face thickness (for mesh contact surface)

    std::shared_ptr<ChMaterialSurfaceSMC> m_contact_mat;           ///< tire contact material
    std::shared_ptr<fea::ChVisualizationFEAmesh> m_visualization;  ///< tire mesh visualization

    // The mass properties of a deformable tire are implicitly included through the FEA mesh.
    // No mass and inertia are added to the associated spindle body.
    virtual double GetAddedMass() const final { return 0; }
    virtual ChVector<> GetAddedInertia() const final { return ChVector<>(0, 0, 0); }

     /// Return the tire mass.
    virtual void InitializeInertiaProperties() override final;

    /// Return the tire moments of inertia (in the tire centroidal frame).
    virtual void UpdateInertiaProperties() override final;

    /// Initialize this tire by associating it to the specified wheel.
    virtual void Initialize(std::shared_ptr<ChWheel> wheel) override;

    /// Get the tire force and moment.
    /// A ChDeformableTire always returns zero forces and moments since tire forces
    /// are implicitly applied to the associated wheel through the tire-wheel connections.
    virtual TerrainForce GetTireForce() const override;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
