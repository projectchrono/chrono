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
// Base class for a deformable tire modeled with an FEA mesh
//
// =============================================================================

#ifndef CH_FEATIRE_H
#define CH_FEATIRE_H

#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLinkMate.h"

#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAbase.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Base class for a deformable tire modeled with an FEA mesh.
/// The tire is attached to the wheel rim through constraints between the rim body and FEA nodes.
class CH_VEHICLE_API ChFEATire : public ChDeformableTire {
  public:
    /// Construct an FEA tire with the specified name.
    ChFEATire(const std::string& name);

    virtual ~ChFEATire();

    /// Enable/disable tire-rim connection (default: true).
    void EnableRimConnection(bool val) { m_connection_enabled = val; }
    bool IsRimConnectionEnabled() const { return m_connection_enabled; }

    /// Attach an FEA visual shape for run-time visualization.
    void AddVisualShapeFEA(std::shared_ptr<ChVisualShapeFEA> shape);

    /// Get the underlying FEA mesh.
    std::shared_ptr<fea::ChMesh> GetMesh() const { return m_mesh; }

    /// Get the mesh contact surface.
    /// If contact is not enabled, an empty shared pointer is returned.
    virtual std::shared_ptr<fea::ChContactSurface> GetContactSurface() const override;

    /// Get a triangle surface mesh over the tire nodes.
    /// If the tire contact surface is a triangle mesh, it is returned. Otherwise, a triangle surface mesh is created from
    /// the boundary faces of the FEA mesh on the first call; this surface is not used for collision.
    /// Note that for elements with rotational nodes (e.g., Reissner shells) this surface has XYZRot triangles.
    virtual std::shared_ptr<fea::ChContactSurfaceMesh> GetSurfaceMesh() const override;

    /// Get the load container associated with this tire.
    std::shared_ptr<ChLoadContainer> GetLoadContainer() const { return m_load_container; }

    /// Report the tire force and moment.
    /// This generalized force encapsulates the tire-terrain forces, as well as the weight
    /// of the tire itself and is calculated as the resultant of all reaction forces and
    /// torques in the tire-wheel connections, as applied at the wheel body center of mass.
    /// The force and moment are expressed in the global frame.
    virtual TerrainForce ReportTireForce(ChTerrain* terrain) const override;

    /// Add visualization assets for the FEA tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the FEA tire subsystem.
    virtual void RemoveVisualizationAssets() override;

  protected:
    /// Tire profile definition.
    struct Profile {
        std::vector<double> t;  ///< independent parameter
        std::vector<double> x;  ///< x coordinate (radial direction)
        std::vector<double> y;  ///< y coordinate (transversal direction)
    };

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

    std::shared_ptr<fea::ChMesh> m_mesh;                                     ///< tire mesh
    std::shared_ptr<ChLoadContainer> m_load_container;                       ///< load container (for pressure load)
    std::vector<std::shared_ptr<fea::ChLinkNodeFrame>> m_connections;        ///< tire-wheel point connections
    std::vector<std::shared_ptr<fea::ChLinkNodeSlopeFrame>> m_connectionsD;  ///< tire-wheel direction connections
    std::vector<std::shared_ptr<ChLinkMateFix>> m_connectionsF;              ///< tire-wheel fix connection (point+rotation)

    bool m_connection_enabled;  ///< enable tire connections to rim

    std::vector<std::shared_ptr<ChVisualShapeFEA>> m_visFEA;  ///< tire mesh FEA visual shapes

    mutable std::shared_ptr<fea::ChContactSurfaceMesh> m_surface_mesh;  ///< surface mesh (if not the contact surface)

    /// Initialize the tire inertia properties (from the FEA mesh).
    virtual void InitializeInertiaProperties() override;

    /// Update the tire inertia properties (from the FEA mesh).
    virtual void UpdateInertiaProperties() override;

    /// Initialize this tire by associating it to the specified wheel.
    virtual void Initialize(std::shared_ptr<ChWheel> wheel) override;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
