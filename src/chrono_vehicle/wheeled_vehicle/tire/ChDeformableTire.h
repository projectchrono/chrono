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
// Base class for a deformable tire (FEA or multibody)
//
// =============================================================================

#ifndef CH_DEFORMABLETIRE_H
#define CH_DEFORMABLETIRE_H

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Base class for a deformable tire model.
/// A deformable tire is modeled with nodes that carry their own state, either as an FEA mesh (see ChFEATire) or as a
/// multibody model (see ChMBTire). Interaction with terrain is through a contact surface over the tire nodes.
class CH_VEHICLE_API ChDeformableTire : public ChTire {
  public:
    /// Construct a deformable tire with the specified name.
    ChDeformableTire(const std::string& name);

    virtual ~ChDeformableTire() {}

    /// Return the tire mass.
    virtual double GetTireMass() const override final;

    /// Return the tire moments of inertia (in the tire centroidal frame).
    virtual ChVector3d GetTireInertia() const override final;

    /// Get the tire contact material.
    /// Note that this is not set until after tire initialization.
    std::shared_ptr<ChContactMaterialSMC> GetContactMaterial() const { return m_contact_mat; }

    /// Enable/disable tire pressure (default: true).
    void EnablePressure(bool val) { m_pressure_enabled = val; }
    bool IsPressureEnabled() const { return m_pressure_enabled; }

    /// Enable/disable tire contact (default: true).
    void EnableContact(bool val) { m_contact_enabled = val; }
    bool IsContactEnabled() const { return m_contact_enabled; }

    /// Get the tire contact surface.
    /// If contact is not enabled or the tire is not initialized, an empty shared pointer is returned.
    virtual std::shared_ptr<fea::ChContactSurface> GetContactSurface() const = 0;

    /// Get a triangle surface mesh over the tire nodes.
    /// This surface is used for interaction with terrain models that require a triangulated tire surface (e.g., CRM
    /// terrain). If the tire contact surface is a triangle mesh, it is returned. Otherwise (node cloud contact surface or
    /// contact disabled), a triangle surface mesh is created on the first call; this surface is not used for collision.
    /// The face normals point outward from the tire (as assumed by the INWARD BCE pattern for CRM terrain).
    /// An empty shared pointer is returned if the tire is not initialized.
    virtual std::shared_ptr<fea::ChContactSurfaceMesh> GetSurfaceMesh() const = 0;

    /// Get the rim radius (inner tire radius).
    //// TODO: consider removing
    virtual double GetRimRadius() const = 0;

    /// Get the tire force and moment expressed in the tire frame.
    /// Currently *NOT IMPLEMENTED*.
    virtual TerrainForce ReportTireForceLocal(ChTerrain* terrain, ChCoordsys<>& tire_frame) const override;

  protected:
    /// Return the default tire pressure.
    virtual double GetDefaultPressure() const = 0;

    /// Create the SMC contact material.
    virtual void CreateContactMaterial() = 0;

    bool m_pressure_enabled;  ///< enable internal tire pressure
    bool m_contact_enabled;   ///< enable tire-terrain contact

    std::shared_ptr<ChContactMaterialSMC> m_contact_mat;  ///< tire contact material

    // The mass properties of a deformable tire are implicitly included through the tire nodes.
    // No mass and inertia are added to the associated spindle body.
    virtual double GetAddedMass() const override final { return 0; }
    virtual ChVector3d GetAddedInertia() const override final { return ChVector3d(0, 0, 0); }

    /// Initialize this tire by associating it to the specified wheel.
    /// A derived class must call this base class implementation first.
    virtual void Initialize(std::shared_ptr<ChWheel> wheel) override;

    /// Get the tire force and moment.
    /// A ChDeformableTire always returns zero forces and moments since tire forces
    /// are implicitly applied to the associated wheel through the tire-wheel connections.
    virtual TerrainForce GetTireForce() const override final;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
