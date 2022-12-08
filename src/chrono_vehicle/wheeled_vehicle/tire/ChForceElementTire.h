// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
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
// Base class for a force element tire model
//
// =============================================================================

#ifndef CH_FORCEELEMENT_TIRE_H
#define CH_FORCEELEMENT_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Base class for a force lement tire model.
class CH_VEHICLE_API ChForceElementTire : public ChTire {
  public:
    virtual ~ChForceElementTire() {}

    /// Report the tire force and moment.
    virtual TerrainForce ReportTireForce(ChTerrain* terrain) const override;

  protected:
    struct ContactData {
        bool in_contact;      // true if disc in contact with terrain
        ChCoordsys<> frame;   // contact frame (x: long, y: lat, z: normal)
        ChVector<> vel;       // relative velocity expressed in contact frame
        double normal_force;  // magnitude of normal contact force
        double depth;         // penetration depth
    };

    /// Construct a tire with the specified name.
    ChForceElementTire(const std::string& name);

    /// Return the tire mass.
    virtual double GetTireMass() const = 0;

    /// Return the tire moments of inertia (in the tire centroidal frame).
    virtual ChVector<> GetTireInertia() const = 0;

    /// Return the vertical tire stiffness contribution to the normal force.
    virtual double GetNormalStiffnessForce(double depth) const = 0;

    /// Return the vertical tire damping contribution to the normal force.
    virtual double GetNormalDampingForce(double depth, double velocity) const = 0;

    /// Get the tire force and moment.
    /// This represents the output from this tire system that is passed to the vehicle system.  Typically, the vehicle
    /// subsystem will pass the tire force to the appropriate suspension subsystem which applies it as an external force
    /// one the wheel body.
    virtual TerrainForce GetTireForce() const override;

    ContactData m_data;             ///< tire-terrain collision information
    TerrainForce m_tireforce;       ///< tire forces (in tire contact frame)
    ChFunction_Recorder m_areaDep;  // lookup table for estimation of penetration depth from intersection area

  private:
    virtual void InitializeInertiaProperties() override final;
    virtual void UpdateInertiaProperties() override final;

    virtual double GetAddedMass() const override final;
    virtual ChVector<> GetAddedInertia() const override final;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
