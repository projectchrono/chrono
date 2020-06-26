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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// MTV cargo truck (5 tons) rear chassis subsystems.
//
// =============================================================================

#ifndef MTV_CHASSIS_REAR_H
#define MTV_CHASSIS_REAR_H

#include "chrono_vehicle/chassis/ChRigidChassis.h"
#include "chrono_vehicle/chassis/ChChassisConnectorTorsion.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

/// @addtogroup vehicle_models_fmtv
/// @{

/// MTV cargo truck (5 tons) rear chassis subsystems.
class CH_MODELS_API MTV_ChassisRear : public ChRigidChassisRear {
  public:
    MTV_ChassisRear(const std::string& name, ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE);
    ~MTV_ChassisRear() {}

    /// Return the mass of the rear chassis body.
    virtual double GetMass() const override { return m_mass; }

    /// Return the inertia tensor of the chassis body.
    virtual const ChMatrix33<>& GetInertia() const override { return m_inertia; }

    /// Get the location of the center of mass in the chassis frame.
    virtual const ChVector<>& GetLocalPosCOM() const override { return m_COM_loc; }

    /// Get the location (in the local frame of this chassis) of the connection to the front chassis.
    virtual const ChVector<>& GetLocalPosConnector() const override { return m_connector_loc; }

  protected:
    virtual void CreateContactMaterials(ChContactMethod contact_method) override;

    ChMatrix33<> m_inertia;

    static const double m_mass;
    static const ChVector<> m_inertiaXX;
    static const ChVector<> m_inertiaXY;
    static const ChVector<> m_COM_loc;

    static const ChVector<> m_connector_loc;
};

// -----------------------------------------------------------------------------

/// MTV torsion chassis connector.
class MTV_ChassisConnector : public chrono::vehicle::ChChassisConnectorTorsion {
  public:
    MTV_ChassisConnector(const std::string& name);
    ~MTV_ChassisConnector() {}

    virtual double GetTorsionStiffness() const override { return m_torsion_stiffness; }

  private:
    static const double m_torsion_stiffness;
};

/// @} vehicle_models_fmtv

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono

#endif
