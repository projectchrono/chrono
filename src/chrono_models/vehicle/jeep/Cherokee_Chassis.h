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
// Authors: Radu Serban, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// Chassis class for Jeep Cherokee 1997
// Vehicle Parameters taken from SAE Paper 1999-01-0121
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#ifndef CHRONO_CHEROKEE_CHASSIS_H
#define CHRONO_CHEROKEE_CHASSIS_H

#include <string>

#include "chrono_vehicle/chassis/ChRigidChassis.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace jeep {

/// @addtogroup vehicle_models_cherokee
/// @{

/// Jeep Cherokee chassis subsystem.
class CH_MODELS_API Cherokee_Chassis : public ChRigidChassis {
  public:
    Cherokee_Chassis(const std::string& name,
                     bool fixed = false,
                     CollisionType chassis_collision_type = CollisionType::NONE);
    ~Cherokee_Chassis() {}

    /// Get the location (in the local frame of this chassis) of the connection to a rear chassis.
    virtual const ChVector3d GetLocalPosRearConnector() const override { return m_connector_rear_loc; }

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

  protected:
    virtual double GetBodyMass() const override { return m_body_mass; }
    virtual ChMatrix33<> GetBodyInertia() const override { return m_body_inertia; }
    virtual ChFrame<> GetBodyCOMFrame() const override { return ChFrame<>(m_body_COM_loc, QUNIT); }

    ChMatrix33<> m_body_inertia;

    static const double m_body_mass;
    static const ChVector3d m_body_inertiaXX;
    static const ChVector3d m_body_inertiaXY;
    static const ChVector3d m_body_COM_loc;
    static const ChVector3d m_connector_rear_loc;
    static const ChCoordsys<> m_driverCsys;
};

/// @} vehicle_models_cherokee

}  // namespace jeep
}  // end namespace vehicle
}  // end namespace chrono

#endif  // CHRONO_CHEROKEE_CHASSIS_H
