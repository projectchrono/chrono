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
// LMTV cargo truck (2.5 tons) rear chassis subsystems.
//
// =============================================================================

#ifndef LMTV_CHASSIS_H
#define LMTV_CHASSIS_H

#include "chrono_vehicle/chassis/ChRigidChassis.h"
#include "chrono_vehicle/chassis/ChChassisConnectorTorsion.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

/// @addtogroup vehicle_models_fmtv
/// @{

/// LMTV rear chassis subsystem.
class CH_MODELS_API LMTV_ChassisRear : public ChRigidChassisRear {
  public:
    LMTV_ChassisRear(const std::string& name, CollisionType chassis_collision_type = CollisionType::NONE);
    ~LMTV_ChassisRear() {}

    /// Get the location (in the local frame of this chassis) of the connection to the front chassis.
    virtual const ChVector<>& GetLocalPosFrontConnector() const override { return m_connector_loc; }

  protected:
    virtual double GetBodyMass() const override { return m_body_mass; }
    virtual ChMatrix33<> GetBodyInertia() const override { return m_body_inertia; }
    virtual ChFrame<> GetBodyCOMFrame() const override { return ChFrame<>(m_body_COM_loc, QUNIT); }

    ChMatrix33<> m_body_inertia;

    static const double m_body_mass;
    static const ChVector<> m_body_inertiaXX;
    static const ChVector<> m_body_inertiaXY;
    static const ChVector<> m_body_COM_loc;

    static const ChVector<> m_connector_loc;
};

// -----------------------------------------------------------------------------

/// LMTV torsion chassis connector.
class LMTV_ChassisConnector : public chrono::vehicle::ChChassisConnectorTorsion {
  public:
    LMTV_ChassisConnector(const std::string& name);
    ~LMTV_ChassisConnector() {}

    virtual double GetTorsionStiffness() const override { return m_torsion_stiffness; }

  private:
    static const double m_torsion_stiffness;
};

/// @} vehicle_models_fmtv

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono

#endif
