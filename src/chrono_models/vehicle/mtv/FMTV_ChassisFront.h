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
// FMTV front chassis subsystem (common for MTV and LMTV trucks)
//
// =============================================================================

#ifndef FMTV_CHASSIS_FRONT_H
#define FMTV_CHASSIS_FRONT_H

#include "chrono_vehicle/chassis/ChRigidChassis.h"
#include "chrono_vehicle/chassis/ChChassisConnectorTorsion.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

/// @addtogroup vehicle_models_fmtv
/// @{

/// FMTV front chassis subsystem (common for MTV and LMTV trucks).
class CH_MODELS_API FMTV_ChassisFront : public ChRigidChassis {
  public:
    FMTV_ChassisFront(const std::string& name,
                      bool fixed = false,
                      CollisionType chassis_collision_type = CollisionType::NONE);
    ~FMTV_ChassisFront() {}

    /// Return the mass of the front chassis body.
    virtual double GetMass() const override { return m_mass; }

    /// Return the inertia tensor of the chassis body.
    virtual const ChMatrix33<>& GetInertia() const override { return m_inertia; }

    /// Get the location of the center of mass in the chassis frame.
    virtual const ChVector<>& GetLocalPosCOM() const override { return m_COM_loc; }

    /// Get the location (in the local frame of this chassis) of the connection to the rear chassis.
    virtual const chrono::ChVector<> GetLocalPosRearConnector() const override { return m_connector_loc; }

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

  protected:
    virtual void CreateContactMaterials(ChContactMethod contact_method) override;

    ChMatrix33<> m_inertia;

    static const double m_mass;
    static const ChVector<> m_inertiaXX;
    static const ChVector<> m_inertiaXY;
    static const ChVector<> m_COM_loc;
    static const chrono::ChVector<> m_connector_loc;
    static const ChCoordsys<> m_driverCsys;
};

/// @} vehicle_models_fmtv

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono

#endif
