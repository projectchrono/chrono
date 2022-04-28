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
// Authors: Radu Serban, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// FEDA chassis subsystem.
//
// =============================================================================

#ifndef FEDA_CHASSIS_H
#define FEDA_CHASSIS_H

#include <string>

#include "chrono_vehicle/chassis/ChRigidChassis.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace feda {

/// @addtogroup vehicle_models_feda
/// @{

/// FEDA chassis subsystem.
class CH_MODELS_API FEDA_Chassis : public ChRigidChassis {
  public:
    FEDA_Chassis(const std::string& name,
                 bool fixed = false,
                 CollisionType chassis_collision_type = CollisionType::NONE);
    ~FEDA_Chassis() {}

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

  protected:
    virtual double GetBodyMass() const override { return m_body_mass; }
    virtual ChMatrix33<> GetBodyInertia() const override { return m_body_inertia; }
    virtual ChFrame<> GetBodyCOMFrame() const override { return ChFrame<>(m_body_COM_loc, QUNIT); }

    virtual void CreateContactMaterials(ChContactMethod contact_method) override;

    ChMatrix33<> m_body_inertia;

    static const double m_body_mass;
    static const ChVector<> m_body_inertiaXX;
    static const ChVector<> m_body_inertiaXY;
    static const ChVector<> m_body_COM_loc;
    static const ChCoordsys<> m_driverCsys;
};

/// @} vehicle_models_feda

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono

#endif
