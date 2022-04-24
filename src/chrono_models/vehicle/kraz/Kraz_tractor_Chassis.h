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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Chassis subsystem for the semi tractor vehicle.
//
// =============================================================================

#ifndef KRAZ_TRACTOR_CHASSIS_H
#define KRAZ_TRACTOR_CHASSIS_H

#include "chrono_vehicle/chassis/ChRigidChassis.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace kraz {

/// @addtogroup vehicle_models_kraz
/// @{

/// Kraz tractor chassis subsystem.
class CH_MODELS_API Kraz_tractor_Chassis : public ChRigidChassis {
  public:
    Kraz_tractor_Chassis(const std::string& name);
    ~Kraz_tractor_Chassis() {}

    /// Get the location (in the local frame of this chassis) of the connection to the rear chassis.
    virtual const ChVector<> GetLocalPosRearConnector() const override { return m_connector_loc; }

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

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
    static const ChCoordsys<> m_driverCsys;
};

#endif

/// @} vehicle_models_kraz

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
