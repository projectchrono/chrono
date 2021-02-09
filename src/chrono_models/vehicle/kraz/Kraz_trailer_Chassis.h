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
// Trailer chassis and connector for the Kraz trailer vehicle model.
//
// =============================================================================

#ifndef KRAZ_TRAILER_CHASSIS_H
#define KRAZ_TRAILER_CHASSIS_H

#include "chrono_vehicle/chassis/ChRigidChassis.h"
#include "chrono_vehicle/chassis/ChChassisConnectorHitch.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace kraz {

/// @addtogroup vehicle_models_kraz
/// @{

// -----------------------------------------------------------------------------

/// Kraz trailer chassis subsystem.
class CH_MODELS_API Kraz_trailer_Chassis : public ChRigidChassisRear {
  public:
    Kraz_trailer_Chassis(const std::string& name);
    ~Kraz_trailer_Chassis() {}

    /// Return the mass of the chassis body.
    virtual double GetMass() const override { return m_mass; }

    /// Return the inertia tensor of the chassis body.
    virtual const ChMatrix33<>& GetInertia() const override { return m_inertia; }

    /// Get the location of the center of mass in the chassis frame.
    virtual const ChVector<>& GetLocalPosCOM() const override { return m_COM_loc; }

    /// Get the location (in the local frame of this chassis) of the connection to the front chassis.
    virtual const ChVector<>& GetLocalPosFrontConnector() const override { return m_connector_loc; }

  private:
    chrono::ChMatrix33<> m_inertia;

    static const double m_mass;
    static const ChVector<> m_inertiaXX;
    static const ChVector<> m_inertiaXY;
    static const ChVector<> m_COM_loc;

    static const ChVector<> m_connector_loc;
};

// -----------------------------------------------------------------------------

/// Kraz tractor-trailer hitch connector subsystem.
class CH_MODELS_API Kraz_trailer_Connector : public ChChassisConnectorHitch {
  public:
    Kraz_trailer_Connector(const std::string& name) : ChChassisConnectorHitch(name) {}
    ~Kraz_trailer_Connector() {}
};


/// @} vehicle_models_kraz

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono

#endif
