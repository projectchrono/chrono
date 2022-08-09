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
// Rear chassis subsystem for the articulated vehicle.
//
// =============================================================================

#ifndef ARTICULATED_CHASSIS_REAR_H
#define ARTICULATED_CHASSIS_REAR_H

#include "chrono_vehicle/chassis/ChRigidChassis.h"

class ACV_ChassisRear : public chrono::vehicle::ChRigidChassisRear {
  public:
    ACV_ChassisRear(const std::string& name);
    ~ACV_ChassisRear() {}

    /// Get the location (in the local frame of this chassis) of the connection to the front chassis.
    virtual const chrono::ChVector<>& GetLocalPosFrontConnector() const override { return m_connector_loc; }

  protected:
    virtual double GetBodyMass() const override { return m_body_mass; }
    virtual chrono::ChMatrix33<> GetBodyInertia() const override { return m_body_inertia; }
    virtual chrono::ChFrame<> GetBodyCOMFrame() const override { return chrono::ChFrame<>(m_body_COM_loc, chrono::QUNIT); }

    chrono::ChMatrix33<> m_body_inertia;

    static const double m_body_mass;
    static const chrono::ChVector<> m_body_inertiaXX;
    static const chrono::ChVector<> m_body_inertiaXY;
    static const chrono::ChVector<> m_body_COM_loc;
    static const chrono::ChVector<> m_connector_loc;
};


#endif
