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

    /// Return the mass of the chassis body.
    virtual double GetMass() const override { return m_mass; }

    /// Return the inertia tensor of the chassis body.
    virtual const chrono::ChMatrix33<>& GetInertia() const override { return m_inertia; }

    /// Get the location of the center of mass in the chassis frame.
    virtual const chrono::ChVector<>& GetLocalPosCOM() const override { return m_COM_loc; }

    /// Get the location (in the local frame of this chassis) of the connection to the front chassis.
    virtual const chrono::ChVector<>& GetLocalPosFrontConnector() const override { return m_connector_loc; }

  protected:
    chrono::ChMatrix33<> m_inertia;

    static const double m_mass;
    static const chrono::ChVector<> m_inertiaXX;
    static const chrono::ChVector<> m_inertiaXY;
    static const chrono::ChVector<> m_COM_loc;
    static const chrono::ChVector<> m_connector_loc;
};


#endif
