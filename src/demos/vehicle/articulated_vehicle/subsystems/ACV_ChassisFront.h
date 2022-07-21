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
// Front chassis subsystem for the articulated vehicle.
//
// =============================================================================

#ifndef ACV_CHASSIS_FRONT_H
#define ACV_CHASSIS_FRONT_H

#include "chrono_vehicle/chassis/ChRigidChassis.h"

class ACV_ChassisFront : public chrono::vehicle::ChRigidChassis {
  public:
    ACV_ChassisFront(const std::string& name, bool fixed = false);
    ~ACV_ChassisFront() {}
    
    /// Get the location (in the local frame of this chassis) of the connection to the rear chassis.
    virtual const chrono::ChVector<> GetLocalPosRearConnector() const override { return m_connector_loc; }

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual chrono::ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

  protected:
    virtual double GetBodyMass() const override { return m_body_mass; }
    virtual chrono::ChMatrix33<> GetBodyInertia() const override { return m_body_inertia; }
    virtual chrono::ChFrame<> GetBodyCOMFrame() const override { return chrono::ChFrame<>(m_body_COM_loc, chrono::QUNIT); }

    chrono::ChMatrix33<> m_body_inertia;

    static const double m_body_mass;
    static const chrono::ChVector<> m_body_inertiaXX;
    static const chrono::ChVector<> m_body_inertiaXY;
    static const chrono::ChVector<> m_body_COM_loc;
    static const chrono::ChCoordsys<> m_driverCsys;
    static const chrono::ChVector<> m_connector_loc;
};

#endif
