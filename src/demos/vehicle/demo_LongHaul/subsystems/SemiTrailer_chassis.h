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
// Trailer chassis and connector for the tractor-trailer vehicle model.
//
// =============================================================================

#ifndef SEMITRAILER_CHASSIS_H
#define SEMITRAILER_CHASSIS_H

#include "chrono_vehicle/chassis/ChRigidChassis.h"
#include "chrono_vehicle/chassis/ChChassisConnectorHitch.h"

// -----------------------------------------------------------------------------

class SemiTrailer_chassis : public chrono::vehicle::ChRigidChassisRear {
  public:
    SemiTrailer_chassis(const std::string& name);
    ~SemiTrailer_chassis() {}

    /// Return the mass of the chassis body.
    virtual double GetMass() const override { return m_mass; }

    /// Return the inertia tensor of the chassis body.
    virtual const chrono::ChMatrix33<>& GetInertia() const override { return m_inertia; }

    /// Get the location of the center of mass in the chassis frame.
    virtual const chrono::ChVector<>& GetLocalPosCOM() const override { return m_COM_loc; }

    /// Get the location (in the local frame of this chassis) of the connection to the front chassis.
    virtual const chrono::ChVector<>& GetLocalPosConnector() const override { return m_connector_loc; }

  private:
    chrono::ChMatrix33<> m_inertia;

    static const double m_mass;
    static const chrono::ChVector<> m_inertiaXX;
    static const chrono::ChVector<> m_inertiaXY;
    static const chrono::ChVector<> m_COM_loc;

    static const chrono::ChVector<> m_connector_loc;
};

// -----------------------------------------------------------------------------

class SemiTrailer_connector : public chrono::vehicle::ChChassisConnectorHitch {
  public:
    SemiTrailer_connector(const std::string& name) : ChChassisConnectorHitch(name) {}
    ~SemiTrailer_connector() {}
};

#endif
