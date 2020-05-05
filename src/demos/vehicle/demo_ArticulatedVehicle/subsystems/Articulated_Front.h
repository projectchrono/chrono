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
// Front of the articulated vehicle model. Implemented as a ChWheeledVehicle.
//
// =============================================================================

#ifndef ARTICULATED_FRONT_H
#define ARTICULATED_FRONT_H

#include "chrono_vehicle/chassis/ChRigidChassis.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

class Articulated_Chassis : public chrono::vehicle::ChRigidChassis {
  public:
    Articulated_Chassis(const std::string& name, bool fixed = false);

    /// Return the mass of the chassis body.
    virtual double GetMass() const override { return m_mass; }

    /// Return the inertia tensor of the chassis body.
    virtual const chrono::ChMatrix33<>& GetInertia() const override { return m_inertia; }

    /// Get the location of the center of mass in the chassis frame.
    virtual const chrono::ChVector<>& GetLocalPosCOM() const override { return m_COM_loc; }

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual chrono::ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

    /// Return offset to rear connection.
    const chrono::ChVector<>& GetLocalConnectionPoint() const { return m_offset; }
    chrono::ChVector<> GetConnectionPoint() const;

  private:
    static const chrono::ChVector<> m_offset;

    chrono::ChMatrix33<> m_inertia;

    static const double m_mass;
    static const chrono::ChVector<> m_inertiaXX;
    static const chrono::ChVector<> m_inertiaXY;
    static const chrono::ChVector<> m_COM_loc;
    static const chrono::ChCoordsys<> m_driverCsys;
};

class Articulated_Front : public chrono::vehicle::ChWheeledVehicle {
  public:
    Articulated_Front(const bool fixed = false, chrono::ChContactMethod contactMethod = chrono::ChContactMethod::NSC);

    ~Articulated_Front() {}

    virtual int GetNumberAxles() const override { return 1; }

    virtual double GetWheelbase() const override { return 1.0; }
    virtual double GetMinTurningRadius() const override { return 5.0; }
    virtual double GetMaxSteeringAngle() const override { return 0.0; }

    virtual void Initialize(const chrono::ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;
};

#endif
