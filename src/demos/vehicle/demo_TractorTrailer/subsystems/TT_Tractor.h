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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz, Alessandro Tasora
// =============================================================================
//
// Tractor for the tractor-trailer vehicle model.
// Can be constructed either with solid-axle or with multi-link suspensions.
// Always uses a articulated rack-pinion steering and a 2WD driveline model.
//
// =============================================================================

#ifndef TT_TRACTOR_H
#define TT_TRACTOR_H

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

class TT_Tractor : public chrono::vehicle::ChWheeledVehicle {
  public:
    TT_Tractor(const bool fixed,
               chrono::vehicle::SuspensionType suspType,
               chrono::ChContactMethod contactMethod = chrono::ChContactMethod::NSC);

    ~TT_Tractor() {}

    virtual int GetNumberAxles() const override { return 2; }

    virtual double GetWheelbase() const override { return 3.38; }
    virtual double GetMinTurningRadius() const override { return 7.7; }
    virtual double GetMaxSteeringAngle() const override { return 30 * chrono::CH_C_DEG_TO_RAD; }

    double GetSpringForce(int axle, chrono::vehicle::VehicleSide side) const;
    double GetSpringLength(int axle, chrono::vehicle::VehicleSide side) const;
    double GetSpringDeformation(int axle, chrono::vehicle::VehicleSide side) const;

    double GetShockForce(int axle, chrono::vehicle::VehicleSide side) const;
    double GetShockLength(int axle, chrono::vehicle::VehicleSide side) const;
    double GetShockVelocity(int axle, chrono::vehicle::VehicleSide side) const;

    virtual void Initialize(const chrono::ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

    // Log debugging information
    void LogHardpointLocations();  /// suspension hardpoints at design
    void DebugLog(int what);       /// shock forces and lengths, constraints, etc.

  private:
    chrono::vehicle::SuspensionType m_suspType;
};

#endif
