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
// Vehicle model with an articulated chassis.
//
// =============================================================================

#ifndef ACV_VEHICLE_H
#define ACV_VEHICLE_H

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

class ACV_Vehicle : public chrono::vehicle::ChWheeledVehicle {
  public:
    ACV_Vehicle(const bool fixed, chrono::ChContactMethod contactMethod = chrono::ChContactMethod::NSC);

    ~ACV_Vehicle() {}

    virtual int GetNumberAxles() const override { return 2; }

    virtual double GetWheelbase() const override { return 1.0; }
    virtual double GetMinTurningRadius() const override { return 5.0; }
    virtual double GetMaxSteeringAngle() const override { return 0; }

    virtual void Initialize(const chrono::ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;
};

#endif
