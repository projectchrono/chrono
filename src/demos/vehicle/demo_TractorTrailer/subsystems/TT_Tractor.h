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
               chrono::ChMaterialSurface::ContactMethod contactMethod = chrono::ChMaterialSurface::NSC);

    ~TT_Tractor() {}

    virtual int GetNumberAxles() const override { return 2; }

    double GetSpringForce(const chrono::vehicle::WheelID& wheel_id) const;
    double GetSpringLength(const chrono::vehicle::WheelID& wheel_id) const;
    double GetSpringDeformation(const chrono::vehicle::WheelID& wheel_id) const;

    double GetShockForce(const chrono::vehicle::WheelID& wheel_id) const;
    double GetShockLength(const chrono::vehicle::WheelID& wheel_id) const;
    double GetShockVelocity(const chrono::vehicle::WheelID& wheel_id) const;

    virtual void Initialize(const chrono::ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

    // Log debugging information
    void LogHardpointLocations();  /// suspension hardpoints at design
    void DebugLog(int what);       /// shock forces and lengths, constraints, etc.

  private:
    chrono::vehicle::SuspensionType m_suspType;
};

#endif
