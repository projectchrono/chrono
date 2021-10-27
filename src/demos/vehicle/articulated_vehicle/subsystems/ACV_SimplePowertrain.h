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
// Generic simple powertrain model for a vehicle.
// - trivial speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#ifndef ACV_SIMPLEPOWERTRAIN_H
#define ACV_SIMPLEPOWERTRAIN_H

#include "chrono_vehicle/powertrain/ChSimplePowertrain.h"

class ACV_SimplePowertrain : public chrono::vehicle::ChSimplePowertrain {
  public:
    ACV_SimplePowertrain(const std::string& name);
    ~ACV_SimplePowertrain() {}

    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;
    virtual double GetMaxTorque() const override { return m_max_torque; }
    virtual double GetMaxSpeed() const override { return m_max_speed; }

  private:
    static const double m_max_torque;
    static const double m_max_speed;
    static const double m_fwd_gear_ratio;
    static const double m_rev_gear_ratio;
};

#endif
