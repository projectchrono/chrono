// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Simple powertrain model for the M113 vehicle.
// - simple speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#ifndef M113_SIMPLEPOWERTRAIN_H
#define M113_SIMPLEPOWERTRAIN_H

#include "chrono_vehicle/powertrain/ChSimplePowertrain.h"
#include "chrono_vehicle/ChVehicle.h"

#include "models/ChApiModels.h"

namespace m113 {

class CH_MODELS_API M113_SimplePowertrain : public chrono::vehicle::ChSimplePowertrain {
  public:
    M113_SimplePowertrain();

    ~M113_SimplePowertrain() {}

    virtual double GetForwardGearRatio() const override { return m_fwd_gear_ratio; }
    virtual double GetReverseGearRatio() const override { return m_rev_gear_ratio; }
    virtual double GetMaxTorque() const override { return m_max_torque; }
    virtual double GetMaxSpeed() const override { return m_max_speed; }

  private:
    static const double m_fwd_gear_ratio;  // forward gear ratio (single gear transmission)
    static const double m_rev_gear_ratio;  // reverse gear ratio
    static const double m_max_torque;      // maximum motor torque
    static const double m_max_speed;       // maximum motor speed
};

}  // end namespace m113

#endif
