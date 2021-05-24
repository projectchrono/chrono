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
// Simple CVT powertrain model for the Marder vehicle.
// - simple speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#ifndef MARDER_SIMPLECVTPOWERTRAIN_H
#define MARDER_SIMPLECVTPOWERTRAIN_H

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/powertrain/ChSimpleCVTPowertrain.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace marder {

/// @addtogroup vehicle_models_marder
/// @{

/// Simple CVT powertrain model for the M113 vehicle (purely kinematic).
class CH_MODELS_API Marder_SimpleCVTPowertrain : public ChSimpleCVTPowertrain {
  public:
    Marder_SimpleCVTPowertrain(const std::string& name);

    ~Marder_SimpleCVTPowertrain() {}

    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;
    virtual double GetMaxTorque() const override { return m_max_torque; }
    virtual double GetMaxPower() const override { return m_max_power; }
    virtual double GetMaxSpeed() const override { return m_max_speed; }

  private:
    static const double m_fwd_gear_ratio;  // forward gear ratio (single gear transmission)
    static const double m_rev_gear_ratio;  // reverse gear ratio
    static const double m_max_torque;      // maximum motor torque
    static const double m_max_power;       // maximum motor power
    static const double m_max_speed;       // maximum engine speed
};

/// @} vehicle_models_marder

}  // end namespace marder
}  // end namespace vehicle
}  // end namespace chrono

#endif
