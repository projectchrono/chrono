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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Simple powertrain model for the HMMWV vehicle.
// - hyperbolical speed-torque curve (CVT)
// - no torque converter
// - no transmission box
//
// =============================================================================

#ifndef HMMWV_SIMPLECVTPOWERTRAIN_H
#define HMMWV_SIMPLECVTPOWERTRAIN_H

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/powertrain/ChSimpleCVTPowertrain.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Simple HMMWV powertrain subsystem (purely kinematic).
class CH_MODELS_API HMMWV_SimpleCVTPowertrain : public ChSimpleCVTPowertrain {
  public:
    HMMWV_SimpleCVTPowertrain(const std::string& name);

    ~HMMWV_SimpleCVTPowertrain() {}

    virtual double GetForwardGearRatio() const override { return m_fwd_gear_ratio; }
    virtual double GetReverseGearRatio() const override { return m_rev_gear_ratio; }
    virtual double GetMaxTorque() const override { return m_max_torque; }
    virtual double GetMaxPower() const override { return m_max_power; }
    virtual double GetCriticalSpeed() const override { return m_critical_speed; }

  private:
    static const double m_fwd_gear_ratio;  // forward gear ratio (single gear transmission)
    static const double m_rev_gear_ratio;  // reverse gear ratio
    static const double m_max_torque;      // maximum motor torque
    static const double m_max_power;       // maximum motor power
    static const double m_critical_speed;  // critical motor speed for torque limiting
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
