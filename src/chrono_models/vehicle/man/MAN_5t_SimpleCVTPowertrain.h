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

#ifndef MAN5T_SIMPLECVTPOWERTRAIN_H
#define MAN5T_SIMPLECVTPOWERTRAIN_H

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/powertrain/ChSimpleCVTPowertrain.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace man {

/// @addtogroup vehicle_models_man
/// @{

/// Simple MAN 5t powertrain subsystem (purely kinematic).
class CH_MODELS_API MAN_5t_SimpleCVTPowertrain : public ChSimpleCVTPowertrain {
  public:
    MAN_5t_SimpleCVTPowertrain(const std::string& name);

    ~MAN_5t_SimpleCVTPowertrain() {}

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

/// @} vehicle_models_man

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono

#endif
