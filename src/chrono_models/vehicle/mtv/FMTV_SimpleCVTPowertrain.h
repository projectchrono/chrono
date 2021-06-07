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
// Simple powertrain model for the FMTV vehicles.
// - hyperbolical speed-torque curve (CVT)
// - no torque converter
// - no transmission box
//
// =============================================================================

#ifndef SMTV_SIMPLECVTPOWERTRAIN_H
#define SMTV_SIMPLECVTPOWERTRAIN_H

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/powertrain/ChSimpleCVTPowertrain.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

/// @addtogroup vehicle_models_fmtv
/// @{

/// Simple powertrain subsystem (purely kinematic) for the FMTV vehicles.
class CH_MODELS_API FMTV_SimpleCVTPowertrain : public ChSimpleCVTPowertrain {
  public:
    FMTV_SimpleCVTPowertrain(const std::string& name);

    ~FMTV_SimpleCVTPowertrain() {}

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

/// @} vehicle_models_fmtv

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono

#endif
