// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
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
// Simplified cvt powertrain model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef SIMPLE_CVT_POWERTRAIN_H
#define SIMPLE_CVT_POWERTRAIN_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/powertrain/ChSimpleCVTPowertrain.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

class CH_VEHICLE_API SimpleCVTPowertrain : public ChSimpleCVTPowertrain {
  public:
    SimpleCVTPowertrain(const std::string& filename);
    SimpleCVTPowertrain(const rapidjson::Document& d);
    ~SimpleCVTPowertrain() {}

    virtual double GetForwardGearRatio() const override { return m_fwd_gear_ratio; }
    virtual double GetReverseGearRatio() const override { return m_rev_gear_ratio; }
    virtual double GetMaxTorque() const override { return m_max_torque; }
    virtual double GetMaxPower() const override { return m_max_power; }
    virtual double GetCriticalSpeed() const override { return m_critical_speed; }

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_fwd_gear_ratio;  // forward gear ratio (single gear transmission)
    double m_rev_gear_ratio;  // reverse gear ratio
    double m_max_torque;      // maximum motor torque
    double m_max_power;       // maximum motor power
    double m_critical_speed;  // critical motor speed where torque limiting begins/ends
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
