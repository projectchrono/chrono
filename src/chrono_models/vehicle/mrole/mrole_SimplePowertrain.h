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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Simple powertrain model for the mrole vehicle.
// - trivial speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#ifndef MROLE_SIMPLEPOWERTRAIN_H
#define MROLE_SIMPLEPOWERTRAIN_H

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/powertrain/ChSimplePowertrain.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace mrole {

/// @addtogroup vehicle_models_mrole
/// @{

/// Simple mrole powertrain subsystem (purely kinematic).
class CH_MODELS_API mrole_SimplePowertrain : public ChSimplePowertrain {
  public:
    mrole_SimplePowertrain(const std::string& name);

    ~mrole_SimplePowertrain() {}

    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;
    virtual double GetMaxTorque() const override { return m_max_torque; }
    virtual double GetMaxSpeed() const override { return m_max_speed; }

  private:
    static const double m_fwd_gear_ratio;  // forward gear ratio (single gear transmission)
    static const double m_rev_gear_ratio;  // reverse gear ratio
    static const double m_max_torque;      // maximum motor torque
    static const double m_max_speed;       // maximum motor speed
};

/// @} vehicle_models_mrole

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono

#endif
