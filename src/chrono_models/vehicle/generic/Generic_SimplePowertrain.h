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

#ifndef GENERIC_SIMPLEPOWERTRAIN_H
#define GENERIC_SIMPLEPOWERTRAIN_H

#include "chrono_vehicle/powertrain/ChSimplePowertrain.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// Simple powertrain model for the generic vehicle (purely kinematic).
class CH_MODELS_API Generic_SimplePowertrain : public ChSimplePowertrain {
  public:
    Generic_SimplePowertrain(const std::string& name);
    ~Generic_SimplePowertrain() {}

    virtual double GetForwardGearRatio() const override { return m_fwd_gear_ratio; }
    virtual double GetReverseGearRatio() const override { return m_rev_gear_ratio; }
    virtual double GetMaxTorque() const override { return m_max_torque; }
    virtual double GetMaxSpeed() const override { return m_max_speed; }

  private:
    static const double m_max_torque;
    static const double m_max_speed;
    static const double m_fwd_gear_ratio;
    static const double m_rev_gear_ratio;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
