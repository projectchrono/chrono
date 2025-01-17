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
// MAN 5t simple driveline model.
//
// =============================================================================

#ifndef MAN5T_SIMPLEDRIVELINE_H
#define MAN5T_SIMPLEDRIVELINE_H

#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDriveline.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace man {

/// @addtogroup vehicle_models_man
/// @{

/// Simple MAN 5t driveline subsystem (purely kinematic).
class CH_MODELS_API MAN_5t_SimpleDriveline : public ChSimpleDriveline {
  public:
    MAN_5t_SimpleDriveline(const std::string& name);

    ~MAN_5t_SimpleDriveline() {}

    virtual double GetFrontTorqueFraction() const override { return m_front_torque_frac; }
    virtual double GetFrontDifferentialMaxBias() const override { return m_front_diff_bias; }
    virtual double GetRearDifferentialMaxBias() const override { return m_rear_diff_bias; }

    virtual double GetFrontConicalGearRatio() const override { return m_front_conicalgear_ratio; }
    virtual double GetRearConicalGearRatio() const override { return m_rear_conicalgear_ratio; }

  private:
    static const double m_front_torque_frac;
    static const double m_front_diff_bias;
    static const double m_rear_diff_bias;
    static const double m_front_conicalgear_ratio;
    static const double m_rear_conicalgear_ratio;
};

/// @} vehicle_models_man

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono

#endif
