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
// HMMWV simple driveline model.
//
// =============================================================================

#ifndef HMMWV_SIMPLEDRIVELINE_H
#define HMMWV_SIMPLEDRIVELINE_H

#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDriveline.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Simple HMMWV driveline subsystem (purely kinematic).
class CH_MODELS_API HMMWV_SimpleDriveline : public ChSimpleDriveline {
  public:
    HMMWV_SimpleDriveline(const std::string& name);

    ~HMMWV_SimpleDriveline() {}

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

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
