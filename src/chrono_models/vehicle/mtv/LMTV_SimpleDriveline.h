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
// LMTV 2.5t simple driveline model.
//
// =============================================================================

#ifndef MAN5T_SIMPLEDRIVELINE_H
#define MAN5T_SIMPLEDRIVELINE_H

#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDriveline.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace mtv {

/// @addtogroup vehicle_models_mtv
/// @{

/// Simple MAN 5t driveline subsystem (purely kinematic).
class CH_MODELS_API LMTV_SimpleDriveline : public ChSimpleDriveline {
  public:
    LMTV_SimpleDriveline(const std::string& name);

    ~LMTV_SimpleDriveline() {}

    virtual double GetFrontTorqueFraction() const override { return m_front_torque_frac; }
    virtual double GetFrontDifferentialMaxBias() const override { return m_front_diff_bias; }
    virtual double GetRearDifferentialMaxBias() const override { return m_rear_diff_bias; }

  private:
    static const double m_front_torque_frac;
    static const double m_front_diff_bias;
    static const double m_rear_diff_bias;
};

/// @} vehicle_models_mtv

}  // namespace mtv
}  // end namespace vehicle
}  // end namespace chrono

#endif
