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
// Simple driveline model template using data from file (JSON format).
//
// =============================================================================

#ifndef SIMPLE_DRIVELINE_H
#define SIMPLE_DRIVELINE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChSimpleDriveline.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_driveline
/// @{

/// Simple driveline model template using data from file (JSON format).
class CH_VEHICLE_API SimpleDriveline : public ChSimpleDriveline {
  public:
    SimpleDriveline(const std::string& filename);
    SimpleDriveline(const rapidjson::Document& d);
    ~SimpleDriveline() {}

    virtual double GetFrontTorqueFraction() const override { return m_front_torque_frac; }
    virtual double GetFrontDifferentialMaxBias() const override { return m_front_diff_bias; }
    virtual double GetRearDifferentialMaxBias() const override { return m_rear_diff_bias; }

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_front_torque_frac;
    double m_front_diff_bias;
    double m_rear_diff_bias;
};

/// @} vehicle_wheeled_driveline

}  // end namespace vehicle
}  // end namespace chrono

#endif
