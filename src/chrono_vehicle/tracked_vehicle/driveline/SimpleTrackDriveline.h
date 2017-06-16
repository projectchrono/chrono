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
// Simple tracked vehicle driveline model template using data from file (JSON format).
//
// =============================================================================

#ifndef SIMPLE_TRACK_DRIVELINE_H
#define SIMPLE_TRACK_DRIVELINE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/driveline/ChSimpleTrackDriveline.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_driveline
/// @{

/// Simple tracked vehicle driveline model template using data from file (JSON format).
class CH_VEHICLE_API SimpleTrackDriveline : public ChSimpleTrackDriveline {
  public:
    SimpleTrackDriveline(const std::string& filename);
    SimpleTrackDriveline(const rapidjson::Document& d);
    ~SimpleTrackDriveline() {}

    virtual double GetDifferentialMaxBias() const override { return m_diff_bias; }

  private:
    void Create(const rapidjson::Document& d);

    double m_diff_bias;
};

/// @} vehicle_tracked_driveline

}  // end namespace vehicle
}  // end namespace chrono

#endif
