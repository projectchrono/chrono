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
// Tracked vehicle simple brake model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef TRACK_BRAKE_SIMPLE_H
#define TRACK_BRAKE_SIMPLE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeSimple.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_brake
/// @{

/// Tracked vehicle simple brake model constructed with data from file (JSON format).
class CH_VEHICLE_API TrackBrakeSimple : public ChTrackBrakeSimple {
  public:
    TrackBrakeSimple(const std::string& filename);
    TrackBrakeSimple(const rapidjson::Document& d);
    ~TrackBrakeSimple() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }

  private:
    void Create(const rapidjson::Document& d);

    double m_maxtorque;
};

/// @} vehicle_tracked_brake

}  // end namespace vehicle
}  // end namespace chrono

#endif
