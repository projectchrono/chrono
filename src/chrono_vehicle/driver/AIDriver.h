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
// AIDriver specified through a JSON file.
//
// =============================================================================

#ifndef AI_DRIVER_H
#define AI_DRIVER_H

#include <string>

#include "chrono_vehicle/driver/ChAIDriver.h"
#include "chrono/motion_functions/ChFunction_Recorder.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_driver
/// @{

/// Driver
class CH_VEHICLE_API AIDriver : public ChAIDriver {
  public:
    /// Construct using data from the specified JSON file.
    AIDriver(ChVehicle& vehicle,          ///< associated vehicle
             const std::string& filename  ///< name of JSON specification file
    );

    ~AIDriver() {}

    /// Return the value of the vehicle steering input (in [-1,+1]) given the desired front and rear wheel angles.
    /// The underlying assumption is of Ackermann steering of a bicycle model.
    /// The current implementation ignores the rear angle (always 0 for a front steering vehicle with Ackermann
    /// geometry) and uses a mapping from front angle to steering input.
    /// Note that the Chrono::Vehicle ISO reference frame convention implies that a positive front angle corresponds
    /// to a turn to the left (i.e., positive value of vehicle steering input).
    virtual double CalculateSteering(double front_axle_angle, double rear_axle_angle) override;

  private:
    ChFunction_Recorder m_steering_map;  ///< front wheel angle to steering input mapping
};

/// @} vehicle_driver

}  // end namespace vehicle
}  // end namespace chrono

#endif
