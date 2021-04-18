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
// AIDriver for the Sedan vehicle.
// Uses a maximum front wheel angle of 0.63 rad (about 36 degrees).
//
// =============================================================================

#ifndef SEDAN_AI_DRIVER
#define SEDAN_AI_DRIVER

#include "chrono_vehicle/driver/ChAIDriver.h"
#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace sedan {

/// @addtogroup vehicle_models_sedan
/// @{

class CH_MODELS_API Sedan_AIDriver : public ChAIDriver {
  public:
    Sedan_AIDriver(ChVehicle& vehicle);
    ~Sedan_AIDriver() {}

    /// Return the value of the vehicle steering input (in [-1,+1]) given the desired front and rear wheel angles.
    /// The underlying assumption is of Ackermann steering of a bicycle model.
    /// This implementation ignores the rear angle (always 0 for a front steering vehicle with Ackermann geometry)
    /// and uses a simple linear interpolation from max angle right to max angle left.
    /// Note that the Chrono::Vehicle ISO reference frame convention implies that a positive front angle corresponds
    /// to a turn to the left (i.e., positive value of vehicle steering input).
    virtual double CalculateSteering(double front_axle_angle, double rear_axle_angle) override;

  private:
    static const double m_max_front_angle;
};

/// @} vehicle_models_sedan

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono

#endif