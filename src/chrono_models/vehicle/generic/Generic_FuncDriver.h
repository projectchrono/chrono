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
//
// =============================================================================

#ifndef GENERIC_FUNCDRIVER_H
#define GENERIC_FUNCDRIVER_H

#include "chrono_vehicle/ChDriver.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// Open-loop driver model for use with a generic vehicle.
class Generic_FuncDriver : public ChDriver {
  public:
    Generic_FuncDriver(ChVehicle& vehicle) : ChDriver(vehicle) {}
    ~Generic_FuncDriver() {}

    virtual void Synchronize(double time) override {
        if (time < 0.5)
            m_throttle = 0;
        else if (time < 1.5)
            m_throttle = 0.4 * (time - 0.5);
        else
            m_throttle = 0.4;

        if (time < 4)
            m_steering = 0;
        else if (time < 6)
            m_steering = 0.25 * (time - 4);
        else if (time < 10)
            m_steering = -0.25 * (time - 6) + 0.5;
        else
            m_steering = -0.5;
    }
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
