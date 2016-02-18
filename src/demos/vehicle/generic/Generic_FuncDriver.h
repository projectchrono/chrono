// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

class Generic_FuncDriver : public chrono::vehicle::ChDriver {
  public:
    Generic_FuncDriver(chrono::vehicle::ChVehicle& vehicle) : chrono::vehicle::ChDriver(vehicle) {}
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

#endif
