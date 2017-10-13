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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/motion_functions/ChFunction_Setpoint.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_Setpoint)


void ChFunction_Setpoint::SetSetpoint(double setpoint, double x) {
    Y = setpoint;
    if (x > this->last_x) {
      
        double dx = x-last_x;
        Y_dx = ( Y - last_Y ) / dx;
        Y_dxdx = (Y_dx - last_Y_dx) / dx;

    }
    last_x = x;
    last_Y = Y;
    last_Y_dx = Y_dx;
}


}  // end namespace chrono
