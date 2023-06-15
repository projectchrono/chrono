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
        double dx = x - last_x;
        Y_dx = (Y - last_Y) / dx;
        Y_dxdx = (Y_dx - last_Y_dx) / dx;
    }
    last_x = x;
    last_Y = Y;
    last_Y_dx = Y_dx;
}

void ChFunction_Setpoint::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunction_Setpoint>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(Y);
    marchive << CHNVP(Y_dx);
    marchive << CHNVP(Y_dxdx);
}

void ChFunction_Setpoint::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunction_Setpoint>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(Y);
    marchive >> CHNVP(Y_dx);
    marchive >> CHNVP(Y_dxdx);
}

}  // end namespace chrono
