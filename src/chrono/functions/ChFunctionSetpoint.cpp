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

#include "chrono/functions/ChFunctionSetpoint.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionSetpoint)

void ChFunctionSetpoint::SetSetpoint(double setpoint, double x) {
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

void ChFunctionSetpoint::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionSetpoint>();
    // serialize parent class
    ChFunction::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(Y);
    archive_out << CHNVP(Y_dx);
    archive_out << CHNVP(Y_dxdx);
}

void ChFunctionSetpoint::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/ archive_in.VersionRead<ChFunctionSetpoint>();
    // deserialize parent class
    ChFunction::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(Y);
    archive_in >> CHNVP(Y_dx);
    archive_in >> CHNVP(Y_dxdx);
}

}  // end namespace chrono
