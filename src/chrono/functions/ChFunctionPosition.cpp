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

#include <memory.h>
#include <cfloat>
#include <cmath>

#include "chrono/functions/ChFunctionPosition.h"

namespace chrono {

static const double FD_STEP = 1e-4;  // forward differentiation stepsize

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChFunctionPosition) // NO! this is an abstract class, rather use for children concrete classes.

ChVector3d ChFunctionPosition::GetLinVel(double s) const {
    return ((GetPos(s + FD_STEP) - GetPos(s)) / FD_STEP);
}
ChVector3d ChFunctionPosition::GetLinAcc(double s) const {
    return ((GetLinVel(s + FD_STEP) - GetLinVel(s)) / FD_STEP);
};

void ChFunctionPosition::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionPosition>();
}

void ChFunctionPosition::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionPosition>();
}

}  // end namespace chrono