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

#include "chrono/functions/ChFunctionRotation.h"
#include "chrono/core/ChMatrixMBD.h"

namespace chrono {

static const double DIFF_STEP = 1e-4;  // forward differentiation stepsize

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChFunctionRotation) // NO! this is an abstract class, rather use for children concrete classes.

ChVector3d ChFunctionRotation::GetAngVel(double s) const {
    ChQuaternion<> q0 = GetQuat(s);
    ChQuaternion<> q1 = GetQuat(s + DIFF_STEP);
    if (q0.Dot(q1) < 0)
        q1 = -q1;  // because q1 and -q1 are the same rotation, but for finite difference we must use the closest to q0
    ChQuaternion<> qdt = (q1 - q0) / DIFF_STEP;
    ChGlMatrix34<> Gl(q0);
    return Gl * qdt;
}

ChVector3d ChFunctionRotation::GetAngAcc(double s) const {
    ChQuaternion<> q0 = GetQuat(s - DIFF_STEP);
    ChQuaternion<> q1 = GetQuat(s);
    ChQuaternion<> q2 = GetQuat(s + DIFF_STEP);
    ChQuaternion<> qdtdt = (q0 - q1 * 2.0 + q2) / (DIFF_STEP * DIFF_STEP);
    ChGlMatrix34<> Gl(q1);
    return Gl * qdtdt;
}

void ChFunctionRotation::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionRotation>();
}

void ChFunctionRotation::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionRotation>();
}

}  // end namespace chrono