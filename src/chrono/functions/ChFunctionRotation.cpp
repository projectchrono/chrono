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

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChFunctionRotation) // NO! this is an abstract class, rather use for children concrete classes.

  static const double FD_PERTURBATION = 1e-7;

ChVector3d ChFunctionRotation::Get_w_loc(double s) const {
    ChQuaternion<> q0 = Get_q(s);
    ChQuaternion<> q1 = Get_q(s + FD_PERTURBATION);
    if (q0.Dot(q1) < 0)
        q1 = -q1;  // because q1 and -q1 are the same rotation, but for finite difference we must use the closest to q0
    ChQuaternion<> qdt = (q1 - q0) / FD_PERTURBATION;
    ChGlMatrix34<> Gl(q0);
    return Gl * qdt;
}

ChVector3d ChFunctionRotation::Get_a_loc(double s) const {
    ChQuaternion<> q0 = Get_q(s - FD_PERTURBATION);
    ChQuaternion<> q1 = Get_q(s);
    ChQuaternion<> q2 = Get_q(s + FD_PERTURBATION);
    ChQuaternion<> qdtdt = (q0 - q1 * 2.0 + q2) / (FD_PERTURBATION * FD_PERTURBATION);
    ChGlMatrix34<> Gl(q1);
    return Gl * qdtdt;
}

void ChFunctionRotation::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionRotation>();
}

void ChFunctionRotation::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChFunctionRotation>();
}

}  // end namespace chrono