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

#include "chrono/functions/ChFunctionRotationSetpoint.h"
#include "chrono/functions/ChFunctionConst.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionRotationSetpoint)

ChFunctionRotationSetpoint::ChFunctionRotationSetpoint() {
    mode = eChSetpointMode::FIRST_ORDER_HOLD;
    this->Reset(0);
}

ChFunctionRotationSetpoint::ChFunctionRotationSetpoint(const ChFunctionRotationSetpoint& other) {
    mode = other.mode;
    S = other.S;
    Q = other.Q;
    W = other.W;
    A = other.A;
    last_S = other.last_S;
    last_Q = other.last_Q;
    last_W = other.last_W;
}

ChFunctionRotationSetpoint::~ChFunctionRotationSetpoint() {}

void ChFunctionRotationSetpoint::Reset(double s) {
    S = s;
    Q = QUNIT;
    W = 0;
    A = 0;
    last_S = 0;
    last_Q = QUNIT;
    last_W = 0;
}

void ChFunctionRotationSetpoint::SetSetpoint(ChQuaternion<> q_setpoint, double s) {
    if (s > S) {
        // if successive setpoint time, scroll buffer of past samples
        last_S = S;
        last_Q = Q;
        last_W = W;
    } else {
        // if same s, just update last sample
    }

    S = s;
    Q = q_setpoint;
    W = 0;
    A = 0;

    if (mode == ZERO_ORDER_HOLD) {
    }
    if (mode == FIRST_ORDER_HOLD) {
        double ds = s - last_S;
        if (ds > 0) {
            W = (last_Q.GetConjugate() * Q).GetRotVec() / ds;
            A = 0;
        }
    }
    /*
    if (mode == SOH) {
        double ds = s - last_s;
        if (ds > 0) {
            W = (Q - last_Q) / ds;
            A = (W - last_W) / ds; //// TO DO - intrinsic W and lastW, but rotation Q might have changed too much..
    require better formula?
        }
    }
    */
}

ChQuaternion<> ChFunctionRotationSetpoint::GetQuat(double s) const {
    if (mode == eChSetpointMode::OVERRIDE)
        return Q;
    ChQuaternion<> dQ;
    dQ.SetFromRotVec(W * (s - S));  // + A * pow((s - S), 2)); //// TO DO - intrinsic W and A, but rotation Q might have
                                    // changed too much.. require better formula?
    return Q * dQ;
}

ChVector3d ChFunctionRotationSetpoint::GetAngVel(double s) const {
    if (mode == eChSetpointMode::OVERRIDE)
        return W;
    return W;  // +A * (s - S); //// TO DO - intrinsic W and A, but rotation Q might have changed too much.. require
               // better formula?
}

ChVector3d ChFunctionRotationSetpoint::GetAngAcc(double s) const {
    if (mode == eChSetpointMode::OVERRIDE)
        return A;
    return A;
}

void ChFunctionRotationSetpoint::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionRotationSetpoint>();
    // serialize parent class
    ChFunctionRotation::ArchiveOut(archive_out);
    // serialize all member data:
    eChSetpointMode_mapper mmapper;
    archive_out << CHNVP(mmapper(mode), "mode");
    archive_out << CHNVP(Q);
    archive_out << CHNVP(W);
    archive_out << CHNVP(A);
}

void ChFunctionRotationSetpoint::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionRotationSetpoint>();
    // deserialize parent class
    ChFunctionRotation::ArchiveIn(archive_in);
    // deserialize all member data:
    eChSetpointMode_mapper mmapper;
    archive_in >> CHNVP(mmapper(mode), "mode");
    archive_in >> CHNVP(Q);
    archive_in >> CHNVP(W);
    archive_in >> CHNVP(A);
}

}  // end namespace chrono