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

#include "chrono/motion_functions/ChFunctionRotation_setpoint.h"
#include "chrono/motion_functions/ChFunction_Const.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionRotation_setpoint)

ChFunctionRotation_setpoint::ChFunctionRotation_setpoint() {
    mode = eChSetpointMode::FOH;
    this->Reset(0);
}

ChFunctionRotation_setpoint::ChFunctionRotation_setpoint(const ChFunctionRotation_setpoint& other) {
    mode = other.mode;
    S = other.S;
    Q = other.Q;
    W = other.W;
    A = other.A;
    last_S = other.last_S;
    last_Q = other.last_Q;
    last_W = other.last_W;
}

ChFunctionRotation_setpoint::~ChFunctionRotation_setpoint() {}

void ChFunctionRotation_setpoint::Reset(double s) {
    S = s;
    Q = QUNIT;
    W = 0;
    A = 0;
    last_S = 0;
    last_Q = QUNIT;
    last_W = 0;
}

void ChFunctionRotation_setpoint::SetSetpoint(ChQuaternion<> q_setpoint, double s) {
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

    if (mode == ZOH) {
    }
    if (mode == FOH) {
        double ds = s - last_S;
        if (ds > 0) {
            W = (last_Q.GetConjugate() * Q).Q_to_Rotv() / ds;
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

ChQuaternion<> ChFunctionRotation_setpoint::Get_q(double s) const {
    if (mode == eChSetpointMode::OVERRIDE)
        return Q;
    ChQuaternion<> dQ;
    dQ.Q_from_Rotv(W * (s - S));  // + A * pow((s - S), 2)); //// TO DO - intrinsic W and A, but rotation Q might have
                                  // changed too much.. require better formula?
    return Q * dQ;
}

ChVector<> ChFunctionRotation_setpoint::Get_w_loc(double s) const {
    if (mode == eChSetpointMode::OVERRIDE)
        return W;
    return W;  // +A * (s - S); //// TO DO - intrinsic W and A, but rotation Q might have changed too much.. require
               // better formula?
}

ChVector<> ChFunctionRotation_setpoint::Get_a_loc(double s) const {
    if (mode == eChSetpointMode::OVERRIDE)
        return A;
    return A;
}

void ChFunctionRotation_setpoint::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionRotation_setpoint>();
    // serialize parent class
    ChFunctionRotation::ArchiveOut(marchive);
    // serialize all member data:
    eChSetpointMode_mapper mmapper;
    marchive << CHNVP(mmapper(mode), "mode");
    marchive << CHNVP(Q);
    marchive << CHNVP(W);
    marchive << CHNVP(A);
}

void ChFunctionRotation_setpoint::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunctionRotation_setpoint>();
    // deserialize parent class
    ChFunctionRotation::ArchiveIn(marchive);
    // deserialize all member data:
    eChSetpointMode_mapper mmapper;
    marchive >> CHNVP(mmapper(mode), "mode");
    marchive >> CHNVP(Q);
    marchive >> CHNVP(W);
    marchive >> CHNVP(A);
}

}  // end namespace chrono