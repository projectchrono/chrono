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

#include <cmath>

#include "chrono/functions/ChFunctionPositionSetpoint.h"
#include "chrono/functions/ChFunctionConst.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionPositionSetpoint)

ChFunctionPositionSetpoint::ChFunctionPositionSetpoint() {
    mode = eChSetpointMode::FIRST_ORDER_HOLD;
    this->Reset(0);
}

ChFunctionPositionSetpoint::ChFunctionPositionSetpoint(const ChFunctionPositionSetpoint& other) {
    mode = other.mode;
    S = other.S;
    P = other.P;
    P_ds = other.P_ds;
    P_dsds = other.P_dsds;
    last_s = other.last_s;
    last_P = other.last_P;
    last_P_ds = other.last_P_ds;
}

ChFunctionPositionSetpoint::~ChFunctionPositionSetpoint() {}

void ChFunctionPositionSetpoint::Reset(double s) {
    S = s;
    P = 0;
    P_ds = 0;
    P_dsds = 0;
    last_s = 0;
    last_P = 0;
    last_P_ds = 0;
}

void ChFunctionPositionSetpoint::SetSetpoint(ChVector3d p_setpoint, double s) {
    if (s > S) {
        // if successive setpoint time, scroll buffer of past samples
        last_s = S;
        last_P = P;
        last_P_ds = P_ds;
    } else {
        // if same s, just update last sample
    }

    S = s;
    P = p_setpoint;
    P_ds = 0;
    P_dsds = 0;

    if (mode == ZERO_ORDER_HOLD) {
    }
    if (mode == FIRST_ORDER_HOLD) {
        double ds = s - last_s;
        if (ds > 0) {
            P_ds = (P - last_P) / ds;
            P_dsds = 0;
        }
    }
    if (mode == SOH) {
        double ds = s - last_s;
        if (ds > 0) {
            P_ds = (P - last_P) / ds;
            P_dsds = (P_ds - last_P_ds) / ds;
        }
    }
}

ChVector3d ChFunctionPositionSetpoint::GetPos(double s) const {
    if (mode == eChSetpointMode::OVERRIDE)
        return P;
    return P + P_ds * (s - S) + P_dsds * std::pow((s - S), 2);
}

ChVector3d ChFunctionPositionSetpoint::GetLinVel(double s) const {
    if (mode == eChSetpointMode::OVERRIDE)
        return P_ds;
    return P_ds + P_dsds * (s - S);
}

ChVector3d ChFunctionPositionSetpoint::GetLinAcc(double s) const {
    if (mode == eChSetpointMode::OVERRIDE)
        return P_dsds;
    return P_dsds;
}

void ChFunctionPositionSetpoint::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionPositionSetpoint>();
    // serialize parent class
    ChFunctionPosition::ArchiveOut(archive_out);
    // serialize all member data:
    eChSetpointMode_mapper mmapper;
    archive_out << CHNVP(mmapper(mode), "mode");
    archive_out << CHNVP(P);
    archive_out << CHNVP(P_ds);
    archive_out << CHNVP(P_dsds);
}

void ChFunctionPositionSetpoint::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionPositionSetpoint>();
    // deserialize parent class
    ChFunctionPosition::ArchiveIn(archive_in);
    // deserialize all member data:
    eChSetpointMode_mapper mmapper;
    archive_in >> CHNVP(mmapper(mode), "mode");
    archive_in >> CHNVP(P);
    archive_in >> CHNVP(P_ds);
    archive_in >> CHNVP(P_dsds);
}

}  // end namespace chrono