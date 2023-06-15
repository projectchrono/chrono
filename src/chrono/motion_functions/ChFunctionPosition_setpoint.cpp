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



#include "chrono/motion_functions/ChFunctionPosition_setpoint.h"
#include "chrono/motion_functions/ChFunction_Const.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionPosition_setpoint) 

ChFunctionPosition_setpoint::ChFunctionPosition_setpoint() {
    mode = eChSetpointMode::FOH;
	this->Reset(0);
}

ChFunctionPosition_setpoint::ChFunctionPosition_setpoint(const ChFunctionPosition_setpoint& other) {
    mode = other.mode;
    S = other.S;
    P = other.P;
    P_ds = other.P_ds;
    P_dsds = other.P_dsds;
    last_s = other.last_s;
    last_P = other.last_P;
    last_P_ds = other.last_P_ds;
}

ChFunctionPosition_setpoint::~ChFunctionPosition_setpoint() {

}

void ChFunctionPosition_setpoint::Reset(double s) {
    S = s;
    P = 0;
    P_ds = 0;
    P_dsds = 0;
    last_s = 0;
    last_P = 0;
    last_P_ds = 0;
}


void ChFunctionPosition_setpoint::SetSetpoint(ChVector<> p_setpoint, double s) {
	
	if (s > S) {  
		// if successive setpoint time, scroll buffer of past samples
		last_s = S;
		last_P = P;
		last_P_ds = P_ds;
	}
	else {
		// if same s, just update last sample
	}

	S = s;
	P = p_setpoint;
	P_ds = 0;
	P_dsds = 0;

	if (mode == ZOH) {
		
	}
	if (mode == FOH) {
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


ChVector<> ChFunctionPosition_setpoint::Get_p(double s) const {
    if (mode == eChSetpointMode::OVERRIDE)
        return P;
    return P + P_ds * (s - S) + P_dsds * pow((s - S), 2);
}

ChVector<> ChFunctionPosition_setpoint::Get_p_ds(double s) const {
    if (mode == eChSetpointMode::OVERRIDE)
        return P_ds;
    return P_ds + P_dsds * (s - S);
}

ChVector<> ChFunctionPosition_setpoint::Get_p_dsds(double s) const {
    if (mode == eChSetpointMode::OVERRIDE)
        return P_dsds;
    return P_dsds;
}



void ChFunctionPosition_setpoint::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionPosition_setpoint>();
	// serialize parent class
    ChFunctionPosition::ArchiveOut(marchive);
    // serialize all member data:
	eChSetpointMode_mapper mmapper;
	marchive << CHNVP(mmapper(mode), "mode");
    marchive << CHNVP(P);
	marchive << CHNVP(P_ds);
	marchive << CHNVP(P_dsds);

}

void ChFunctionPosition_setpoint::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunctionPosition_setpoint>();
	// deserialize parent class
    ChFunctionPosition::ArchiveIn(marchive);
    // deserialize all member data:
	eChSetpointMode_mapper mmapper;
	marchive >> CHNVP(mmapper(mode), "mode");
    marchive >> CHNVP(P);
	marchive >> CHNVP(P_ds);
	marchive >> CHNVP(P_dsds);
}



}  // end namespace chrono