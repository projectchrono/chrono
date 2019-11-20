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



#include "chrono/motion_functions/ChFunctionPosition_line.h"
#include "chrono/geometry/ChLineSegment.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionPosition_line) 

ChFunctionPosition_line::ChFunctionPosition_line() {

	// default trajectory is a segment
    this->trajectory_line = chrono_types::make_shared<geometry::ChLineSegment>();
}


ChFunctionPosition_line::ChFunctionPosition_line(const ChFunctionPosition_line& other) {

	//trajectory_line = other.trajectory_line;
    trajectory_line = std::shared_ptr<geometry::ChLine>((geometry::ChLine*)other.trajectory_line->Clone());  // deep copy
}

ChFunctionPosition_line::~ChFunctionPosition_line() {

}



ChVector<> ChFunctionPosition_line::Get_p(double s) const {
	ChVector<> result;
    trajectory_line->Evaluate(result, s);
	return result;
}

ChVector<> ChFunctionPosition_line::Get_p_ds(double s) const {
	ChVector<> result;
    trajectory_line->Derive(result, s);  // some chLine implement the Derive analytically, so better exploit this and do not fallback on BDF
	return result;
}

ChVector<> ChFunctionPosition_line::Get_p_dsds(double s) const {
	// note: same as not implementing the function and let the fallback default BDF numerical differentiation do similar computation... to remove?
	double tstep = BDF_STEP_HIGH;
	double tr_time = s;
    double tr_timeB = s + tstep;
    double tr_timeA = s - tstep;

    ChVector<> result, resultB, resultA;

    trajectory_line->Evaluate(result, tr_time);
    trajectory_line->Evaluate(resultA, tr_timeA);
    trajectory_line->Evaluate(resultB, tr_timeB);

    return (resultA + resultB - result * 2) * (4 / pow(2 * tstep, 2));
}



void ChFunctionPosition_line::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionPosition>();
	// serialize parent class
    ChFunctionPosition::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(trajectory_line);

}

void ChFunctionPosition_line::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChFunctionPosition_line>();
	// deserialize parent class
    ChFunctionPosition::ArchiveIN(marchive);
    // deserialize all member data:
    marchive >> CHNVP(trajectory_line);

}



}  // end namespace chrono