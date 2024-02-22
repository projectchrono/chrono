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



#include "chrono/motion_functions/ChFunctionPositionXYZFunctions.h"
#include "chrono/motion_functions/ChFunctionConst.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionPositionXYZFunctions) 

ChFunctionPositionXYZFunctions::ChFunctionPositionXYZFunctions() {

	// default s(t) function. User will provide better fx.
    this->px = chrono_types::make_shared<ChFunctionConst>(0);
	this->py = chrono_types::make_shared<ChFunctionConst>(0);
	this->pz = chrono_types::make_shared<ChFunctionConst>(0);
}


ChFunctionPositionXYZFunctions::ChFunctionPositionXYZFunctions(const ChFunctionPositionXYZFunctions& other) {

	this->px = std::shared_ptr<ChFunction>(other.px->Clone());
	this->py = std::shared_ptr<ChFunction>(other.py->Clone());
	this->pz = std::shared_ptr<ChFunction>(other.pz->Clone());
}

ChFunctionPositionXYZFunctions::~ChFunctionPositionXYZFunctions() {

}



ChVector3d ChFunctionPositionXYZFunctions::Get_p(double s) const {
	return ChVector3d(
		px->GetVal(s), 
		py->GetVal(s), 
		pz->GetVal(s));
}

ChVector3d ChFunctionPositionXYZFunctions::Get_p_ds(double s) const {
	return ChVector3d(
		px->GetDer(s), 
		py->GetDer(s), 
		pz->GetDer(s));
}

ChVector3d ChFunctionPositionXYZFunctions::Get_p_dsds(double s) const {
	return ChVector3d(
		px->GetDer2(s), 
		py->GetDer2(s), 
		pz->GetDer2(s));
}

void ChFunctionPositionXYZFunctions::Estimate_s_domain(double& smin, double& smax) const {

}



void ChFunctionPositionXYZFunctions::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionPosition>();
	// serialize parent class
    ChFunctionPosition::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(px);
	marchive << CHNVP(py);
	marchive << CHNVP(pz);
}

void ChFunctionPositionXYZFunctions::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunctionPositionXYZFunctions>();
	// deserialize parent class
    ChFunctionPosition::ArchiveIn(marchive);
    // deserialize all member data:
    marchive >> CHNVP(px);
	marchive >> CHNVP(py);
	marchive >> CHNVP(pz);
}



}  // end namespace chrono