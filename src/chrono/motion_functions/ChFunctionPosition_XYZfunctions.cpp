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



#include "chrono/motion_functions/ChFunctionPosition_XYZfunctions.h"
#include "chrono/motion_functions/ChFunction_Const.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionPosition_XYZfunctions) 

ChFunctionPosition_XYZfunctions::ChFunctionPosition_XYZfunctions() {

	// default s(t) function. User will provide better fx.
    this->px = chrono_types::make_shared<ChFunction_Const>(0);
	this->py = chrono_types::make_shared<ChFunction_Const>(0);
	this->pz = chrono_types::make_shared<ChFunction_Const>(0);
}


ChFunctionPosition_XYZfunctions::ChFunctionPosition_XYZfunctions(const ChFunctionPosition_XYZfunctions& other) {

	this->px = std::shared_ptr<ChFunction>(other.px->Clone());
	this->py = std::shared_ptr<ChFunction>(other.py->Clone());
	this->pz = std::shared_ptr<ChFunction>(other.pz->Clone());
}

ChFunctionPosition_XYZfunctions::~ChFunctionPosition_XYZfunctions() {

}



ChVector<> ChFunctionPosition_XYZfunctions::Get_p(double s) const {
	return ChVector<>(
		px->Get_y(s), 
		py->Get_y(s), 
		pz->Get_y(s));
}

ChVector<> ChFunctionPosition_XYZfunctions::Get_p_ds(double s) const {
	return ChVector<>(
		px->Get_y_dx(s), 
		py->Get_y_dx(s), 
		pz->Get_y_dx(s));
}

ChVector<> ChFunctionPosition_XYZfunctions::Get_p_dsds(double s) const {
	return ChVector<>(
		px->Get_y_dxdx(s), 
		py->Get_y_dxdx(s), 
		pz->Get_y_dxdx(s));
}

void ChFunctionPosition_XYZfunctions::Estimate_s_domain(double& smin, double& smax) const {

}



void ChFunctionPosition_XYZfunctions::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionPosition>();
	// serialize parent class
    ChFunctionPosition::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(px);
	marchive << CHNVP(py);
	marchive << CHNVP(pz);
}

void ChFunctionPosition_XYZfunctions::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunctionPosition_XYZfunctions>();
	// deserialize parent class
    ChFunctionPosition::ArchiveIn(marchive);
    // deserialize all member data:
    marchive >> CHNVP(px);
	marchive >> CHNVP(py);
	marchive >> CHNVP(pz);
}



}  // end namespace chrono