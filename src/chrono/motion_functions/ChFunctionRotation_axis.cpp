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



#include "chrono/motion_functions/ChFunctionRotation_axis.h"
#include "chrono/motion_functions/ChFunction_Const.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionRotation_axis) 

ChFunctionRotation_axis::ChFunctionRotation_axis() {

	// default s(t) function. User will provide better fx.
    this->fangle = chrono_types::make_shared<ChFunction_Const>(0);
	this->axis = VECT_Z;
}


ChFunctionRotation_axis::ChFunctionRotation_axis(const ChFunctionRotation_axis& other) {

	this->fangle = std::shared_ptr<ChFunction>(other.fangle->Clone());
	this->axis = other.axis;
}

ChFunctionRotation_axis::~ChFunctionRotation_axis() {

}



ChQuaternion<> ChFunctionRotation_axis::Get_q(double s) const {
	
	return Q_from_AngAxis(this->fangle->Get_y(s), this->axis);
}

ChVector<> ChFunctionRotation_axis::Get_w_loc(double s) const {

	return this->fangle->Get_y_dx(s) * this->axis;
}

ChVector<> ChFunctionRotation_axis::Get_a_loc(double s) const {

	return this->fangle->Get_y_dxdx(s) * this->axis;
}



void ChFunctionRotation_axis::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionRotation_axis>();
	// serialize parent class
    ChFunctionRotation::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(fangle);
	marchive << CHNVP(axis);
}

void ChFunctionRotation_axis::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunctionRotation_axis>();
	// deserialize parent class
    ChFunctionRotation::ArchiveIn(marchive);
    // deserialize all member data:
    marchive >> CHNVP(fangle);
	marchive >> CHNVP(axis);
}



}  // end namespace chrono