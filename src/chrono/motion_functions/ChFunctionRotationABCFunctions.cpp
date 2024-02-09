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



#include "chrono/motion_functions/ChFunctionRotationABCFunctions.h"
#include "chrono/motion_functions/ChFunctionConst.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionRotationABCFunctions) 

ChFunctionRotationABCFunctions::ChFunctionRotationABCFunctions() {

	// default s(t) function. User will provide better fx.
    this->angleA = chrono_types::make_shared<ChFunctionConst>(0);
	this->angleB = chrono_types::make_shared<ChFunctionConst>(0);
	this->angleC = chrono_types::make_shared<ChFunctionConst>(0);
	this->angleset = AngleSet::RXYZ;
}


ChFunctionRotationABCFunctions::ChFunctionRotationABCFunctions(const ChFunctionRotationABCFunctions& other) {

	this->angleA = std::shared_ptr<ChFunction>(other.angleA->Clone());
	this->angleB = std::shared_ptr<ChFunction>(other.angleB->Clone());
	this->angleC = std::shared_ptr<ChFunction>(other.angleC->Clone());
	this->angleset = other.angleset;
}

ChFunctionRotationABCFunctions::~ChFunctionRotationABCFunctions() {

}



ChQuaternion<> ChFunctionRotationABCFunctions::Get_q(double s) const {
	
	return Angle_to_Quat(this->angleset, 
						  ChVector3d(
							this->angleA->Get_y(s),
							this->angleB->Get_y(s),
							this->angleC->Get_y(s)
						  )
						);
}



void ChFunctionRotationABCFunctions::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionRotationABCFunctions>();
	// serialize parent class
    ChFunctionRotation::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(angleA);
	marchive << CHNVP(angleB);
	marchive << CHNVP(angleC);
	marchive << CHNVP((int)angleset); //***TODO: use CH_ENUM_MAPPER_BEGIN ... END to serialize enum with readable names
}

void ChFunctionRotationABCFunctions::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunctionRotationABCFunctions>();
	// deserialize parent class
    ChFunctionRotation::ArchiveIn(marchive);
    // deserialize all member data:
    marchive >> CHNVP(angleA);
	marchive >> CHNVP(angleB);
	marchive >> CHNVP(angleC);
	int foo;
	marchive >> CHNVP(foo);
	angleset = (AngleSet)foo;  //***TODO: use CH_ENUM_MAPPER_BEGIN ... END to serialize enum with readable names
}



}  // end namespace chrono