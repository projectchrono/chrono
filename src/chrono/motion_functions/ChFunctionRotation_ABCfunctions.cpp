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



#include "chrono/motion_functions/ChFunctionRotation_ABCfunctions.h"
#include "chrono/motion_functions/ChFunction_Const.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionRotation_ABCfunctions) 

ChFunctionRotation_ABCfunctions::ChFunctionRotation_ABCfunctions() {

	// default s(t) function. User will provide better fx.
    this->angleA = chrono_types::make_shared<ChFunction_Const>(0);
	this->angleB = chrono_types::make_shared<ChFunction_Const>(0);
	this->angleC = chrono_types::make_shared<ChFunction_Const>(0);
	this->angleset = AngleSet::RXYZ;
}


ChFunctionRotation_ABCfunctions::ChFunctionRotation_ABCfunctions(const ChFunctionRotation_ABCfunctions& other) {

	this->angleA = std::shared_ptr<ChFunction>(other.angleA->Clone());
	this->angleB = std::shared_ptr<ChFunction>(other.angleB->Clone());
	this->angleC = std::shared_ptr<ChFunction>(other.angleC->Clone());
	this->angleset = other.angleset;
}

ChFunctionRotation_ABCfunctions::~ChFunctionRotation_ABCfunctions() {

}



ChQuaternion<> ChFunctionRotation_ABCfunctions::Get_q(double s) const {
	
	return Angle_to_Quat(this->angleset, 
						  ChVector<>(
							this->angleA->Get_y(s),
							this->angleB->Get_y(s),
							this->angleC->Get_y(s)
						  )
						);
}



void ChFunctionRotation_ABCfunctions::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionRotation_ABCfunctions>();
	// serialize parent class
    ChFunctionRotation::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(angleA);
	marchive << CHNVP(angleB);
	marchive << CHNVP(angleC);
	marchive << CHNVP((int)angleset); //***TODO: use CH_ENUM_MAPPER_BEGIN ... END to serialize enum with readable names
}

void ChFunctionRotation_ABCfunctions::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunctionRotation_ABCfunctions>();
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