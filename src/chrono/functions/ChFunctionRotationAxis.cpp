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

#include "chrono/functions/ChFunctionRotationAxis.h"
#include "chrono/functions/ChFunctionConst.h"

#include "chrono/core/ChRotation.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionRotationAxis)

ChFunctionRotationAxis::ChFunctionRotationAxis() {
    // default s(t) function. User will provide better fx.
    this->fangle = chrono_types::make_shared<ChFunctionConst>(0);
    this->axis = VECT_Z;
}

ChFunctionRotationAxis::ChFunctionRotationAxis(const ChFunctionRotationAxis& other) {
    this->fangle = std::shared_ptr<ChFunction>(other.fangle->Clone());
    this->axis = other.axis;
}

ChFunctionRotationAxis::~ChFunctionRotationAxis() {}

ChQuaternion<> ChFunctionRotationAxis::Get_q(double s) const {
    return QuatFromAngleAxis(this->fangle->GetVal(s), this->axis);
}

ChVector3d ChFunctionRotationAxis::Get_w_loc(double s) const {
    return this->fangle->GetDer(s) * this->axis;
}

ChVector3d ChFunctionRotationAxis::Get_a_loc(double s) const {
    return this->fangle->GetDer2(s) * this->axis;
}

void ChFunctionRotationAxis::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionRotationAxis>();
    // serialize parent class
    ChFunctionRotation::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(fangle);
    marchive << CHNVP(axis);
}

void ChFunctionRotationAxis::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChFunctionRotationAxis>();
    // deserialize parent class
    ChFunctionRotation::ArchiveIn(marchive);
    // deserialize all member data:
    marchive >> CHNVP(fangle);
    marchive >> CHNVP(axis);
}

}  // end namespace chrono