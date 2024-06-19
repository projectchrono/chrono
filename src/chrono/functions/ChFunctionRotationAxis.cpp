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

ChQuaternion<> ChFunctionRotationAxis::GetQuat(double s) const {
    return QuatFromAngleAxis(this->fangle->GetVal(s), this->axis);
}

ChVector3d ChFunctionRotationAxis::GetAngVel(double s) const {
    return this->fangle->GetDer(s) * this->axis;
}

ChVector3d ChFunctionRotationAxis::GetAngAcc(double s) const {
    return this->fangle->GetDer2(s) * this->axis;
}

void ChFunctionRotationAxis::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionRotationAxis>();
    // serialize parent class
    ChFunctionRotation::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(fangle);
    archive_out << CHNVP(axis);
}

void ChFunctionRotationAxis::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionRotationAxis>();
    // deserialize parent class
    ChFunctionRotation::ArchiveIn(archive_in);
    // deserialize all member data:
    archive_in >> CHNVP(fangle);
    archive_in >> CHNVP(axis);
}

}  // end namespace chrono