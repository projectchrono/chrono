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

#include "chrono/functions/ChFunctionRotationABCFunctions.h"
#include "chrono/functions/ChFunctionConst.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionRotationABCFunctions)

ChFunctionRotationABCFunctions::ChFunctionRotationABCFunctions() {
    angleset = RotRepresentation::CARDAN_ANGLES_XYZ;

    angleA = chrono_types::make_shared<ChFunctionConst>(0);
    angleB = chrono_types::make_shared<ChFunctionConst>(0);
    angleC = chrono_types::make_shared<ChFunctionConst>(0);
}

ChFunctionRotationABCFunctions::ChFunctionRotationABCFunctions(const ChFunctionRotationABCFunctions& other) {
    angleset = other.angleset;

    angleA = std::shared_ptr<ChFunction>(other.angleA->Clone());
    angleB = std::shared_ptr<ChFunction>(other.angleB->Clone());
    angleC = std::shared_ptr<ChFunction>(other.angleC->Clone());
}

ChFunctionRotationABCFunctions::~ChFunctionRotationABCFunctions() {}

void ChFunctionRotationABCFunctions::SetRotationRepresentation(const RotRepresentation rot_rep) {
    if (rot_rep != RotRepresentation::EULER_ANGLES_ZXZ && rot_rep != RotRepresentation::CARDAN_ANGLES_XYZ &&
        rot_rep != RotRepresentation::CARDAN_ANGLES_ZXY && rot_rep != RotRepresentation::CARDAN_ANGLES_ZYX) {
        std::cerr << "Unknown input rotation representation" << std::endl;
        throw std::runtime_error("Unknown input rotation representation");
        return;
    }

    angleset = rot_rep;
}

ChQuaternion<> ChFunctionRotationABCFunctions::GetQuat(double s) const {
    return QuatFromAngleSet({angleset, ChVector3d(angleA->GetVal(s), angleB->GetVal(s), angleC->GetVal(s))});
}

// To avoid putting the following mapper macro inside the class definition,
// enclose macros in local 'ChFunctionRotationABCFunctions_RotRep_enum_mapper'.
class ChFunctionRotationABCFunctions_RotRep_enum_mapper : public ChFunctionRotationABCFunctions {
  public:
    typedef RotRepresentation ChRotationRepresentation;

    CH_ENUM_MAPPER_BEGIN(ChRotationRepresentation);
    CH_ENUM_VAL(RotRepresentation::ANGLE_AXIS);
    CH_ENUM_VAL(RotRepresentation::EULER_ANGLES_ZXZ);
    CH_ENUM_VAL(RotRepresentation::CARDAN_ANGLES_ZXY);
    CH_ENUM_VAL(RotRepresentation::CARDAN_ANGLES_ZYX);
    CH_ENUM_VAL(RotRepresentation::CARDAN_ANGLES_XYZ);
    CH_ENUM_VAL(RotRepresentation::RODRIGUES);
    CH_ENUM_MAPPER_END(ChRotationRepresentation);
};

void ChFunctionRotationABCFunctions::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionRotationABCFunctions>();

    ChFunctionRotation::ArchiveOut(archive_out);

    archive_out << CHNVP(angleA);
    archive_out << CHNVP(angleB);
    archive_out << CHNVP(angleC);

    ChFunctionRotationABCFunctions_RotRep_enum_mapper::ChRotationRepresentation_mapper setmapper;
    archive_out << CHNVP(setmapper(angleset), "angle_set");
}

void ChFunctionRotationABCFunctions::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionRotationABCFunctions>();

    ChFunctionRotation::ArchiveIn(archive_in);

    archive_in >> CHNVP(angleA);
    archive_in >> CHNVP(angleB);
    archive_in >> CHNVP(angleC);

    ChFunctionRotationABCFunctions_RotRep_enum_mapper::ChRotationRepresentation_mapper setmapper;
    archive_in >> CHNVP(setmapper(angleset), "angle_set");
}

}  // end namespace chrono