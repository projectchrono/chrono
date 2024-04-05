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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Demo for quaternion to Euler angle sequence conversions.
//
// =============================================================================

#include <iostream>
#include <iomanip>
#include <cmath>

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChRotation.h"

using namespace chrono;

void test_Euler123() {
    ChQuaternion<> q;
    ChVector3d eu;

    double alpha1 = 10.0;
    double beta1 = 11;
    double gamma1 = 12;

    double alpha2 = -17.3;
    double beta2 = -41;
    double gamma2 = -0.7;

    std::cout << "Rotations about frame axes:" << std::endl;

    std::cout << std::fixed << std::setprecision(4);

    std::cout << "Rotation about X of " << alpha1 << " deg.";
    q.SetFromAngleX(CH_DEG_TO_RAD * alpha1);
    eu = CH_RAD_TO_DEG * q.GetCardanAnglesXYZ();
    std::cout << "\teu = {" << eu << "}" << std::endl;

    std::cout << "Rotation about Y of " << beta1 << " deg.";
    q.SetFromAngleY(CH_DEG_TO_RAD * beta1);
    eu = CH_RAD_TO_DEG * q.GetCardanAnglesXYZ();
    std::cout << "\teu = {" << eu << "}" << std::endl;

    std::cout << "Rotation about Z of " << gamma1 << " deg.";
    q.SetFromAngleZ(CH_DEG_TO_RAD * gamma1);
    eu = CH_RAD_TO_DEG * q.GetCardanAnglesXYZ();
    std::cout << "\teu = {" << eu << "}" << std::endl;

    std::cout << "Rotation about X of " << alpha2 << " deg.";
    q.SetFromAngleX(CH_DEG_TO_RAD * alpha2);
    eu = CH_RAD_TO_DEG * q.GetCardanAnglesXYZ();
    std::cout << "\teu = {" << eu << "}" << std::endl;

    std::cout << "Rotation about Y of " << beta2 << " deg.";
    q.SetFromAngleY(CH_DEG_TO_RAD * beta2);
    eu = CH_RAD_TO_DEG * q.GetCardanAnglesXYZ();
    std::cout << "\teu = {" << eu << "}" << std::endl;

    std::cout << "Rotation about Z of " << gamma2 << " deg.";
    q.SetFromAngleZ(CH_DEG_TO_RAD * gamma2);
    eu = CH_RAD_TO_DEG * q.GetCardanAnglesXYZ();
    std::cout << "\teu = {" << eu << "}" << std::endl;

    std::cout << "\n-----------------\n" << std::endl;

    std::cout << "Quaternion from Euler angles:" << std::endl;

    eu = {alpha1, beta1, gamma1};
    std::cout << " Input = {" << eu << "}  ";
    q.SetFromCardanAnglesXYZ(CH_DEG_TO_RAD * eu);
    std::cout << "q = {" << q << "}  ";
    eu = CH_RAD_TO_DEG * q.GetCardanAnglesXYZ();
    std::cout << "Output = {" << eu << "}" << std::endl;

    eu = {alpha2, beta2, gamma2};
    std::cout << " Input = {" << eu << "}  ";
    q.SetFromCardanAnglesXYZ(CH_DEG_TO_RAD * eu);
    std::cout << "q = {" << q << "}  ";
    eu = CH_RAD_TO_DEG * q.GetCardanAnglesXYZ();
    std::cout << "Output = {" << eu << "}" << std::endl;

    std::cout << "\n-----------------\n" << std::endl;

    std::cout << "Euler angles -> Quaternion -> Euler angles (using free functions):" << std::endl;
    AngleSet set;

    eu = {alpha1, beta1, gamma1};
    std::cout << " Input = {" << eu << "}  ";
    q = QuatFromAngleSet({RotRepresentation::CARDAN_ANGLES_XYZ, CH_DEG_TO_RAD * eu});
    std::cout << "q = {" << q << "}  ";
    set = AngleSetFromQuat(RotRepresentation::CARDAN_ANGLES_XYZ, q);
    std::cout << CH_RAD_TO_DEG * set.angles << std::endl;

    eu = {alpha2, beta2, gamma2};
    std::cout << " Input = {" << eu << "}";
    q = QuatFromAngleSet({RotRepresentation::CARDAN_ANGLES_XYZ, CH_DEG_TO_RAD * eu});
    std::cout << "q = {" << q << "}  ";
    set = AngleSetFromQuat(RotRepresentation::CARDAN_ANGLES_XYZ, q);
    std::cout << CH_RAD_TO_DEG * set.angles << std::endl;

    std::cout << "\n-----------------\n" << std::endl;

    std::cout << "Rotation matrix for sequence (90, 90, 90)" << std::endl;

    eu = {CH_PI_2, CH_PI_2, CH_PI_2};
    q = QuatFromAngleSet({RotRepresentation::CARDAN_ANGLES_XYZ, CH_DEG_TO_RAD * eu});
    ChMatrix33<> R(q);
    std::cout << R << std::endl;
}

int main(int argc, char* argv[]) {
    test_Euler123();

    return 0;
}
