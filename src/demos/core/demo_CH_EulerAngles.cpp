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
#include <cmath>

#include "chrono/core/ChChrono.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix33.h"

using namespace chrono;

void test_Euler123() {
    std::cout << "Test Euler sequence 1-2-3" << std::endl;

    ChQuaternion<> q;
    ChVector<> eu;

    double alpha1 = 10;
    double beta1 = 11;
    double gamma1 = 12;

    double alpha2 = -17.3;
    double beta2 = -41;
    double gamma2 = -0.7;

    std::cout << "  Rotations about frame axes:" << std::endl;

    std::cout << "    Rotation about X of " << alpha1 << " deg.";
    q.Q_from_AngX(CH_C_DEG_TO_RAD * alpha1);
    eu = CH_C_RAD_TO_DEG * q.Q_to_Euler123();
    std::cout << "    eu = {" << eu.x() << ";" << eu.y() << ";" << eu.z() << "}" << std::endl;

    std::cout << "    Rotation about Y of " << beta1 << " deg.";
    q.Q_from_AngY(CH_C_DEG_TO_RAD * beta1);
    eu = CH_C_RAD_TO_DEG * q.Q_to_Euler123();
    std::cout << "    eu = {" << eu.x() << ";" << eu.y() << ";" << eu.z() << "}" << std::endl;

    std::cout << "    Rotation about Z of " << gamma1 << " deg.";
    q.Q_from_AngZ(CH_C_DEG_TO_RAD * gamma1);
    eu = CH_C_RAD_TO_DEG * q.Q_to_Euler123();
    std::cout << "    eu = {" << eu.x() << ";" << eu.y() << ";" << eu.z() << "}" << std::endl;

    std::cout << "    Rotation about X of " << alpha2 << " deg.";
    q.Q_from_AngX(CH_C_DEG_TO_RAD * alpha2);
    eu = CH_C_RAD_TO_DEG * q.Q_to_Euler123();
    std::cout << "    eu = {" << eu.x() << ";" << eu.y() << ";" << eu.z() << "}" << std::endl;

    std::cout << "    Rotation about Y of " << beta2 << " deg.";
    q.Q_from_AngY(CH_C_DEG_TO_RAD * beta2);
    eu = CH_C_RAD_TO_DEG * q.Q_to_Euler123();
    std::cout << "    eu = {" << eu.x() << ";" << eu.y() << ";" << eu.z() << "}" << std::endl;

    std::cout << "    Rotation about Z of " << gamma2 << " deg.";
    q.Q_from_AngZ(CH_C_DEG_TO_RAD * gamma2);
    eu = CH_C_RAD_TO_DEG * q.Q_to_Euler123();
    std::cout << "    eu = {" << eu.x() << ";" << eu.y() << ";" << eu.z() << "}" << std::endl;

    std::cout << std::endl << "  Quaternion from Euler angles:" << std::endl;

    eu = {alpha1, beta1, gamma1};
    std::cout << "    Input = {" << eu.x() << ";" << eu.y() << ";" << eu.z() << "}";
    q.Q_from_Euler123(CH_C_DEG_TO_RAD * eu);
    eu = CH_C_RAD_TO_DEG * q.Q_to_Euler123();
    std::cout << "    Output = {" << eu.x() << ";" << eu.y() << ";" << eu.z() << "}" << std::endl;

    eu = {alpha2, beta2, gamma2};
    std::cout << "    Input = {" << eu.x() << ";" << eu.y() << ";" << eu.z() << "}";
    q.Q_from_Euler123(CH_C_DEG_TO_RAD * eu);
    eu = CH_C_RAD_TO_DEG * q.Q_to_Euler123();
    std::cout << "    Output = {" << eu.x() << ";" << eu.y() << ";" << eu.z() << "}" << std::endl;

    std::cout << std::endl << "  Euler angles -> Quaternion -> Euler angles (using free functions):" << std::endl;

    eu = {alpha1, beta1, gamma1};
    std::cout << "    Input = {" << eu.x() << ";" << eu.y() << ";" << eu.z() << "}";
    q = Q_from_Euler123(CH_C_DEG_TO_RAD * eu);
    eu = CH_C_RAD_TO_DEG * Q_to_Euler123(q);
    std::cout << "    Output = {" << eu.x() << ";" << eu.y() << ";" << eu.z() << "}" << std::endl;

    eu = {alpha2, beta2, gamma2};
    std::cout << "    Input = {" << eu.x() << ";" << eu.y() << ";" << eu.z() << "}";
    q = Q_from_Euler123(CH_C_DEG_TO_RAD * eu);
    eu = CH_C_RAD_TO_DEG * Q_to_Euler123(q);
    std::cout << "    Output = {" << eu.x() << ";" << eu.y() << ";" << eu.z() << "}" << std::endl;

    std::cout << std::endl << "  Rotation matrix for sequence (90, 90, 90)" << std::endl;

    eu = { CH_C_PI_2, CH_C_PI_2, CH_C_PI_2 };
    q = Q_from_Euler123(eu);
    ChMatrix33<> R(q);
    std::cout << "    " << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << std::endl;
    std::cout << "    " << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << std::endl;
    std::cout << "    " << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << std::endl;
}

int main(int argc, char* argv[]) {
    test_Euler123();

    return 0;
}
