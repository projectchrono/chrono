// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#include <cmath>
#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "chrono/core/ChRotation.h"

namespace chrono {

static const double FD_STEP = 1e-4;

ChVector3d ChRotation::AngleSetToAngleSet(ChRotation::Representation from,
                                          Representation to,
                                          const ChVector3d& from_angles) {
    ChMatrix33<> R;

    switch (from) {
        case Representation::EULER_ANGLES_ZXZ:
            R.SetFromEulerAnglesZXZ(from_angles);
            break;
        case Representation::CARDAN_ANGLES_ZXY:
            R.SetFromCardanAnglesZXY(from_angles);
            break;
        case Representation::CARDAN_ANGLES_ZYX:
            R.SetFromCardanAnglesZYX(from_angles);
            break;
        case Representation::CARDAN_ANGLES_XYZ:
            R.SetFromCardanAnglesXYZ(from_angles);
            break;
        default:
            std::cerr << "Unknown input angle set representation" << std::endl;
            throw std::runtime_error("Unknown input angle set representation");
            break;
    }

    ChVector3d to_angles;
    switch (to) {
        case Representation::EULER_ANGLES_ZXZ:
            to_angles = R.GetEulerAnglesZXZ();
            break;
        case Representation::CARDAN_ANGLES_ZXY:
            to_angles = R.GetCardanAnglesZXY();
            break;
        case Representation::CARDAN_ANGLES_ZYX:
            to_angles = R.GetCardanAnglesZYX();
            break;
        case Representation::CARDAN_ANGLES_XYZ:
            to_angles = R.GetCardanAnglesXYZ();
            break;
        default:
            std::cerr << "Unknown output angle set representation" << std::endl;
            throw std::runtime_error("Unknown output angle set representation");
            break;
    }

    return to_angles;
}

ChVector3d ChRotation::AngleSetToRodriguez(Representation from, const ChVector3d& angles) {
    ChMatrix33<> R;

    switch (from) {
        case Representation::EULER_ANGLES_ZXZ:
            R.SetFromEulerAnglesZXZ(angles);
            break;
        case Representation::CARDAN_ANGLES_ZXY:
            R.SetFromCardanAnglesZXY(angles);
            break;
        case Representation::CARDAN_ANGLES_ZYX:
            R.SetFromCardanAnglesZYX(angles);
            break;
        case Representation::CARDAN_ANGLES_XYZ:
            R.SetFromCardanAnglesXYZ(angles);
            break;
        default:
            std::cerr << "Unknown input angle set representation" << std::endl;
            throw std::runtime_error("Unknown input angle set representation");
            break;
    }

    return R.GetRodriguezParameters();
}

ChVector3d ChRotation::RodriguezToAngleSet(Representation to, const ChVector3d& params) {
    ChMatrix33<> R;
    R.SetFromRodriguezParameters(params);

    ChVector3d to_angles;
    switch (to) {
        case Representation::EULER_ANGLES_ZXZ:
            to_angles = R.GetEulerAnglesZXZ();
            break;
        case Representation::CARDAN_ANGLES_ZXY:
            to_angles = R.GetCardanAnglesZXY();
            break;
        case Representation::CARDAN_ANGLES_ZYX:
            to_angles = R.GetCardanAnglesZYX();
            break;
        case Representation::CARDAN_ANGLES_XYZ:
            to_angles = R.GetCardanAnglesXYZ();
            break;
        default:
            std::cerr << "Unknown output angle set representation" << std::endl;
            throw std::runtime_error("Unknown output angle set representation");
            break;
    }

    return to_angles;
}

ChVector3d ChRotation::QuaternionToRodriguez(const ChQuaterniond& q) {
    ChMatrix33<> R(q);
    return R.GetRodriguezParameters();
}

ChQuaterniond ChRotation::RodriguezToQuaternion(const ChVector3d& params) {
    ChMatrix33<> R;
    R.SetFromRodriguezParameters(params);
    return R.GetQuaternion();
}

ChQuaterniond ChRotation::RodriguezToQuaternionDer(const ChVector3d& params_der, const ChQuaterniond& q) {
    auto params1 = QuaternionToRodriguez(q);
    auto params2 = Vadd(params1, Vmul(params_der, FD_STEP));
    auto q2 = RodriguezToQuaternion(params2);

    return Qscale(Qsub(q2, q), 1 / FD_STEP);
}

ChQuaterniond ChRotation::RodriguezToQuaternionDer2(const ChVector3d& params_der2, const ChQuaterniond& q) {
    auto ang0 = QuaternionToRodriguez(q);
    auto paramsA = Vsub(ang0, Vmul(params_der2, FD_STEP));
    auto paramsB = Vadd(ang0, Vmul(params_der2, FD_STEP));
    auto qa = RodriguezToQuaternion(paramsA);
    auto qb = RodriguezToQuaternion(paramsB);

    return Qscale(Qadd(Qadd(qa, qb), Qscale(q, -2)), 1 / FD_STEP);
}

ChVector3d ChRotation::QuaternionToAngleSet(ChRotation::Representation to, const ChQuaterniond& q) {
    ChMatrix33<> R(q);

    ChVector3d to_angles;
    switch (to) {
        case Representation::EULER_ANGLES_ZXZ:
            to_angles = R.GetEulerAnglesZXZ();
            break;
        case Representation::CARDAN_ANGLES_ZXY:
            to_angles = R.GetCardanAnglesZXY();
            break;
        case Representation::CARDAN_ANGLES_ZYX:
            to_angles = R.GetCardanAnglesZYX();
            break;
        case Representation::CARDAN_ANGLES_XYZ:
            to_angles = R.GetCardanAnglesXYZ();
            break;
        default:
            std::cerr << "Unknown output angle set representation" << std::endl;
            throw std::runtime_error("Unknown output angle set representation");
            break;
    }

    return to_angles;
}

ChQuaterniond ChRotation::AngleSetToQuaternion(Representation from, const ChVector3d& angles) {
    ChMatrix33<> R;

    switch (from) {
        case Representation::EULER_ANGLES_ZXZ:
            R.SetFromEulerAnglesZXZ(angles);
            break;
        case Representation::CARDAN_ANGLES_ZXY:
            R.SetFromCardanAnglesZXY(angles);
            break;
        case Representation::CARDAN_ANGLES_ZYX:
            R.SetFromCardanAnglesZYX(angles);
            break;
        case Representation::CARDAN_ANGLES_XYZ:
            R.SetFromCardanAnglesXYZ(angles);
            break;
        default:
            std::cerr << "Unknown input angle set representation" << std::endl;
            throw std::runtime_error("Unknown input angle set representation");
            break;
    }

    return R.GetQuaternion();
}

ChQuaterniond ChRotation::AngleSetToQuaternionDer(Representation from,
                                                  const ChVector3d& angles_der,
                                                  const ChQuaterniond& q) {
    auto ang1 = QuaternionToAngleSet(from, q);
    auto ang2 = Vadd(ang1, Vmul(angles_der, FD_STEP));
    auto q2 = AngleSetToQuaternion(from, ang2);

    return Qscale(Qsub(q2, q), 1 / FD_STEP);
}

ChQuaterniond ChRotation::AngleSetToQuaternionDer2(Representation from,
                                                   const ChVector3d& angles_der2,
                                                   const ChQuaterniond& q) {
    auto ang0 = QuaternionToAngleSet(from, q);
    auto angA = Vsub(ang0, Vmul(angles_der2, FD_STEP));
    auto angB = Vadd(ang0, Vmul(angles_der2, FD_STEP));
    auto qa = AngleSetToQuaternion(from, angA);
    auto qb = AngleSetToQuaternion(from, angB);

    return Qscale(Qadd(Qadd(qa, qb), Qscale(q, -2)), 1 / FD_STEP);
}

}  // end namespace chrono
