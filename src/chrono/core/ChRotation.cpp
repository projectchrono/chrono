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
#include "chrono/utils/ChUtils.h"

namespace chrono {

static const double FD_STEP = 1e-4;

// --------------------------

AngleSet AngleSetFromAngleSet(RotRepresentation to_seq, const AngleSet& set) {
    ChMatrix33<> R;

    switch (set.seq) {
        case RotRepresentation::EULER_ANGLES_ZXZ:
            R.SetFromEulerAnglesZXZ(set.angles);
            break;
        case RotRepresentation::CARDAN_ANGLES_ZXY:
            R.SetFromCardanAnglesZXY(set.angles);
            break;
        case RotRepresentation::CARDAN_ANGLES_ZYX:
            R.SetFromCardanAnglesZYX(set.angles);
            break;
        case RotRepresentation::CARDAN_ANGLES_XYZ:
            R.SetFromCardanAnglesXYZ(set.angles);
            break;
        default:
            std::cerr << "Unknown input angle set representation" << std::endl;
            throw std::runtime_error("Unknown input angle set representation");
            break;
    }

    ChVector3d to_angles;
    switch (to_seq) {
        case RotRepresentation::EULER_ANGLES_ZXZ:
            to_angles = R.GetEulerAnglesZXZ();
            break;
        case RotRepresentation::CARDAN_ANGLES_ZXY:
            to_angles = R.GetCardanAnglesZXY();
            break;
        case RotRepresentation::CARDAN_ANGLES_ZYX:
            to_angles = R.GetCardanAnglesZYX();
            break;
        case RotRepresentation::CARDAN_ANGLES_XYZ:
            to_angles = R.GetCardanAnglesXYZ();
            break;
        default:
            std::cerr << "Unknown output angle set representation" << std::endl;
            throw std::runtime_error("Unknown output angle set representation");
            break;
    }

    return AngleSet({to_seq, to_angles});
}

ChVector3d RodriguesFromAngleSet(const AngleSet& set) {
    ChMatrix33<> R;

    switch (set.seq) {
        case RotRepresentation::EULER_ANGLES_ZXZ:
            R.SetFromEulerAnglesZXZ(set.angles);
            break;
        case RotRepresentation::CARDAN_ANGLES_ZXY:
            R.SetFromCardanAnglesZXY(set.angles);
            break;
        case RotRepresentation::CARDAN_ANGLES_ZYX:
            R.SetFromCardanAnglesZYX(set.angles);
            break;
        case RotRepresentation::CARDAN_ANGLES_XYZ:
            R.SetFromCardanAnglesXYZ(set.angles);
            break;
        default:
            std::cerr << "Unknown input angle set representation" << std::endl;
            throw std::runtime_error("Unknown input angle set representation");
            break;
    }

    return R.GetRodriguesParameters();
}

AngleSet AngleSetFromRodrigues(RotRepresentation to_seq, const ChVector3d& params) {
    ChMatrix33<> R;
    R.SetFromRodriguesParameters(params);

    ChVector3d to_angles;
    switch (to_seq) {
        case RotRepresentation::EULER_ANGLES_ZXZ:
            to_angles = R.GetEulerAnglesZXZ();
            break;
        case RotRepresentation::CARDAN_ANGLES_ZXY:
            to_angles = R.GetCardanAnglesZXY();
            break;
        case RotRepresentation::CARDAN_ANGLES_ZYX:
            to_angles = R.GetCardanAnglesZYX();
            break;
        case RotRepresentation::CARDAN_ANGLES_XYZ:
            to_angles = R.GetCardanAnglesXYZ();
            break;
        default:
            std::cerr << "Unknown output angle set representation" << std::endl;
            throw std::runtime_error("Unknown output angle set representation");
            break;
    }

    return AngleSet({to_seq, to_angles});
}

// --------------------------

AngleAxis AngleAxisFromQuat(const ChQuaterniond& q) {
    double angle;
    ChVector3d axis;

    if (std::abs(q.e0()) < 0.99999999) {
        double arg = std::acos(q.e0());
        double invsine = 1 / std::sin(arg);
        ChVector3d vtemp;
        vtemp.x() = invsine * q.e1();
        vtemp.y() = invsine * q.e2();
        vtemp.z() = invsine * q.e3();
        angle = 2 * arg;
        axis = Vnorm(vtemp);
    } else {
        axis.x() = 1;
        axis.y() = 0;
        axis.z() = 0;
        angle = 0;
    }

    return AngleAxis({angle, axis});
}

ChQuaterniond QuatFromAngleAxis(double angle, const ChVector3d& axis) {
    ChQuaterniond q;
    double halfang;
    double sinhalf;

    halfang = angle / 2;
    sinhalf = std::sin(halfang);

    q.e0() = std::cos(halfang);
    q.e1() = axis.x() * sinhalf;
    q.e2() = axis.y() * sinhalf;
    q.e3() = axis.z() * sinhalf;

    return q;
}

ChQuaterniond QuatFromAngleAxis(const AngleAxis& angle_axis) {
    return QuatFromAngleAxis(angle_axis.angle, angle_axis.axis);
}

ChQuaterniond QuatDtFromAngleAxis(const ChQuaterniond& quat, double angle_dt, const ChVector3d& axis) {
    ChVector3d W;

    W = Vmul(axis, angle_dt);

    return QuatDtFromAngVelAbs(W, quat);
}

ChQuaterniond QuatDt2FromAngleAxis(double angle_dtdt,
                                   const ChVector3d& axis,
                                   const ChQuaterniond& q,
                                   const ChQuaterniond& q_dt) {
    ChVector3d Acc;

    Acc = Vmul(axis, angle_dtdt);

    return QuatDt2FromAngAccAbs(Acc, q, q_dt);
}

ChQuaterniond QuatFromAngleX(double angle) {
    return QuatFromAngleAxis({angle, ChVector3d(1, 0, 0)});
}

ChQuaterniond QuatFromAngleY(double angle) {
    return QuatFromAngleAxis({angle, ChVector3d(0, 1, 0)});
}

ChQuaterniond QuatFromAngleZ(double angle) {
    return QuatFromAngleAxis({angle, ChVector3d(0, 0, 1)});
}

// --------------------------

ChVector3d RotVecFromQuat(const ChQuaterniond& q) {
    return q.GetRotVec();
}

ChQuaterniond QuatFromRotVec(const ChVector3d& vec) {
    ChQuaterniond q;
    q.SetFromRotVec(vec);
    return q;
}

// --------------------------

ChVector3d RodriguesFromQuat(const ChQuaterniond& q) {
    ChMatrix33<> R(q);
    return R.GetRodriguesParameters();
}

ChQuaterniond QuatFromRodrigues(const ChVector3d& params) {
    ChMatrix33<> R;
    R.SetFromRodriguesParameters(params);
    return R.GetQuaternion();
}

ChQuaterniond QuatDtFromRodrigues(const ChVector3d& params, const ChQuaterniond& q) {
    auto params1 = RodriguesFromQuat(q);
    auto params2 = Vadd(params1, Vmul(params, FD_STEP));
    auto q2 = QuatFromRodrigues(params2);

    return Qscale(Qsub(q2, q), 1 / FD_STEP);
}

ChQuaterniond QuatDt2FromRodrigues(const ChVector3d& params, const ChQuaterniond& q) {
    auto params0 = RodriguesFromQuat(q);
    auto paramsA = Vsub(params0, Vmul(params, FD_STEP));
    auto paramsB = Vadd(params0, Vmul(params, FD_STEP));
    auto qa = QuatFromRodrigues(paramsA);
    auto qb = QuatFromRodrigues(paramsB);

    return Qscale(Qadd(Qadd(qa, qb), Qscale(q, -2)), 1 / FD_STEP);
}

// --------------------------

AngleSet AngleSetFromQuat(RotRepresentation to_seq, const ChQuaterniond& q) {
    ChMatrix33<> R(q);

    ChVector3d to_angles;
    switch (to_seq) {
        case RotRepresentation::EULER_ANGLES_ZXZ:
            to_angles = R.GetEulerAnglesZXZ();
            break;
        case RotRepresentation::CARDAN_ANGLES_ZXY:
            to_angles = R.GetCardanAnglesZXY();
            break;
        case RotRepresentation::CARDAN_ANGLES_ZYX:
            to_angles = R.GetCardanAnglesZYX();
            break;
        case RotRepresentation::CARDAN_ANGLES_XYZ:
            to_angles = R.GetCardanAnglesXYZ();
            break;
        default:
            std::cerr << "Unknown output angle set representation" << std::endl;
            throw std::runtime_error("Unknown output angle set representation");
            break;
    }

    return AngleSet({to_seq, to_angles});
}

ChQuaterniond QuatFromAngleSet(const AngleSet& set) {
    ChMatrix33<> R;

    switch (set.seq) {
        case RotRepresentation::EULER_ANGLES_ZXZ:
            R.SetFromEulerAnglesZXZ(set.angles);
            break;
        case RotRepresentation::CARDAN_ANGLES_ZXY:
            R.SetFromCardanAnglesZXY(set.angles);
            break;
        case RotRepresentation::CARDAN_ANGLES_ZYX:
            R.SetFromCardanAnglesZYX(set.angles);
            break;
        case RotRepresentation::CARDAN_ANGLES_XYZ:
            R.SetFromCardanAnglesXYZ(set.angles);
            break;
        default:
            std::cerr << "Unknown input angle set representation" << std::endl;
            throw std::runtime_error("Unknown input angle set representation");
            break;
    }

    return R.GetQuaternion();
}

ChQuaterniond QuatDtFromAngleSet(const AngleSet& set, const ChQuaterniond& q) {
    auto set1 = AngleSetFromQuat(set.seq, q);
    auto ang2 = Vadd(set1.angles, Vmul(set.angles, FD_STEP));
    auto q2 = QuatFromAngleSet({set.seq, ang2});

    return Qscale(Qsub(q2, q), 1 / FD_STEP);
}

ChQuaterniond QuatDt2FromAngleSet(const AngleSet& set, const ChQuaterniond& q) {
    auto set0 = AngleSetFromQuat(set.seq, q);
    auto angA = Vsub(set0.angles, Vmul(set.angles, FD_STEP));
    auto angB = Vadd(set0.angles, Vmul(set.angles, FD_STEP));
    auto qa = QuatFromAngleSet({set.seq, angA});
    auto qb = QuatFromAngleSet({set.seq, angB});

    return Qscale(Qadd(Qadd(qa, qb), Qscale(q, -2)), 1 / FD_STEP);
}

// --------------------------

ChQuaterniond QuatFromVec2Vec(const ChVector3d& start, const ChVector3d& end) {
    const double ANGLE_TOLERANCE = 1e-6;
    ChQuaterniond quat;
    double halfang;
    double sinhalf;
    ChVector3d axis;

    double lenXlen = start.Length() * end.Length();
    axis = start % end;
    double sinangle = ChClamp(axis.Length() / lenXlen, -1.0, +1.0);
    double cosangle = ChClamp(start ^ end / lenXlen, -1.0, +1.0);

    // Consider three cases: Parallel, Opposite, non-collinear
    if (std::abs(sinangle) == 0.0 && cosangle > 0) {
        // fr_vect & to_vect are parallel
        quat.e0() = 1.0;
        quat.e1() = 0.0;
        quat.e2() = 0.0;
        quat.e3() = 0.0;
    } else if (std::abs(sinangle) < ANGLE_TOLERANCE && cosangle < 0) {
        // fr_vect & to_vect are opposite, i.e. ~180 deg apart
        axis = start.GetOrthogonalVector() + (-end).GetOrthogonalVector();
        axis.Normalize();
        quat.e0() = 0.0;
        quat.e1() = ChClamp(axis.x(), -1.0, +1.0);
        quat.e2() = ChClamp(axis.y(), -1.0, +1.0);
        quat.e3() = ChClamp(axis.z(), -1.0, +1.0);
    } else {
        // fr_vect & to_vect are not co-linear case
        axis.Normalize();
        halfang = 0.5 * std::atan2(sinangle, cosangle);
        sinhalf = std::sin(halfang);

        quat.e0() = std::cos(halfang);
        quat.e1() = sinhalf * axis.x();
        quat.e2() = sinhalf * axis.y();
        quat.e3() = sinhalf * axis.z();
    }
    return (quat);
}

}  // end namespace chrono
