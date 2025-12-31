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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include <cmath>

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/utils/ChUtils.h"

namespace chrono {

const ChQuaterniond QNULL(0., 0., 0., 0.);
const ChQuaterniond QUNIT(1., 0., 0., 0.);

const ChQuaterniond Q_ROTATE_Y_TO_X(1 / CH_SQRT_2, 0, 0, -1 / CH_SQRT_2);
const ChQuaterniond Q_ROTATE_Y_TO_Z(1 / CH_SQRT_2, 1 / CH_SQRT_2, 0, 0);
const ChQuaterniond Q_ROTATE_X_TO_Y(1 / CH_SQRT_2, 0, 0, 1 / CH_SQRT_2);
const ChQuaterniond Q_ROTATE_X_TO_Z(1 / CH_SQRT_2, 0, -1 / CH_SQRT_2, 0);
const ChQuaterniond Q_ROTATE_Z_TO_Y(1 / CH_SQRT_2, -1 / CH_SQRT_2, 0, 0);
const ChQuaterniond Q_ROTATE_Z_TO_X(1 / CH_SQRT_2, 0, 1 / CH_SQRT_2, 0);

const ChQuaterniond Q_FLIP_AROUND_X(0., 1., 0., 0.);
const ChQuaterniond Q_FLIP_AROUND_Y(0., 0., 1., 0.);
const ChQuaterniond Q_FLIP_AROUND_Z(0., 0., 0., 1.);

static const double FD_STEP = 1e-4;

// -----------------------------------------------------------------------------

// Check if two quaternions are equal
bool Qequal(const ChQuaterniond& qa, const ChQuaterniond& qb) {
    return qa == qb;
}

// Check if quaternion is not null
bool Qnotnull(const ChQuaterniond& qa) {
    return (qa.e0() != 0) || (qa.e1() != 0) || (qa.e2() != 0) || (qa.e3() != 0);
}

double Qlength(const ChQuaterniond& q) {
    return (std::sqrt(std::pow(q.e0(), 2) + std::pow(q.e1(), 2) + std::pow(q.e2(), 2) + std::pow(q.e3(), 2)));
}

ChQuaterniond Qscale(const ChQuaterniond& q, double fact) {
    ChQuaterniond result;
    result.e0() = q.e0() * fact;
    result.e1() = q.e1() * fact;
    result.e2() = q.e2() * fact;
    result.e3() = q.e3() * fact;
    return result;
}

ChQuaterniond Qadd(const ChQuaterniond& qa, const ChQuaterniond& qb) {
    ChQuaterniond result;
    result.e0() = qa.e0() + qb.e0();
    result.e1() = qa.e1() + qb.e1();
    result.e2() = qa.e2() + qb.e2();
    result.e3() = qa.e3() + qb.e3();
    return result;
}

ChQuaterniond Qsub(const ChQuaterniond& qa, const ChQuaterniond& qb) {
    ChQuaterniond result;
    result.e0() = qa.e0() - qb.e0();
    result.e1() = qa.e1() - qb.e1();
    result.e2() = qa.e2() - qb.e2();
    result.e3() = qa.e3() - qb.e3();
    return result;
}

// Return the norm two of the quaternion. Euler's parameters have norm = 1
ChQuaterniond Qnorm(const ChQuaterniond& q) {
    double invlength;
    invlength = 1 / (Qlength(q));
    return Qscale(q, invlength);
}

// Return the conjugate of the quaternion [s,v1,v2,v3] is [s,-v1,-v2,-v3]
ChQuaterniond Qconjugate(const ChQuaterniond& q) {
    ChQuaterniond res;
    res.e0() = q.e0();
    res.e1() = -q.e1();
    res.e2() = -q.e2();
    res.e3() = -q.e3();
    return (res);
}

// Return the product of two quaternions. It is non-commutative (like cross product in vectors).
ChQuaterniond Qcross(const ChQuaterniond& qa, const ChQuaterniond& qb) {
    ChQuaterniond res;
    res.e0() = qa.e0() * qb.e0() - qa.e1() * qb.e1() - qa.e2() * qb.e2() - qa.e3() * qb.e3();
    res.e1() = qa.e0() * qb.e1() + qa.e1() * qb.e0() - qa.e3() * qb.e2() + qa.e2() * qb.e3();
    res.e2() = qa.e0() * qb.e2() + qa.e2() * qb.e0() + qa.e3() * qb.e1() - qa.e1() * qb.e3();
    res.e3() = qa.e0() * qb.e3() + qa.e3() * qb.e0() - qa.e2() * qb.e1() + qa.e1() * qb.e2();
    return (res);
}

// -----------------------------------------------------------------------------

// Get the quaternion time derivative from the vector of angular speed, with w specified in _absolute_ coords.
ChQuaterniond QuatDtFromAngVelAbs(const ChVector3d& w, const ChQuaterniond& q) {
    ChQuaterniond qw;
    double half = 0.5;

    qw.e0() = 0;
    qw.e1() = w.x();
    qw.e2() = w.y();
    qw.e3() = w.z();

    return Qscale(Qcross(qw, q), half);  // {q_dt} = 1/2 {0,w}*{q}
}

// Get the quaternion time derivative from the vector of angular speed, with w specified in _local_ coords.
ChQuaterniond QuatDtFromAngVelRel(const ChVector3d& w, const ChQuaterniond& q) {
    ChQuaterniond qw;
    double half = 0.5;

    qw.e0() = 0;
    qw.e1() = w.x();
    qw.e2() = w.y();
    qw.e3() = w.z();

    return Qscale(Qcross(q, qw), half);  // {q_dt} = 1/2 {q}*{0,w_rel}
}

// Get the quaternion first derivative from the vector of angular acceleration with a specified in _absolute_ coords.
ChQuaterniond QuatDt2FromAngAccAbs(const ChVector3d& a, const ChQuaterniond& q, const ChQuaterniond& q_dt) {
    ChQuaterniond ret;
    ret.SetDt2FromAngAccAbs(a, q, q_dt);
    return ret;
}

//	Get the quaternion second derivative from the vector of angular acceleration with a specified in _relative_ coords.
ChQuaterniond QuatDt2FromAngAccRel(const ChVector3d& a, const ChQuaterniond& q, const ChQuaterniond& q_dt) {
    ChQuaterniond ret;
    ret.SetDt2FromAngAccRel(a, q, q_dt);
    return ret;
}

// -----------------------------------------------------------------------------

// Given the imaginary (vectorial) {e1 e2 e3} part of a quaternion,
// find the entire quaternion q = {e0, e1, e2, e3}.
// Note: singularities are possible.
ChQuaterniond QuatFromImaginary(const ChVector3d& im) {
    ChQuaterniond q;
    q.e1() = im.x();
    q.e2() = im.y();
    q.e3() = im.z();
    q.e0() = std::sqrt(1 - q.e1() * q.e1() - q.e2() * q.e2() - q.e3() * q.e3());
    return q;
}

// Given the imaginary (vectorial) {e1 e2 e3} part of a quaternion time derivative,
// find the entire quaternion q = {e0, e1, e2, e3}.
// Note: singularities are possible.
ChQuaterniond QuatDtFromImaginary(const ChVector3d& im_dt, const ChQuaterniond& q) {
    ChQuaterniond q_dt;
    q_dt.e1() = im_dt.x();
    q_dt.e2() = im_dt.y();
    q_dt.e3() = im_dt.z();
    q_dt.e0() = (-q.e1() * q_dt.e1() - q.e2() * q_dt.e2() - q.e3() * q_dt.e3()) / q.e0();
    return q_dt;
}

// Given the imaginary (vectorial) {e1 e2 e3} part of a quaternion second time derivative,
// find the entire quaternion q = {e0, e1, e2, e3}.
// Note: singularities are possible.
ChQuaterniond QuatDt2FromImaginary(const ChVector3d& im_dtdt, const ChQuaterniond& q, const ChQuaterniond& q_dt) {
    ChQuaterniond q_dtdt;
    q_dtdt.e1() = im_dtdt.x();
    q_dtdt.e2() = im_dtdt.y();
    q_dtdt.e3() = im_dtdt.z();
    q_dtdt.e0() = (-q.e1() * q_dtdt.e1() - q.e2() * q_dtdt.e2() - q.e3() * q_dtdt.e3()                               //
                   - q_dt.e0() * q_dt.e0() - q_dt.e1() * q_dt.e1() - q_dt.e2() * q_dt.e2() - q_dt.e3() * q_dt.e3())  //
                  / q.e0();                                                                                          //
    return q_dtdt;
}

// -----------------------------------------------------------------------------

// Get the X axis of a coordsystem, given the quaternion which
// represents the alignment of the coordsystem.
ChVector3d VaxisXfromQuat(const ChQuaterniond& quat) {
    ChVector3d res;
    res.x() = (std::pow(quat.e0(), 2) + std::pow(quat.e1(), 2)) * 2 - 1;
    res.y() = ((quat.e1() * quat.e2()) + (quat.e0() * quat.e3())) * 2;
    res.z() = ((quat.e1() * quat.e3()) - (quat.e0() * quat.e2())) * 2;
    return res;
}

}  // end namespace chrono
