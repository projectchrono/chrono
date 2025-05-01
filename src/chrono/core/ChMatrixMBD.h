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
// Authors: Radu Serban
// =============================================================================
//
// Definition of special Multi-Body Dynamics 3x4, 4x3, and 4x4 matrices.
//
// =============================================================================

#ifndef CH_MATRIX_MBD_H
#define CH_MATRIX_MBD_H

#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChCoordsys.h"

namespace chrono {

/// @addtogroup chrono_linalg
/// @{

// =============================================================================

/// 3x4 dense matrix (fixed-size, row-major ordering)
template <typename T = double>
using ChMatrix34 = Eigen::Matrix<T, 3, 4, Eigen::RowMajor>;

/// 4x3 dense matrix (fixed-size, row-major ordering)
template <typename T = double>
using ChMatrix43 = Eigen::Matrix<T, 4, 3, Eigen::ColMajor>;

/// 4x4 dense matrix (fixed-size, row-major ordering)
template <typename T = double>
using ChMatrix44 = Eigen::Matrix<T, 4, 4, Eigen::RowMajor>;

// =============================================================================

/// Special MBD 3x4 matrix [Fp(q)], as in  [Fp(q)] * [Fm(q)]' = [A(q)]
template <typename Real = double>
class ChFpMatrix34 : public ChMatrix34<Real> {
  public:
    ChFpMatrix34(const ChQuaternion<Real>& q) {
        (*this)(0, 0) = q.e1();
        (*this)(0, 1) = q.e0();
        (*this)(0, 2) = -q.e3();
        (*this)(0, 3) = q.e2();
        (*this)(1, 0) = q.e2();
        (*this)(1, 1) = q.e3();
        (*this)(1, 2) = q.e0();
        (*this)(1, 3) = -q.e1();
        (*this)(2, 0) = q.e3();
        (*this)(2, 1) = -q.e2();
        (*this)(2, 2) = q.e1();
        (*this)(2, 3) = q.e0();
    }

    // Allows multiplying to other Eigen matrices.
    using Eigen::Matrix<Real, 3, 4, Eigen::RowMajor>::operator*;
};

/// Special MBD 3x4 matrix [Fm(q)], as in  [Fp(q)] * [Fm(q)]' = [A(q)]
template <typename Real = double>
class ChFmMatrix34 : public ChMatrix34<Real> {
  public:
    ChFmMatrix34(const ChQuaternion<Real>& q) {
        (*this)(0, 0) = q.e1();
        (*this)(0, 1) = q.e0();
        (*this)(0, 2) = q.e3();
        (*this)(0, 3) = -q.e2();
        (*this)(1, 0) = q.e2();
        (*this)(1, 1) = -q.e3();
        (*this)(1, 2) = q.e0();
        (*this)(1, 3) = q.e1();
        (*this)(2, 0) = q.e3();
        (*this)(2, 1) = q.e2();
        (*this)(2, 2) = -q.e1();
        (*this)(2, 3) = q.e0();
    }

    // Allows multiplying to other Eigen matrices.
    using Eigen::Matrix<Real, 3, 4, Eigen::RowMajor>::operator*;
};

/// Special MBD 3x4 matrix [Gl(q)], as in local angular speed conversion.
/// Wl = [Gl] * q_dt   (also, [Gl(q)] = 2*[Fp(q')] = 2*[G] with G matrix as in Shabana)
template <typename Real = double>
class ChGlMatrix34 : public ChMatrix34<Real> {
  public:
    ChGlMatrix34(const ChQuaternion<Real>& q) {
        Real de0 = 2 * q.e0();
        Real de1 = 2 * q.e1();
        Real de2 = 2 * q.e2();
        Real de3 = 2 * q.e3();
        (*this)(0, 0) = -de1;
        (*this)(0, 1) = de0;
        (*this)(0, 2) = de3;
        (*this)(0, 3) = -de2;
        (*this)(1, 0) = -de2;
        (*this)(1, 1) = -de3;
        (*this)(1, 2) = de0;
        (*this)(1, 3) = de1;
        (*this)(2, 0) = -de3;
        (*this)(2, 1) = de2;
        (*this)(2, 2) = -de1;
        (*this)(2, 3) = de0;
    }

    // Allows multiplying to other Eigen matrices.
    using Eigen::Matrix<Real, 3, 4, Eigen::RowMajor>::operator*;

    /// Computes the product v=[Gl(mq)]*qb  without the need of having
    /// the [Gl] matrix (just pass the mq quaternion, since Gl is function of mq)
    static ChVector3<Real> Gl_times_q(const ChQuaternion<Real>& mq, const ChQuaternion<Real>& q) {
        Real de0 = 2 * mq.e0();
        Real de1 = 2 * mq.e1();
        Real de2 = 2 * mq.e2();
        Real de3 = 2 * mq.e3();
        return ChVector3<Real>(-de1 * q.e0() + de0 * q.e1() + de3 * q.e2() - de2 * q.e3(),   //
                               -de2 * q.e0() - de3 * q.e1() + de0 * q.e2() + de1 * q.e3(),   //
                               -de3 * q.e0() + de2 * q.e1() - de1 * q.e2() + de0 * q.e3());  //
    }

    /// Computes the product q=[Gl(mq)]*v  without the need of having
    /// the [Gl] matrix (just pass the mq quaternion, since Gl is function of mq)
    static ChQuaternion<Real> GlT_times_v(const ChQuaternion<Real>& mq, const ChVector3<Real>& v) {
        Real de0 = 2 * mq.e0();
        Real de1 = 2 * mq.e1();
        Real de2 = 2 * mq.e2();
        Real de3 = 2 * mq.e3();
        return ChQuaternion<Real>(-de1 * v.x() - de2 * v.y() - de3 * v.z(),   //
                                  +de0 * v.x() - de3 * v.y() + de2 * v.z(),   //
                                  +de3 * v.x() + de0 * v.y() - de1 * v.z(),   //
                                  -de2 * v.x() + de1 * v.y() + de0 * v.z());  //
    }
};

/// Special MBD 3x4 matrix [Gw(q)], as in absolute angular speed conversion.
/// Ww = [Gw] * q_dt   (also, [Gw(q)] = 2*[Fm(q')] = 2*[E] with E matrix as in Shabana)
template <typename Real = double>
class ChGwMatrix34 : public ChMatrix34<Real> {
  public:
    ChGwMatrix34(const ChQuaternion<Real>& q) {
        Real de0 = 2 * q.e0();
        Real de1 = 2 * q.e1();
        Real de2 = 2 * q.e2();
        Real de3 = 2 * q.e3();
        (*this)(0, 0) = -de1;
        (*this)(0, 1) = de0;
        (*this)(0, 2) = -de3;
        (*this)(0, 3) = de2;
        (*this)(1, 0) = -de2;
        (*this)(1, 1) = de3;
        (*this)(1, 2) = de0;
        (*this)(1, 3) = -de1;
        (*this)(2, 0) = -de3;
        (*this)(2, 1) = -de2;
        (*this)(2, 2) = de1;
        (*this)(2, 3) = de0;
    }

    // Allows multiplying to other Eigen matrices.
    using Eigen::Matrix<Real, 3, 4, Eigen::RowMajor>::operator*;
};

/// Special MBD 3x3 "star" matrix, , representing vector cross products. \n
/// (1) given two 3d vectors a and b, a x b = [Astar(a)] * b.             \n
/// (2) double cross product.
template <typename Real = double>
class ChStarMatrix33 : public Eigen::Matrix<Real, 3, 3, Eigen::RowMajor> {
  public:
    /// Construct a 3x3 "star matrix" (aka "tilde matrix") for matrix form of cross product.
    /// If a and b are 3d vectors, then a x b = [Astar(a)] * b.
    ChStarMatrix33(const ChVector3<Real>& v) {
        (*this)(0, 0) = 0;
        (*this)(0, 1) = -v.z();
        (*this)(0, 2) = v.y();
        (*this)(1, 0) = v.z();
        (*this)(1, 1) = 0;
        (*this)(1, 2) = -v.x();
        (*this)(2, 0) = -v.y();
        (*this)(2, 1) = v.x();
        (*this)(2, 2) = 0;
    }

    /// Construct a 3x3 "star matrix" representing a double cross product.
    ChStarMatrix33(const ChVector3<Real>& vA, const ChVector3<Real>& vB) {
        (*this)(0, 0) = -vA.y() * vB.y() - vA.z() * vB.z();
        (*this)(1, 0) = vA.x() * vB.y();
        (*this)(2, 0) = vA.x() * vB.z();
        (*this)(0, 1) = vA.y() * vB.x();
        (*this)(1, 1) = -vA.z() * vB.z() - vA.x() * vB.x();
        (*this)(2, 1) = vA.y() * vB.z();
        (*this)(0, 2) = vA.z() * vB.x();
        (*this)(1, 2) = vA.z() * vB.y();
        (*this)(2, 2) = -vA.x() * vB.x() - vA.y() * vB.y();
    }

    /// Allows multiplying to other Eigen matrices.
    using Eigen::Matrix<Real, 3, 3, Eigen::RowMajor>::operator*;

    /// Multiply this matrix by a 3d vector.
    ChVector3<Real> operator*(const ChVector3<Real>& v) const {
        return ChVector3<Real>((*this)(0, 0) * v.x() + (*this)(0, 1) * v.y() + (*this)(0, 2) * v.z(),
                               (*this)(1, 0) * v.x() + (*this)(1, 1) * v.y() + (*this)(1, 2) * v.z(),
                               (*this)(2, 0) * v.x() + (*this)(2, 1) * v.y() + (*this)(2, 2) * v.z());
    }
};

/// Alias for a 3x3 star matrix of doubles.
using ChStarMatrix33d = ChStarMatrix33<double>;

/// Special MBD 4x4 "star" matrix, representing quaternion cross product.
/// That is, given two quaternions a and b, a x b= [Astar] * b
template <typename Real = double>
class ChStarMatrix44 : public ChMatrix44<Real> {
  public:
    /// Constructor from a given quaternion.
    ChStarMatrix44(const ChQuaternion<Real>& q) {
        (*this)(0, 0) = q.e0();
        (*this)(0, 1) = -q.e1();
        (*this)(0, 2) = -q.e2();
        (*this)(0, 3) = -q.e3();
        (*this)(1, 0) = q.e1();
        (*this)(1, 1) = q.e0();
        (*this)(1, 2) = -q.e3();
        (*this)(1, 3) = q.e2();
        (*this)(2, 0) = q.e2();
        (*this)(2, 1) = q.e3();
        (*this)(2, 2) = q.e0();
        (*this)(2, 3) = -q.e1();
        (*this)(3, 0) = q.e3();
        (*this)(3, 1) = -q.e2();
        (*this)(3, 2) = q.e1();
        (*this)(3, 3) = q.e0();
    }

    /// Transposes only the lower-right 3x3 submatrix of a hemisymmetric 4x4 matrix,
    /// used when the 4x4 matrix is a "star" matrix [q] coming from a quaternion q:
    /// the non commutative quaternion product is:
    ///     q1 x q2  =  [q1]*q2  =  [q2st]*q1
    /// where [q2st] is the "semi-transpose" of [q2].
    void semiTranspose() {
        (*this)(1, 2) *= -1;
        (*this)(1, 3) *= -1;
        (*this)(2, 1) *= -1;
        (*this)(2, 3) *= -1;
        (*this)(3, 1) *= -1;
        (*this)(3, 2) *= -1;
    }

    /// Change the sign of the 2nd, 3rd and 4th columns of this matrix.
    /// The product between a quaternion q1 and the conjugate of q2 (q2'), is:
    ///    q1 x q2'  = [q1]*q2'   = [q1sn]*q2
    /// where [q1sn] is the semi-negation of the 4x4 matrix [q1].
    void semiNegate() { this->template rightCols<3>() *= -1; }
};

// =============================================================================
// Operations with 3x4 matrices

/// Multiply a 3x4 matrix with a quaternion and return a 3d vector.
template <typename T, typename U>
ChVector3<T> operator*(const ChMatrix34<T>& A, const ChQuaternion<U>& q) {
    return ChVector3<T>(A(0, 0) * (T)q.e0() + A(0, 1) * (T)q.e1() + A(0, 2) * (T)q.e2() + A(0, 3) * (T)q.e3(),
                        A(1, 0) * (T)q.e0() + A(1, 1) * (T)q.e1() + A(1, 2) * (T)q.e2() + A(1, 3) * (T)q.e3(),
                        A(2, 0) * (T)q.e0() + A(2, 1) * (T)q.e1() + A(2, 2) * (T)q.e2() + A(2, 3) * (T)q.e3());
}

/// Multiply a 4x3 matrix with a 3d vector and return a quaternion.
template <typename T, typename U>
ChQuaternion<T> operator*(const ChMatrix43<T>& A, const ChVector3<U>& v) {
    return ChQuaternion<T>(A(0, 0) * (T)v.x() + A(0, 1) * (T)v.y() + A(0, 2) * (T)v.z(),
                           A(1, 0) * (T)v.x() + A(1, 1) * (T)v.y() + A(1, 2) * (T)v.z(),
                           A(2, 0) * (T)v.x() + A(2, 1) * (T)v.y() + A(2, 2) * (T)v.z(),
                           A(3, 0) * (T)v.x() + A(3, 1) * (T)v.y() + A(3, 2) * (T)v.z());
}

/// Multiply the transpose of a 3x4 matrix with a 3d vector and return a quaternion.
template <typename T, typename U>
ChQuaternion<T> operator*(const Eigen::Transpose<Eigen::Matrix<T, 3, 4, Eigen::RowMajor>>& A, const ChVector3<U>& v) {
    return ChQuaternion<T>(A(0, 0) * (T)v.x() + A(0, 1) * (T)v.y() + A(0, 2) * (T)v.z(),
                           A(1, 0) * (T)v.x() + A(1, 1) * (T)v.y() + A(1, 2) * (T)v.z(),
                           A(2, 0) * (T)v.x() + A(2, 1) * (T)v.y() + A(2, 2) * (T)v.z(),
                           A(3, 0) * (T)v.x() + A(3, 1) * (T)v.y() + A(3, 2) * (T)v.z());
}

/// Multiply a 4x4 matrix with a quaternion and return a quaternion.
template <typename T, typename U>
ChQuaternion<T> operator*(const ChMatrix44<T>& A, const ChQuaternion<U>& q) {
    return ChQuaternion<T>(A(0, 0) * (T)q.e0() + A(0, 1) * (T)q.e1() + A(0, 2) * (T)q.e2() + A(0, 3) * (T)q.e3(),
                           A(1, 0) * (T)q.e0() + A(1, 1) * (T)q.e1() + A(1, 2) * (T)q.e2() + A(1, 3) * (T)q.e3(),
                           A(2, 0) * (T)q.e0() + A(2, 1) * (T)q.e1() + A(2, 2) * (T)q.e2() + A(2, 3) * (T)q.e3(),
                           A(3, 0) * (T)q.e0() + A(3, 1) * (T)q.e1() + A(3, 2) * (T)q.e2() + A(3, 3) * (T)q.e3());
}

/// @} chrono_linalg

}  // end namespace chrono

#endif
