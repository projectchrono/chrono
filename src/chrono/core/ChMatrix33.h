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
// Authors: Radu Serban, Alessandro Tasora
// =============================================================================

#ifndef CHMATRIX33_H
#define CHMATRIX33_H

#include <cmath>

#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuaternion.h"

namespace chrono {

/// @addtogroup chrono_linalg
/// @{

/// Definition of a 3x3 fixed size matrix to represent 3D rotations and inertia tensors.
template <typename Real = double>
class ChMatrix33 : public Eigen::Matrix<Real, 3, 3, Eigen::RowMajor> {
  public:
    /// Default constructor: uninitialized 3x3 matrix.
    ChMatrix33() : Eigen::Matrix<Real, 3, 3, Eigen::RowMajor>() {}

    /// Constructor from Eigen expressions.
    template <typename OtherDerived>
    ChMatrix33(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix<Real, 3, 3, Eigen::RowMajor>(other) {}

    /// Construct a diagonal matrix with the specified value on the diagonal.
    ChMatrix33(Real val);

    /// Construct a diagonal matrix with the specified values on the diagonal.
    ChMatrix33(const ChVector<Real>& v);

    /// Construct a symmetric 3x3 matrix with the specified vectors for the diagonal and off-digonal elements.
    /// The off-diagonal vector is assumed to contain the elements A(0,1), A(0,2), A(1,2) in this order.
    ChMatrix33(const ChVector<>& diag, const ChVector<>& off_diag);

    /// Construct a 3x3 rotation matrix from the given quaternion.
    ChMatrix33(const ChQuaternion<Real>& q);

    /// Construct a 3x3 rotation matrix from an angle and a rotation axis.
    /// Note that the axis direction must be normalized.
    ChMatrix33(Real angle, const ChVector<>& axis);

    /// Construct a 3x3 matrix with the given vectors as columns.
    /// If the three vectors are mutually orthogonal unit vectors, the resulting matrix is a rotation matrix.
    ChMatrix33(const ChVector<>& X, const ChVector<>& Y, const ChVector<>& Z);

    /// This method allows assigning Eigen expressions to ChMatrix33.
    template <typename OtherDerived>
    ChMatrix33& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Matrix<Real, 3, 3, Eigen::RowMajor>::operator=(other);
        return *this;
    }

    /// Allows multiplying a 3x3 matrix to other Eigen matrices.
    using Eigen::Matrix<Real, 3, 3, Eigen::RowMajor>::operator*;

    /// Multiply this matrix by a 3d vector.
    ChVector<Real> operator*(const ChVector<Real>& v) const;

    /// Fill this 3x3 matrix as a rotation matrix, given a unit quaternion.
    void Set_A_quaternion(const ChQuaternion<Real>& quat);

    /// Fill this 3x3 matrix as a rotation matrix, given three Euler angles.
    void Set_A_Eulero(const ChVector<Real>& angles);

    /// Fill this 3x3 matrix as a rotation matrix, given three Cardano angles.
    void Set_A_Cardano(const ChVector<Real>& angles);

    /// Fill this 3x3 matrix as a rotation matrix, given three head, pitch, banking angles.
    void Set_A_Hpb(const ChVector<Real>& angles);

    /// Fill this 3x3 matrix as a rotation matrix, given three angles of consecutive rotations about x,y,z axis.
    void Set_A_Rxyz(const ChVector<Real>& xyz);

    /// Fill this 3x3 matrix as a rotation matrix, given three Rodriguez parameters.
    void Set_A_Rodriguez(const ChVector<Real>& rod);

    /// Fill this 3x3 matrix as a rotation matrix, given the three versors X,Y,Z of the basis.
    void Set_A_axis(const ChVector<Real>& X, const ChVector<Real>& Y, const ChVector<Real>& Z);

    /// Fill this 3x3 matrix as a rotation matrix with the X axis along the provided direction.
    /// Uses the Gram-Schmidt orthonormalization. The optional argument Vsingular, together with Xdir, suggests the XY
    /// plane (as long as Xdir is not too close to lying in that plane, in which case a different direction is
    /// selected).
    void Set_A_Xdir(const ChVector<Real>& Xdir,                                ///< X axis
                    const ChVector<Real>& Vsingular = ChVector<Real>(0, 1, 0)  ///< suggested Y axis
    );

    /// Return the versor of X axis.
    ChVector<Real> Get_A_Xaxis() const;

    /// Return the versor of Y axis.
    ChVector<Real> Get_A_Yaxis() const;

    /// Return the versor of Z axis.
    ChVector<Real> Get_A_Zaxis() const;

    /// Return the corresponding unit quaternion.
    /// Assumes that this is a rotation matrix.
    ChQuaternion<Real> Get_A_quaternion() const;

    /// Return the Eulero angles.
    /// Assumes that this is a rotation matrix.
    ChVector<Real> Get_A_Eulero() const;

    /// Return the Cardano angles.
    /// Assumes that this is a rotation matrix.
    ChVector<Real> Get_A_Cardano() const;

    /// Return the head-pitch-banking angles.
    /// Assumes that this is a rotation matrix.
    ChVector<Real> Get_A_Hpb() const;

    /// Return the angles for consecutive rotations on x,y,z axes.
    /// Assumes that this is a rotation matrix.
    ChVector<Real> Get_A_Rxyz() const;

    /// Return the Rodriguez parameters.
    /// Assumes that this is a rotation matrix.
    ChVector<Real> Get_A_Rodriguez() const;

    /// Assuming this matrix is a rotation matrix, get Ax vector.
    ChVector<Real> GetAx() const;

    /// Compute eigenvectors and eigenvalues.
    /// Note: only for self-adjoint matrices (e.g. inertia tensors).
    void SelfAdjointEigenSolve(ChMatrix33<Real>& evec, ChVectorN<Real, 3>& evals) const;
};

// -----------------------------------------------------------------------------

/// Multiply a transposed 3x3 matrix with a vector.
template <typename Real>
ChVector<Real> operator*(const Eigen::Transpose<Eigen::Matrix<Real, 3, 3, Eigen::RowMajor>>& A,
                         const ChVector<Real>& v) {
    return ChVector<Real>(A(0, 0) * v.x() + A(0, 1) * v.y() + A(0, 2) * v.z(),
                          A(1, 0) * v.x() + A(1, 1) * v.y() + A(1, 2) * v.z(),
                          A(2, 0) * v.x() + A(2, 1) * v.y() + A(2, 2) * v.z());
}

/// Multiply a transposed const 3x3 matrix with a vector.
template <typename Real>
ChVector<Real> operator*(const Eigen::Transpose<const Eigen::Matrix<Real, 3, 3, Eigen::RowMajor>>& A,
                         const ChVector<Real>& v) {
    return ChVector<Real>(A(0, 0) * v.x() + A(0, 1) * v.y() + A(0, 2) * v.z(),
                          A(1, 0) * v.x() + A(1, 1) * v.y() + A(1, 2) * v.z(),
                          A(2, 0) * v.x() + A(2, 1) * v.y() + A(2, 2) * v.z());
}

/// Return the outer product (a 3x3 matrix) of two vectors. 
template <class Real>
ChMatrix33<Real> TensorProduct(const ChVector<Real>& vA, const ChVector<Real>& vB) {
    ChMatrix33<Real> T;
    T(0, 0) = vA.x() * vB.x();
    T(0, 1) = vA.x() * vB.y();
    T(0, 2) = vA.x() * vB.z();
    T(1, 0) = vA.y() * vB.x();
    T(1, 1) = vA.y() * vB.y();
    T(1, 2) = vA.y() * vB.z();
    T(2, 0) = vA.z() * vB.x();
    T(2, 1) = vA.z() * vB.y();
    T(2, 2) = vA.z() * vB.z();
    return T;
}

// -----------------------------------------------------------------------------
// Implementation of ChMatrix33 functions
// -----------------------------------------------------------------------------

template <typename Real>
ChMatrix33<Real>::ChMatrix33(const ChQuaternion<Real>& q) {
    this->Set_A_quaternion(q);
}

template <typename Real>
ChMatrix33<Real>::ChMatrix33(Real val) {
    this->setZero();
    this->diagonal().setConstant(val);
}

template <typename Real>
ChMatrix33<Real>::ChMatrix33(const ChVector<Real>& v) {
    this->setZero();
    this->diagonal() = v.eigen();
}

template <typename Real>
ChMatrix33<Real>::ChMatrix33(const ChVector<>& diag, const ChVector<>& off_diag) {
    this->diagonal() = diag.eigen();

    (*this)(0, 1) = off_diag.x();
    (*this)(1, 0) = off_diag.x();

    (*this)(0, 2) = off_diag.y();
    (*this)(2, 0) = off_diag.y();

    (*this)(1, 2) = off_diag.z();
    (*this)(2, 1) = off_diag.z();
}

template <typename Real>
ChMatrix33<Real>::ChMatrix33(Real angle, const ChVector<>& axis) {
    ChQuaternion<Real> mr;
    mr.Q_from_AngAxis(angle, axis);
    this->Set_A_quaternion(mr);
}

template <typename Real>
ChMatrix33<Real>::ChMatrix33(const ChVector<>& X, const ChVector<>& Y, const ChVector<>& Z) {
    this->Set_A_axis(X, Y, Z);
}

template <typename Real>
ChVector<Real> ChMatrix33<Real>::operator*(const ChVector<Real>& v) const {
    return ChVector<Real>((*this)(0, 0) * v.x() + (*this)(0, 1) * v.y() + (*this)(0, 2) * v.z(),
                          (*this)(1, 0) * v.x() + (*this)(1, 1) * v.y() + (*this)(1, 2) * v.z(),
                          (*this)(2, 0) * v.x() + (*this)(2, 1) * v.y() + (*this)(2, 2) * v.z());
}

template <typename Real>
inline void ChMatrix33<Real>::Set_A_quaternion(const ChQuaternion<Real>& q) {
    Real e0e0 = q.e0() * q.e0();
    Real e1e1 = q.e1() * q.e1();
    Real e2e2 = q.e2() * q.e2();
    Real e3e3 = q.e3() * q.e3();
    Real e0e1 = q.e0() * q.e1();
    Real e0e2 = q.e0() * q.e2();
    Real e0e3 = q.e0() * q.e3();
    Real e1e2 = q.e1() * q.e2();
    Real e1e3 = q.e1() * q.e3();
    Real e2e3 = q.e2() * q.e3();

    (*this)(0, 0) = (e0e0 + e1e1) * 2 - 1;
    (*this)(0, 1) = (e1e2 - e0e3) * 2;
    (*this)(0, 2) = (e1e3 + e0e2) * 2;
    (*this)(1, 0) = (e1e2 + e0e3) * 2;
    (*this)(1, 1) = (e0e0 + e2e2) * 2 - 1;
    (*this)(1, 2) = (e2e3 - e0e1) * 2;
    (*this)(2, 0) = (e1e3 - e0e2) * 2;
    (*this)(2, 1) = (e2e3 + e0e1) * 2;
    (*this)(2, 2) = (e0e0 + e3e3) * 2 - 1;
}

template <typename Real>
inline void ChMatrix33<Real>::Set_A_Eulero(const ChVector<Real>& angles) {
    Real cx = std::cos(angles.x());
    Real cy = std::cos(angles.y());
    Real cz = std::cos(angles.z());
    Real sx = std::sin(angles.x());
    Real sy = std::sin(angles.y());
    Real sz = std::sin(angles.z());

    (*this)(0, 0) = (cz * cx) - (cy * sx * sz);
    (*this)(0, 1) = -(sz * cx) - (cy * sx * cz);
    (*this)(0, 2) = sy * sx;
    (*this)(1, 0) = (cz * sx) + (cy * cx * sz);
    (*this)(1, 1) = -(sz * sx) + (cy * cx * cz);
    (*this)(1, 2) = -sy * cx;
    (*this)(2, 0) = sy * sz;
    (*this)(2, 1) = sy * cz;
    (*this)(2, 2) = cy;
}

template <typename Real>
inline void ChMatrix33<Real>::Set_A_Cardano(const ChVector<Real>& angles) {
    Real cx = std::cos(angles.x());
    Real cy = std::cos(angles.y());
    Real cz = std::cos(angles.z());
    Real sx = std::sin(angles.x());
    Real sy = std::sin(angles.y());
    Real sz = std::sin(angles.z());

    (*this)(0, 0) = (cx * cz) - (sz * sx * sy);
    (*this)(0, 1) = -sx * cy;
    (*this)(0, 2) = (cx * sz) + (sx * sy * cz);
    (*this)(1, 0) = (sx * cz) + (cx * sy * sz);
    (*this)(1, 1) = cy * cx;
    (*this)(1, 2) = (sx * sz) - (cx * sy * cz);
    (*this)(2, 0) = -sz * cy;
    (*this)(2, 1) = sy;
    (*this)(2, 2) = cy * cz;
}

template <typename Real>
inline void ChMatrix33<Real>::Set_A_Hpb(const ChVector<Real>& angles) {
    Real cx = std::cos(angles.y());
    Real cy = std::cos(angles.x());
    Real cz = std::cos(angles.z());
    Real sx = std::sin(angles.y());
    Real sy = std::sin(angles.x());
    Real sz = std::sin(angles.z());

    (*this)(0, 0) = (cz * cy) - (sz * sx * sy);
    (*this)(0, 1) = -(sz * cy) - (cz * sx * sy);
    (*this)(0, 2) = -cx * sy;
    (*this)(1, 0) = sz * cx;
    (*this)(1, 1) = cz * cx;
    (*this)(1, 2) = -sx;
    (*this)(2, 0) = (cz * sy) + (sz * sx * cy);
    (*this)(2, 1) = -(sz * sy) + (cz * sx * cy);
    (*this)(2, 2) = cx * cy;
}

template <typename Real>
inline void ChMatrix33<Real>::Set_A_Rxyz(const ChVector<Real>& xyz) {
    Real cx = std::cos(xyz.x());
    Real cy = std::cos(xyz.y());
    Real cz = std::cos(xyz.z());
    Real sx = std::sin(xyz.x());
    Real sy = std::sin(xyz.y());
    Real sz = std::sin(xyz.z());

    (*this)(0, 0) = cy * cz;
    (*this)(0, 1) = cy * sz;
    (*this)(0, 2) = -sy;
    (*this)(1, 0) = (sx * sy * cz) - (cx * sz);
    (*this)(1, 1) = (sx * sy * sz) + (cx * cz);
    (*this)(1, 2) = sx * cy;
    (*this)(2, 0) = (cx * sy * cz) + (sx * sz);
    (*this)(2, 1) = (cx * sy * sz) - (sx * cz);
    (*this)(2, 2) = cx * cy;
}

template <typename Real>
inline void ChMatrix33<Real>::Set_A_Rodriguez(const ChVector<Real>& rod) {
    Real gam = std::pow(rod.x(), 2) + std::pow(rod.y(), 2) + std::pow(rod.z(), 2);

    (*this)(0, 0) = 1 + std::pow(rod.x(), 2) - std::pow(rod.y(), 2) - std::pow(rod.z(), 2);
    (*this)(0, 1) = 2 * (rod.x() * rod.y() - rod.z());
    (*this)(0, 2) = 2 * (rod.x() * rod.z() + rod.y());
    (*this)(1, 0) = 2 * (rod.x() * rod.y() + rod.z());
    (*this)(1, 1) = 1 - std::pow(rod.x(), 2) + std::pow(rod.y(), 2) - std::pow(rod.z(), 2);
    (*this)(1, 2) = 2 * (rod.y() * rod.z() - rod.x());
    (*this)(2, 0) = 2 * (rod.x() * rod.z() - rod.y());
    (*this)(2, 1) = 2 * (rod.y() * rod.z() + rod.x());
    (*this)(2, 2) = 1 - std::pow(rod.x(), 2) - std::pow(rod.y(), 2) + std::pow(rod.z(), 2);

    *this *= 1 / (1 + gam);
}

template <typename Real>
inline void ChMatrix33<Real>::Set_A_axis(const ChVector<Real>& X, const ChVector<Real>& Y, const ChVector<Real>& Z) {
    (*this)(0, 0) = X.x();
    (*this)(0, 1) = Y.x();
    (*this)(0, 2) = Z.x();
    (*this)(1, 0) = X.y();
    (*this)(1, 1) = Y.y();
    (*this)(1, 2) = Z.y();
    (*this)(2, 0) = X.z();
    (*this)(2, 1) = Y.z();
    (*this)(2, 2) = Z.z();
}

template <typename Real>
inline void ChMatrix33<Real>::Set_A_Xdir(const ChVector<Real>& Xdir, const ChVector<Real>& Vsingular) {
    ChVector<Real> mX;
    ChVector<Real> mY;
    ChVector<Real> mZ;
    Xdir.DirToDxDyDz(mX, mY, mZ, Vsingular);
    this->Set_A_axis(mX, mY, mZ);
}

template <typename Real>
inline ChVector<Real> ChMatrix33<Real>::Get_A_Eulero() const {
    ChVector<Real> eul;

    eul.y() = std::acos((*this)(2, 2));                       // rho, nutation
    eul.z() = std::acos((*this)(2, 1) / std::sin(eul.y()));   // csi, spin
    eul.x() = std::acos(-(*this)(1, 2) / std::sin(eul.y()));  // rho, nutation

    if (eul.y() == 0) {  // handle undefined initial position set
        eul.x() = 0;
        eul.z() = 0;
    }

    return eul;
}

template <typename Real>
inline ChVector<Real> ChMatrix33<Real>::Get_A_Cardano() const {
    ChVector<Real> car;

    Real mel21 = (*this)(2, 1);
    if (mel21 > 1)
        mel21 = 1;
    if (mel21 < -1)
        mel21 = -1;

    car.y() = std::asin(mel21);

    Real arg2 = (*this)(2, 2) / std::cos(car.y());
    if (arg2 > 1)
        arg2 = 1;
    if (arg2 < -1)
        arg2 = -1;
    Real arg3 = (*this)(1, 1) / std::cos(car.y());
    if (arg3 > 1)
        arg3 = 1;
    if (arg3 < -1)
        arg3 = -1;

    car.z() = std::acos(arg2);
    car.x() = std::acos(arg3);

    return car;
}

template <typename Real>
inline ChVector<Real> ChMatrix33<Real>::Get_A_Hpb() const {
    ChVector<Real> Hpb;

    Real arg1 = -(*this)(1, 2);
    if (arg1 > 1)
        arg1 = 1;
    if (arg1 < -1)
        arg1 = -1;

    Hpb.y() = std::asin(arg1);  // P

    Real arg2 = (*this)(2, 2) / std::cos(Hpb.y());
    if (arg2 > 1)
        arg2 = 1;
    if (arg2 < -1)
        arg2 = -1;
    Real arg3 = (*this)(1, 1) / std::cos(Hpb.y());
    if (arg3 > 1)
        arg3 = 1;
    if (arg3 < -1)
        arg3 = -1;

    Hpb.x() = std::acos(arg2);  // H
    Hpb.z() = std::acos(arg3);  // B

    return Hpb;
}

template <typename Real>
inline ChVector<Real> ChMatrix33<Real>::Get_A_Rxyz() const {
    ChVector<Real> Rxyz;

    Real arg1 = -(*this)(0, 2);
    if (arg1 > 1)
        arg1 = 1;
    if (arg1 < -1)
        arg1 = -1;

    Rxyz.y() = std::asin(arg1);

    Real arg2 = (*this)(0, 1) / std::cos(Rxyz.y());
    if (arg2 > 1)
        arg2 = 1;
    if (arg2 < -1)
        arg2 = -1;
    Real arg3 = (*this)(1, 2) / std::cos(Rxyz.y());
    if (arg3 > 1)
        arg3 = 1;
    if (arg3 < -1)
        arg3 = -1;

    Rxyz.z() = std::asin(arg2);
    Rxyz.x() = std::asin(arg3);

    return Rxyz;
}

template <typename Real>
inline ChVector<Real> ChMatrix33<Real>::Get_A_Rodriguez() const {
    ChVector<Real> rod;
    ChQuaternion<Real> qtemp = Get_A_quaternion();
    // warning: infinite results may happen..
    rod.x() = qtemp.e1() / qtemp.e0();
    rod.y() = qtemp.e2() / qtemp.e0();
    rod.z() = qtemp.e3() / qtemp.e0();

    return rod;
}

template <typename Real>
inline ChQuaternion<Real> ChMatrix33<Real>::Get_A_quaternion() const {
    ChQuaternion<Real> q;
    Real s, tr;
    Real half = (Real)0.5;

    Real m00 = (*this)(0, 0);
    Real m01 = (*this)(0, 1);
    Real m02 = (*this)(0, 2);
    Real m10 = (*this)(1, 0);
    Real m11 = (*this)(1, 1);
    Real m12 = (*this)(1, 2);
    Real m20 = (*this)(2, 0);
    Real m21 = (*this)(2, 1);
    Real m22 = (*this)(2, 2);

    tr = m00 + m11 + m22;  // diag sum

    if (tr >= 0) {
        s = std::sqrt(tr + 1);
        q.e0() = half * s;
        s = half / s;
        q.e1() = (m21 - m12) * s;
        q.e2() = (m02 - m20) * s;
        q.e3() = (m10 - m01) * s;
    } else {
        int i = 0;

        if (m11 > m00) {
            i = 1;
            if (m22 > m11)
                i = 2;
        } else {
            if (m22 > m00)
                i = 2;
        }

        switch (i) {
            case 0:
                s = std::sqrt(m00 - m11 - m22 + 1);
                q.e1() = half * s;
                s = half / s;
                q.e2() = (m01 + m10) * s;
                q.e3() = (m20 + m02) * s;
                q.e0() = (m21 - m12) * s;
                break;
            case 1:
                s = std::sqrt(m11 - m22 - m00 + 1);
                q.e2() = half * s;
                s = half / s;
                q.e3() = (m12 + m21) * s;
                q.e1() = (m01 + m10) * s;
                q.e0() = (m02 - m20) * s;
                break;
            case 2:
                s = std::sqrt(m22 - m00 - m11 + 1);
                q.e3() = half * s;
                s = half / s;
                q.e1() = (m20 + m02) * s;
                q.e2() = (m12 + m21) * s;
                q.e0() = (m10 - m01) * s;
                break;
        }
    }

    return q;
}

template <typename Real>
inline ChVector<Real> ChMatrix33<Real>::Get_A_Xaxis() const {
    ChVector<Real> X;
    X.x() = (*this)(0, 0);
    X.y() = (*this)(1, 0);
    X.z() = (*this)(2, 0);
    return X;
}

template <typename Real>
inline ChVector<Real> ChMatrix33<Real>::Get_A_Yaxis() const {
    ChVector<Real> Y;
    Y.x() = (*this)(0, 1);
    Y.y() = (*this)(1, 1);
    Y.z() = (*this)(2, 1);
    return Y;
}

template <typename Real>
inline ChVector<Real> ChMatrix33<Real>::Get_A_Zaxis() const {
    ChVector<Real> Z;
    Z.x() = (*this)(0, 2);
    Z.y() = (*this)(1, 2);
    Z.z() = (*this)(2, 2);
    return Z;
}

template <typename Real>
inline ChVector<Real> ChMatrix33<Real>::GetAx() const {
    return ChVector<Real>(0.5 * ((*this)(2, 1) - (*this)(1, 2)),  //
                          0.5 * ((*this)(0, 2) - (*this)(2, 0)),  //
                          0.5 * ((*this)(1, 0) - (*this)(0, 1)));
}

template <typename Real>
inline void ChMatrix33<Real>::SelfAdjointEigenSolve(ChMatrix33<Real>& evec, ChVectorN<Real, 3>& evals) const {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Real, 3, 3, Eigen::RowMajor>> es(*this);
    evals = es.eigenvalues();
    evec = es.eigenvectors();
}

/// @} chrono_linalg

}  // end namespace chrono

#endif
