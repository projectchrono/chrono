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

#ifndef CHMATRIX33_H
#define CHMATRIX33_H

#include "chrono/core/ChMatrixNM.h"

namespace chrono {

/// ChMatrix33
///
/// A special type of NxM matrix: the 3x3 matrix that is commonly used
/// to represent coordinate transformations in 3D space.
/// This matrix cannot be resized.
/// The 3x3 matrix can be multiplied/added with other matrix types.
///
/// Further info at the @ref manual_ChMatrix33 manual page.

template <class Real = double>
class ChMatrix33 : public ChMatrixNM<Real, 3, 3> {
  public:
    ChMatrix33() : ChMatrixNM<Real, 3, 3>() {}

    /// Copy constructor
    ChMatrix33(const ChMatrix33<Real>& msource) : ChMatrixNM<Real, 3, 3>(msource) {}

    /// Copy constructor from all types of base matrices (only with same size)
    template <class RealB>
    ChMatrix33(const ChMatrix<RealB>& msource) {
        assert(msource.GetColumns() == 3 && msource.GetRows() == 3);
        this->rows = 3;
        this->columns = 3;
        this->address = this->buffer;
        for (int i = 0; i < 9; ++i)
            this->address[i] = (Real)msource.GetAddress()[i];
    }

    /// Construct a diagonal 3x3 matrix with all diagonal elements equal to the specified value.
    ChMatrix33(Real val) : ChMatrixNM<Real, 3, 3>() {
        this->Reset();
        this->Set33Element(0, 0, val);
        this->Set33Element(1, 1, val);
        this->Set33Element(2, 2, val);
    }

    /// Construct a 3x3 matrix with the specified vector as its diagonal.
    template <class RealB>
    ChMatrix33(const ChVector<RealB>& vec) : ChMatrixNM<Real, 3, 3>() {
        this->Reset();
        this->Set33Element(0, 0, vec.x());
        this->Set33Element(1, 1, vec.y());
        this->Set33Element(2, 2, vec.z());
    }

    /// Construct a symmetric 3x3 matrix with the specified vectors for the
    /// diagonal and off-diagonal.  The off-diagonal vector is assumed to contain
    /// the elements A(0,1), A(0,2), A(1,2) in this order.
    template <class RealB>
    ChMatrix33(const ChVector<RealB>& diag, const ChVector<>& off_diag) : ChMatrixNM<Real, 3, 3>() {
        this->Set33Element(0, 0, diag.x());
        this->Set33Element(1, 1, diag.y());
        this->Set33Element(2, 2, diag.z());

        this->Set33Element(0, 1, off_diag.x());
        this->Set33Element(1, 0, off_diag.x());

        this->Set33Element(0, 2, off_diag.y());
        this->Set33Element(2, 0, off_diag.y());

        this->Set33Element(1, 2, off_diag.z());
        this->Set33Element(2, 1, off_diag.z());
    }

    /// The constructor which builds a 3x3 matrix given a quaternion representing rotation.
    template <class RealB>
    ChMatrix33(const ChQuaternion<RealB>& mq) {
        this->rows = 3;
        this->columns = 3;
        this->address = this->buffer;
        Set_A_quaternion(mq);
    }

    /// Constructor that builds a rotation matrix from an gale of rotation and an axis,
    /// defined in absolute coords. NOTE, axis must be normalized!
    template <class RealB>
    ChMatrix33(const Real angle,             ///< angle of rotation, in radians
               const ChVector<RealB>& axis)  ///< axis of rotation, normalized
    {
        this->Set_A_AngAxis(angle, axis);
    }

    /// Complete generic constructor from 9 elements, ordered as three in first row,
    /// three in second row, three in third row.
    template <class RealB>
    ChMatrix33(const RealB& m00,
               const RealB& m01,
               const RealB& m02,
               const RealB& m10,
               const RealB& m11,
               const RealB& m12,
               const RealB& m20,
               const RealB& m21,
               const RealB& m22) {
        this->Set33Element(0, 0, m00);
        this->Set33Element(0, 1, m01);
        this->Set33Element(0, 2, m02);
        this->Set33Element(1, 0, m10);
        this->Set33Element(1, 1, m11);
        this->Set33Element(1, 2, m12);
        this->Set33Element(2, 0, m20);
        this->Set33Element(2, 1, m21);
        this->Set33Element(2, 2, m22);
    }

    //
    // OPERATORS
    //

    /// Assignment operator (from generic other matrix, acceptable only if other matrix has 3x3 size)
    ChMatrix33<Real>& operator=(const ChMatrix<Real>& matbis) {
        assert(matbis.GetColumns() == 3 && matbis.GetRows() == 3);
        ChMatrix<Real>::operator=(matbis);
        return *this;
    }

    /// Negates sign of the matrix.
    /// Performance warning: a new object is created.
    ChMatrix33<Real> operator-() const {
        ChMatrix33<Real> result(*this);
        result.MatrNeg();
        return result;
    }

    /// Sums this matrix and another matrix (of same size)
    /// Performance warning: a new object is created.
    template <class RealB>
    ChMatrix33<Real> operator+(const ChMatrix<RealB>& matbis) const {
        ChMatrix33<Real> result;
        result.MatrAdd(*this, matbis);
        return result;
    }

    /// Subtracts this matrix and another matrix (of same size).
    /// Performance warning: a new object is created.
    template <class RealB>
    ChMatrix33<Real> operator-(const ChMatrix<RealB>& matbis) const {
        ChMatrix33<Real> result;
        result.MatrSub(*this, matbis);
        return result;
    }

    /// Multiplies this ChMatrix33 matrix and another ChMatrix33 matrix.
    /// Performance warning: a new object is created.
    template <class RealB>
    ChMatrix33<Real> operator*(const ChMatrix33<RealB>& matbis) const {
        ChMatrix33<Real> result;
        result.MatrMultiply(*this, matbis);
        return result;
    }

    /// Multiplies this matrix and another ChMatrixNM matrix (3xN).
    /// Performance warning: a new object is created (of ChMatrixNM type).
    template <class RealB, int B_rows, int B_columns>
    ChMatrixNM<Real, 3, B_columns> operator*(const ChMatrixNM<RealB, 3, B_columns>& matbis) const {
        ChMatrixNM<Real, 3, B_columns> result;  // try statical sizing
        result.MatrMultiply(*this, matbis);
        return result;
    }

    /// Multiplies this matrix and another generic matrix.
    /// Performance warning: a new object is created (of ChMatrixDynamic type).
    template <class RealB>
    ChMatrixDynamic<Real> operator*(const ChMatrix<RealB>& matbis) const {
        ChMatrixDynamic<Real> result(this->rows, matbis.GetColumns());
        result.MatrMultiply(*this, matbis);
        return result;
    }

    /// Multiplies this matrix by a scalar value
    /// Performance warning: a new object is created.
    ChMatrix33<Real> operator*(const Real factor) const {
        ChMatrix33<Real> result(*this);
        result.MatrScale(factor);
        return result;
    }

    /// Multiplies this matrix by a vector.
    ChVector<Real> operator*(const ChVector<Real>& myvect) const { return this->Matr_x_Vect(myvect); }

    //
    // FUNCTIONS
    //

    /// Reset to identity a 3x3 matrix (ones on diagonal, zero elsewhere)
    /// Note: optimized, for 3x3 matrices ONLY!
    void Set33Identity() {
        this->Reset();
        this->Set33Element(0, 0, 1);
        this->Set33Element(1, 1, 1);
        this->Set33Element(2, 2, 1);
    }

    /// Return true if this matrix is the identity 3x3 matrix.
    bool IsIdentity() const {
        return this->Get33Element(0, 0) == 1 && this->Get33Element(0, 1) == 0 && this->Get33Element(0, 2) == 0 &&
               this->Get33Element(1, 0) == 0 && this->Get33Element(1, 1) == 1 && this->Get33Element(1, 2) == 0 &&
               this->Get33Element(2, 0) == 0 && this->Get33Element(2, 1) == 0 && this->Get33Element(2, 2) == 1;
    }

    /// Multiplies this matrix by a vector, like in coordinate rotation [M]*v.
    ///  \return The result of the multiplication, i.e. a vector.
    template <class RealB>
    ChVector<Real> Matr_x_Vect(const ChVector<RealB>& va) const {
        return ChVector<Real>(this->Get33Element(0, 0) * (Real)va.x() + this->Get33Element(0, 1) * (Real)va.y() +
                                  this->Get33Element(0, 2) * (Real)va.z(),
                              this->Get33Element(1, 0) * (Real)va.x() + this->Get33Element(1, 1) * (Real)va.y() +
                                  this->Get33Element(1, 2) * (Real)va.z(),
                              this->Get33Element(2, 0) * (Real)va.x() + this->Get33Element(2, 1) * (Real)va.y() +
                                  this->Get33Element(2, 2) * (Real)va.z());
    }

    /// Multiplies this matrix (transposed) by a vector, as [M]'*v
    ///  \return The result of the multiplication, i.e. a vector.
    template <class RealB>
    ChVector<Real> MatrT_x_Vect(const ChVector<RealB>& va) const {
        return ChVector<Real>(this->Get33Element(0, 0) * (Real)va.x() + this->Get33Element(1, 0) * (Real)va.y() +
                                  this->Get33Element(2, 0) * (Real)va.z(),
                              this->Get33Element(0, 1) * (Real)va.x() + this->Get33Element(1, 1) * (Real)va.y() +
                                  this->Get33Element(2, 1) * (Real)va.z(),
                              this->Get33Element(0, 2) * (Real)va.x() + this->Get33Element(1, 2) * (Real)va.y() +
                                  this->Get33Element(2, 2) * (Real)va.z());
    }

    /// Fast inversion of small matrices. Result will be in 'matra'.
    /// \return Returns the determinant.
    template <class RealB>
    Real FastInvert(ChMatrix33<RealB>& matra) {
        Real det;
        Real sdet0, sdet1, sdet2;

        sdet0 = +(this->Get33Element(1, 1) * this->Get33Element(2, 2)) -
                (this->Get33Element(2, 1) * this->Get33Element(1, 2));
        sdet1 = -(this->Get33Element(1, 0) * this->Get33Element(2, 2)) +
                (this->Get33Element(2, 0) * this->Get33Element(1, 2));
        sdet2 = +(this->Get33Element(1, 0) * this->Get33Element(2, 1)) -
                (this->Get33Element(2, 0) * this->Get33Element(1, 1));

        det = sdet0 * this->Get33Element(0, 0) + sdet1 * this->Get33Element(0, 1) + sdet2 * this->Get33Element(0, 2);

        matra.Set33Element(0, 0, sdet0 / det);
        matra.Set33Element(1, 0, sdet1 / det);
        matra.Set33Element(2, 0, sdet2 / det);
        matra.Set33Element(0, 1,
                           (-(this->Get33Element(0, 1) * this->Get33Element(2, 2)) +
                            (this->Get33Element(2, 1) * this->Get33Element(0, 2))) /
                               det);
        matra.Set33Element(1, 1,
                           (+(this->Get33Element(0, 0) * this->Get33Element(2, 2)) -
                            (this->Get33Element(2, 0) * this->Get33Element(0, 2))) /
                               det);
        matra.Set33Element(2, 1,
                           (-(this->Get33Element(0, 0) * this->Get33Element(2, 1)) +
                            (this->Get33Element(2, 0) * this->Get33Element(0, 1))) /
                               det);
        matra.Set33Element(0, 2,
                           (+(this->Get33Element(0, 1) * this->Get33Element(1, 2)) -
                            (this->Get33Element(1, 1) * this->Get33Element(0, 2))) /
                               det);
        matra.Set33Element(1, 2,
                           (-(this->Get33Element(0, 0) * this->Get33Element(1, 2)) +
                            (this->Get33Element(1, 0) * this->Get33Element(0, 2))) /
                               det);
        matra.Set33Element(2, 2,
                           (+(this->Get33Element(0, 0) * this->Get33Element(1, 1)) -
                            (this->Get33Element(1, 0) * this->Get33Element(0, 1))) /
                               det);

        return det;
    }

#define EIG_ROTATE(a, i, j, k, l)                 \
    g = a->Get33Element(i, j);                    \
    h = a->Get33Element(k, l);                    \
    a->Set33Element(i, j, g - s * (h + g * tau)); \
    a->Set33Element(k, l, h + s * (g - h * tau));

    /// Returns 3 eigenvalues and 3 eigenvectors in a 3x3 matrix,
    /// Notes:
    ///   - only for cases where eigenvalues 'd' are real!!
    ///   - output eigenvectors as columns of matrix 'v'
    ///   - this original matrix is modified
    void FastEigen(ChMatrix33<Real>& v, Real d[]) {
        int n = 3;
        int j, iq, ip, i;
        Real tresh, theta, tau, t, sm, s, h, g, c;
        int nrot;
        Real b[3];
        Real z[3];

        v.Set33Identity();

        for (ip = 0; ip < n; ip++) {
            b[ip] = this->Get33Element(ip, ip);
            d[ip] = this->Get33Element(ip, ip);
            z[ip] = 0;
        }

        nrot = 0;

        for (i = 0; i < 50; i++) {
            sm = 0;
            for (ip = 0; ip < n; ip++)
                for (iq = ip + 1; iq < n; iq++)
                    sm += fabs(this->Get33Element(ip, iq));

            if (sm == (Real)0) {
                return;
            }

            if (i < 3)
                tresh = (Real)0.2 * sm / (n * n);
            else
                tresh = 0;

            for (ip = 0; ip < n; ip++)
                for (iq = ip + 1; iq < n; iq++) {
                    g = (Real)100.0 * fabs(this->Get33Element(ip, iq));
                    if (i > 3 && fabs(d[ip]) + g == fabs(d[ip]) && fabs(d[iq]) + g == fabs(d[iq]))
                        this->Set33Element(ip, iq, 0);
                    else if (fabs(this->Get33Element(ip, iq)) > tresh) {
                        h = d[iq] - d[ip];
                        if (fabs(h) + g == fabs(h))
                            t = (this->Get33Element(ip, iq)) / h;
                        else {
                            theta = (Real)0.5 * h / this->Get33Element(ip, iq);
                            t = (Real)1.0 / (fabs(theta) + sqrt((Real)1.0 + theta * theta));
                            if (theta < 0)
                                t = -t;
                        }
                        c = (Real)1.0 / sqrt((Real)1.0 + t * t);
                        s = t * c;
                        tau = s / ((Real)1.0 + c);
                        h = t * this->Get33Element(ip, iq);
                        z[ip] -= h;
                        z[iq] += h;
                        d[ip] -= h;
                        d[iq] += h;
                        this->Set33Element(ip, iq, 0);
                        for (j = 0; j < ip; j++) {
                            EIG_ROTATE(this, j, ip, j, iq);
                        }
                        for (j = ip + 1; j < iq; j++) {
                            EIG_ROTATE(this, ip, j, j, iq);
                        }
                        for (j = iq + 1; j < n; j++) {
                            EIG_ROTATE(this, ip, j, iq, j);
                        }
                        for (j = 0; j < n; j++) {
                            EIG_ROTATE((&v), j, ip, j, iq);
                        }
                        nrot++;
                    }
                }

            for (ip = 0; ip < n; ip++) {
                b[ip] += z[ip];
                d[ip] = b[ip];
                z[ip] = 0;
            }
        }

        return;
    }

    /// Fills a 3x3 matrix as a rotation matrix corresponding
    /// to the rotation expressed by the quaternion 'quat'.
    template <class RealB>
    void Set_A_quaternion(const ChQuaternion<RealB>& quat) {
        Real e0e0 = (Real)(quat.e0() * quat.e0());
        Real e1e1 = (Real)(quat.e1() * quat.e1());
        Real e2e2 = (Real)(quat.e2() * quat.e2());
        Real e3e3 = (Real)(quat.e3() * quat.e3());
        Real e0e1 = (Real)(quat.e0() * quat.e1());
        Real e0e2 = (Real)(quat.e0() * quat.e2());
        Real e0e3 = (Real)(quat.e0() * quat.e3());
        Real e1e2 = (Real)(quat.e1() * quat.e2());
        Real e1e3 = (Real)(quat.e1() * quat.e3());
        Real e2e3 = (Real)(quat.e2() * quat.e3());

        this->Set33Element(0, 0, (e0e0 + e1e1) * 2 - 1);
        this->Set33Element(0, 1, (e1e2 - e0e3) * 2);
        this->Set33Element(0, 2, (e1e3 + e0e2) * 2);
        this->Set33Element(1, 0, (e1e2 + e0e3) * 2);
        this->Set33Element(1, 1, (e0e0 + e2e2) * 2 - 1);
        this->Set33Element(1, 2, (e2e3 - e0e1) * 2);
        this->Set33Element(2, 0, (e1e3 - e0e2) * 2);
        this->Set33Element(2, 1, (e2e3 + e0e1) * 2);
        this->Set33Element(2, 2, (e0e0 + e3e3) * 2 - 1);
    }

    /// Fills a 3x3 matrix as the "star" matrix, representing vector cross product.
    /// That is, given two 3d vectors a and b, aXb= [Astar]*b
    template <class RealB>
    void Set_X_matrix(const ChVector<RealB>& vect) {
        this->Set33Element(0, 0, 0);
        this->Set33Element(0, 1, -(Real)vect.z());
        this->Set33Element(0, 2, (Real)vect.y());
        this->Set33Element(1, 0, (Real)vect.z());
        this->Set33Element(1, 1, 0);
        this->Set33Element(1, 2, -(Real)vect.x());
        this->Set33Element(2, 0, -(Real)vect.y());
        this->Set33Element(2, 1, (Real)vect.x());
        this->Set33Element(2, 2, 0);
    }

    /// Fills a 3x3 matrix as product of two 'cross product' matrices,
    /// as double vector cross product.
    template <class RealB>
    void Set_XY_matrix(const ChVector<RealB>& vectA, const ChVector<RealB>& vectB) {
        this->Set33Element(0, 0, -vectA.y() * vectB.y() - vectA.z() * vectB.z());
        this->Set33Element(1, 0, vectA.x() * vectB.y());
        this->Set33Element(2, 0, vectA.x() * vectB.z());
        this->Set33Element(0, 1, vectA.y() * vectB.x());
        this->Set33Element(1, 1, -vectA.z() * vectB.z() - vectA.x() * vectB.x());
        this->Set33Element(2, 1, vectA.y() * vectB.z());
        this->Set33Element(0, 2, vectA.z() * vectB.x());
        this->Set33Element(1, 2, vectA.z() * vectB.y());
        this->Set33Element(2, 2, -vectA.x() * vectB.x() - vectA.y() * vectB.y());
    }

    /// Fills a 3x3 matrix as a rotation matrix, given the three
    /// versors X,Y,Z of the basis.
    template <class RealB>
    void Set_A_axis(const ChVector<RealB>& X, const ChVector<RealB>& Y, const ChVector<RealB>& Z) {
        this->Set33Element(0, 0, (Real)X.x());
        this->Set33Element(0, 1, (Real)Y.x());
        this->Set33Element(0, 2, (Real)Z.x());
        this->Set33Element(1, 0, (Real)X.y());
        this->Set33Element(1, 1, (Real)Y.y());
        this->Set33Element(1, 2, (Real)Z.y());
        this->Set33Element(2, 0, (Real)X.z());
        this->Set33Element(2, 1, (Real)Y.z());
        this->Set33Element(2, 2, (Real)Z.z());
    }

    /// Fills a 3x3 matrix as a rotation matrix, given the three
    /// Eulero angles (not to be confused with 'Eulero parameters', aka quaternions)
    template <class RealB>
    void Set_A_Eulero(const ChVector<RealB>& eul) {
        Real cx = cos((Real)eul.x());
        Real cy = cos((Real)eul.y());
        Real cz = cos((Real)eul.z());
        Real sx = sin((Real)eul.x());
        Real sy = sin((Real)eul.y());
        Real sz = sin((Real)eul.z());

        this->Set33Element(0, 0, ((cz * cx) - (cy * sx * sz)));
        this->Set33Element(0, 1, (-(sz * cx) - (cy * sx * cz)));
        this->Set33Element(0, 2, (sy * sx));
        this->Set33Element(1, 0, ((cz * sx) + (cy * cx * sz)));
        this->Set33Element(1, 1, (-(sz * sx) + (cy * cx * cz)));
        this->Set33Element(1, 2, (-sy * cx));
        this->Set33Element(2, 0, (sy * sz));
        this->Set33Element(2, 1, (sy * cz));
        this->Set33Element(2, 2, (cy));
    }

    /// Fills a 3x3 matrix as a rotation matrix, given the three
    /// Cardano angles.
    template <class RealB>
    void Set_A_Cardano(const ChVector<RealB>& car) {
        Real cx = cos((Real)car.x());
        Real cy = cos((Real)car.y());
        Real cz = cos((Real)car.z());
        Real sx = sin((Real)car.x());
        Real sy = sin((Real)car.y());
        Real sz = sin((Real)car.z());

        this->Set33Element(0, 0, ((cx * cz) - (sz * sx * sy)));
        this->Set33Element(0, 1, (-sx * cy));
        this->Set33Element(0, 2, ((cx * sz) + (sx * sy * cz)));
        this->Set33Element(1, 0, ((sx * cz) + (cx * sy * sz)));
        this->Set33Element(1, 1, (cy * cx));
        this->Set33Element(1, 2, ((sx * sz) - (cx * sy * cz)));
        this->Set33Element(2, 0, (-sz * cy));
        this->Set33Element(2, 1, (sy));
        this->Set33Element(2, 2, (cy * cz));
    }

    /// Fills a 3x3 matrix as a rotation matrix, given the three
    /// head, pitch, banking  angles.
    template <class RealB>
    void Set_A_Hpb(const ChVector<RealB>& hpb) {
        Real cx = cos((Real)hpb.y());
        Real cy = cos((Real)hpb.x());
        Real cz = cos((Real)hpb.z());
        Real sx = sin((Real)hpb.y());
        Real sy = sin((Real)hpb.x());
        Real sz = sin((Real)hpb.z());

        this->Set33Element(0, 0, ((cz * cy) - (sz * sx * sy)));
        this->Set33Element(0, 1, (-(sz * cy) - (cz * sx * sy)));
        this->Set33Element(0, 2, (-cx * sy));
        this->Set33Element(1, 0, (sz * cx));
        this->Set33Element(1, 1, (cz * cx));
        this->Set33Element(1, 2, (-sx));
        this->Set33Element(2, 0, ((cz * sy) + (sz * sx * cy)));
        this->Set33Element(2, 1, (-(sz * sy) + (cz * sx * cy)));
        this->Set33Element(2, 2, (cx * cy));
    }

    /// Fills a 3x3 matrix as a rotation matrix, given the three
    /// angles of consecutive rotations about x,y,z axis.
    template <class RealB>
    void Set_A_Rxyz(const ChVector<RealB>& xyz) {
        Real cx = cos((Real)xyz.x());
        Real cy = cos((Real)xyz.y());
        Real cz = cos((Real)xyz.z());
        Real sx = sin((Real)xyz.x());
        Real sy = sin((Real)xyz.y());
        Real sz = sin((Real)xyz.z());

        this->Set33Element(0, 0, (cy * cz));
        this->Set33Element(0, 1, (cy * sz));
        this->Set33Element(0, 2, (-sy));
        this->Set33Element(1, 0, ((sx * sy * cz) - (cx * sz)));
        this->Set33Element(1, 1, ((sx * sy * sz) + (cx * cz)));
        this->Set33Element(1, 2, (sx * cy));
        this->Set33Element(2, 0, ((cx * sy * cz) + (sx * sz)));
        this->Set33Element(2, 1, ((cx * sy * sz) - (sx * cz)));
        this->Set33Element(2, 2, (cx * cy));
    }

    /// Fills a 3x3 matrix as a rotation matrix, given the three
    /// Rodriguez' parameters.
    template <class RealB>
    void Set_A_Rodriguez(const ChVector<RealB>& rod) {
        Real gam = (Real)(pow(rod.x(), 2) + pow(rod.y(), 2) + pow(rod.z(), 2));

        this->Set33Element(0, 0, (Real)(1 + pow(rod.x(), 2) - pow(rod.y(), 2) - pow(rod.z(), 2)));
        this->Set33Element(0, 1, (Real)(2 * (rod.x() * rod.y() - rod.z())));
        this->Set33Element(0, 2, (Real)(2 * (rod.x() * rod.z() + rod.y())));
        this->Set33Element(1, 0, (Real)(2 * (rod.x() * rod.y() + rod.z())));
        this->Set33Element(1, 1, (Real)(1 - pow(rod.x(), 2) + pow(rod.y(), 2) - pow(rod.z(), 2)));
        this->Set33Element(1, 2, (Real)(2 * (rod.y() * rod.z() - rod.x())));
        this->Set33Element(2, 0, (Real)(2 * (rod.x() * rod.z() - rod.y())));
        this->Set33Element(2, 1, (Real)(2 * (rod.y() * rod.z() + rod.x())));
        this->Set33Element(2, 2, (Real)(1 - pow(rod.x(), 2) - pow(rod.y(), 2) + pow(rod.z(), 2)));

        this->MatrScale((1 / (1 + gam)));
    }

    /// Use the Gram-Schmidt orthonormalization to find the three
    /// orthogonal vectors of a coordinate system whose X axis is this vector.
    /// mVsingular (optional) suggests the XY plane, possibly it is not too parallel to X.
    void Set_A_Xdir(const ChVector<Real>& Xdir,  ///< X axis
                    const ChVector<Real>& Vsingular =
                        ChVector<Real>(0, 1, 0))  ///< a direction on XY plane (optional suggestion for Y axis)
    {
        ChVector<Real> mX;
        ChVector<Real> mY;
        ChVector<Real> mZ;
        Xdir.DirToDxDyDz(mX, mY, mZ, Vsingular);
        this->Set_A_axis(mX, mY, mZ);
    }

    /// Sets the rotation matrix from an gale of rotation and an axis,
    /// defined in _absolute_ coords. NOTE, axis must be normalized!
    void Set_A_AngAxis(const Real angle,            ///< angle of rotation, in radians
                       const ChVector<Real>& axis)  ///< axis of rotation, normalized
    {
        ChQuaternion<Real> mr;
        mr.Q_from_AngAxis(angle, axis);
        this->Set_A_quaternion(mr);
    }

    /// Given a 3x3 rotation matrix, computes the corresponding
    /// quaternion.
    ChQuaternion<Real> Get_A_quaternion() const {
        ChQuaternion<Real> q;
        Real s, tr;
        Real half = (Real)0.5;

        // for speed reasons: ..
        Real m00 = this->Get33Element(0, 0);
        Real m01 = this->Get33Element(0, 1);
        Real m02 = this->Get33Element(0, 2);
        Real m10 = this->Get33Element(1, 0);
        Real m11 = this->Get33Element(1, 1);
        Real m12 = this->Get33Element(1, 2);
        Real m20 = this->Get33Element(2, 0);
        Real m21 = this->Get33Element(2, 1);
        Real m22 = this->Get33Element(2, 2);

        tr = m00 + m11 + m22;  // diag sum

        if (tr >= 0) {
            s = sqrt(tr + 1);
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
                    s = sqrt(m00 - m11 - m22 + 1);
                    q.e1() = half * s;
                    s = half / s;
                    q.e2() = (m01 + m10) * s;
                    q.e3() = (m20 + m02) * s;
                    q.e0() = (m21 - m12) * s;
                    break;
                case 1:
                    s = sqrt(m11 - m22 - m00 + 1);
                    q.e2() = half * s;
                    s = half / s;
                    q.e3() = (m12 + m21) * s;
                    q.e1() = (m01 + m10) * s;
                    q.e0() = (m02 - m20) * s;
                    break;
                case 2:
                    s = sqrt(m22 - m00 - m11 + 1);
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

    /// Given a 3x3 rotation matrix, returns the versor of X axis.
    ChVector<Real> Get_A_Xaxis() const {
        ChVector<Real> X;
        X.x() = this->Get33Element(0, 0);
        X.y() = this->Get33Element(1, 0);
        X.z() = this->Get33Element(2, 0);
        return X;
    }

    /// Given a 3x3 rotation matrix, returns the versor of Y axis.
    ChVector<Real> Get_A_Yaxis() const {
        ChVector<Real> Y;
        Y.x() = this->Get33Element(0, 1);
        Y.y() = this->Get33Element(1, 1);
        Y.z() = this->Get33Element(2, 1);
        return Y;
    }

    /// Given a 3x3 rotation matrix, returns the versor of Z axis.
    ChVector<Real> Get_A_Zaxis() const {
        ChVector<Real> Z;
        Z.x() = this->Get33Element(0, 2);
        Z.y() = this->Get33Element(1, 2);
        Z.z() = this->Get33Element(2, 2);
        return Z;
    }

    /// Given a 3x3 rotation matrix, returns the Eulero angles.
    ChVector<Real> Get_A_Eulero() const {
        ChVector<Real> eul;

        eul.y() = acos(this->GetElement(2, 2));                    // rho, nutation
        eul.z() = acos((this->GetElement(2, 1)) / sin(eul.y()));   // csi, spin
        eul.x() = acos(-(this->GetElement(1, 2)) / sin(eul.y()));  // rho, nutation

        if (eul.y() == 0)  // handle undefined initial position set
        {
            eul.x() = 0;
            eul.z() = 0;
        }

        return eul;
    }

    /// Given a 3x3 rotation matrix, returns the Cardano angles.
    ChVector<Real> Get_A_Cardano() const {
        ChVector<Real> car;

        Real mel21 = this->GetElement(2, 1);
        if (mel21 > 1)
            mel21 = 1;
        if (mel21 < -1)
            mel21 = -1;

        car.y() = asin(mel21);

        Real arg2 = (this->GetElement(2, 2)) / cos(car.y());
        if (arg2 > 1)
            arg2 = 1;
        if (arg2 < -1)
            arg2 = -1;
        Real arg3 = (this->GetElement(1, 1)) / cos(car.y());
        if (arg3 > 1)
            arg3 = 1;
        if (arg3 < -1)
            arg3 = -1;

        car.z() = acos(arg2);
        car.x() = acos(arg3);

        return car;
    }

    /// Given a 3x3 rotation matrix, returns the head-pitch-banking angles.
    ChVector<Real> Get_A_Hpb() const {
        ChVector<Real> Hpb;

        Real arg1 = -(this->GetElement(1, 2));
        if (arg1 > 1)
            arg1 = 1;
        if (arg1 < -1)
            arg1 = -1;

        Hpb.y() = asin(arg1);  // P

        Real arg2 = (this->GetElement(2, 2)) / cos(Hpb.y());
        if (arg2 > 1)
            arg2 = 1;
        if (arg2 < -1)
            arg2 = -1;
        Real arg3 = (this->GetElement(1, 1)) / cos(Hpb.y());
        if (arg3 > 1)
            arg3 = 1;
        if (arg3 < -1)
            arg3 = -1;

        Hpb.x() = acos(arg2);  // H
        Hpb.z() = acos(arg3);  // B

        return Hpb;
    }

    /// Given a 3x3 rotation matrix, returns the angles for
    /// consecutive rotations on x,y,z axes
    ChVector<Real> Get_A_Rxyz() const {
        ChVector<Real> Rxyz;

        Real arg1 = -(this->GetElement(0, 2));
        if (arg1 > 1)
            arg1 = 1;
        if (arg1 < -1)
            arg1 = -1;
        Rxyz.y() = asin(arg1);

        Real arg2 = (this->GetElement(0, 1)) / cos(Rxyz.y());
        if (arg2 > 1)
            arg2 = 1;
        if (arg2 < -1)
            arg2 = -1;
        Real arg3 = (this->GetElement(1, 2)) / cos(Rxyz.y());
        if (arg3 > 1)
            arg3 = 1;
        if (arg3 < -1)
            arg3 = -1;

        Rxyz.z() = asin(arg2);
        Rxyz.x() = asin(arg3);

        return Rxyz;
    }

    /// Given a 3x3 rotation matrix, returns the Rodriguez parameters.
    ChVector<Real> Get_A_Rodriguez() const {
        ChVector<Real> rod;
        ChQuaternion<Real> qtemp;
        qtemp = Get_A_quaternion();
        // warning: infinite results may happen..
        rod.x() = qtemp.e1() / qtemp.e0();
        rod.y() = qtemp.e2() / qtemp.e0();
        rod.z() = qtemp.e3() / qtemp.e0();

        return rod;
    }

    /// Returns the absolute value of each element in the matrix
    ChMatrix33<Real> Get_Abs_Matrix() const {
        ChMatrix33<Real> temp;

        temp.SetElement(0, 0, fabs(this->GetElement(0, 0)));
        temp.SetElement(0, 1, fabs(this->GetElement(0, 1)));
        temp.SetElement(0, 2, fabs(this->GetElement(0, 2)));

        temp.SetElement(1, 0, fabs(this->GetElement(1, 0)));
        temp.SetElement(1, 1, fabs(this->GetElement(1, 1)));
        temp.SetElement(1, 2, fabs(this->GetElement(1, 2)));

        temp.SetElement(2, 0, fabs(this->GetElement(2, 0)));
        temp.SetElement(2, 1, fabs(this->GetElement(2, 1)));
        temp.SetElement(2, 2, fabs(this->GetElement(2, 2)));

        return temp;
    }

    /// Given a 3x3 rotation matrix, get the diagonal.
    ChVector<Real> Get_Diag() const {
        ChVector<Real> temp;
        temp.x() = this->Get33Element(0, 0);
        temp.y() = this->Get33Element(1, 1);
        temp.z() = this->Get33Element(2, 2);
        return temp;
    }

    /// Return the sum of the three elements on the diagonal
    Real GetTrace() const { return this->Get33Element(0, 0) + this->Get33Element(1, 1) + this->Get33Element(2, 2); }

    /// Assuming it is an orthogonal rotation matrix, get Ax vector
    ChVector<Real> GetAx() const {
        return ChVector<Real>(0.5 * (this->Get33Element(2, 1) - this->Get33Element(1, 2)),
                              0.5 * (this->Get33Element(0, 2) - this->Get33Element(2, 0)),
                              0.5 * (this->Get33Element(1, 0) - this->Get33Element(0, 1)));
    };

    /// Return a symmetric matrix =(1/2)*(A+A')
    ChMatrix33<Real> GetSymm() const {
        Real m12 = 0.5 * (this->Get33Element(1, 0) + this->Get33Element(0, 1));
        Real m13 = 0.5 * (this->Get33Element(2, 0) + this->Get33Element(0, 2));
        Real m23 = 0.5 * (this->Get33Element(2, 1) + this->Get33Element(1, 2));

        return ChMatrix33<Real>(this->Get33Element(0, 0), m12, m13, m12, this->Get33Element(1, 1), m23, m13, m23,
                                this->Get33Element(2, 2));
    };

    /// Convert to a 2-dimensional array
    void To_Marray(double marr[3][3]) {
        marr[0][0] = this->Get33Element(0, 0);
        marr[0][1] = this->Get33Element(0, 1);
        marr[0][2] = this->Get33Element(0, 2);
        marr[1][0] = this->Get33Element(1, 0);
        marr[1][1] = this->Get33Element(1, 1);
        marr[1][2] = this->Get33Element(1, 2);
        marr[2][0] = this->Get33Element(2, 0);
        marr[2][1] = this->Get33Element(2, 1);
        marr[2][2] = this->Get33Element(2, 2);
    }

    /// Convert from a 2-dimensional array
    ChMatrix33<Real> From_Marray(double marr[3][3]) {
        ChMatrix33<Real> mma;

        mma(0, 0) = marr[0][0];
        mma(0, 1) = marr[0][1];
        mma(0, 2) = marr[0][2];
        mma(1, 0) = marr[1][0];
        mma(1, 1) = marr[1][1];
        mma(1, 2) = marr[1][2];
        mma(2, 0) = marr[2][0];
        mma(2, 1) = marr[2][1];
        mma(2, 2) = marr[2][2];

        return mma;
    }
};

// Compute a 3x3 matrix as a tensor product between two vectors (outer product of vectors)
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

}  // end namespace chrono

#endif
