// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// This file contains the definitions of spatial vectors and matrices.
//
// =============================================================================

#ifndef CH_SPATIAL_H
#define CH_SPATIAL_H

#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChMatrixMBD.h"

namespace chrono {

// Shorthand notation for a vector with 6 doubles.
using ChVector6 = ChVectorN<double, 6>;

// Shorthand notation for a 3xN matrix of doubles.
// Note: must use ColumnMajor ordering because Eigen compains when instantiating with N=1 if using RowMajor.
template <int N>
using ChMatrix3N = ChMatrixNM_col<double, 3, N>;

// Shorthand notation for a 6xN matrix of doubles.
// Note: must use ColumnMajor ordering because Eigen compains when instantiating with N=1 if using RowMajor.
template <int N>
using ChMatrix6N = ChMatrixNM_col<double, 6, N>;

// Shorthand notation for an NxN matrix of doubles.
template <int N>
using ChMatrixNN = ChMatrixNM<double, N, N>;

namespace soa {

// Forward declarations
class ChShiftMat;
class ChShiftMatT;
template <int N>
class ChVelMat;
template <int N>
class ChVelMatT;

/// @addtogroup chrono_soa
/// @{

// -----------------------------------------------------------------------------

/// Definition of a spatial vector.
/// A ChSpatialVec encapsulates two 3D vectors. Since these spatial vectors most often represent a spatial velocity, the
/// two components are called "angular" and "linear" (stored ion this order).
class ChSpatialVec : public ChVector6 {
  public:
    ChSpatialVec() : ChVector6() {}

    /// Construct a spatial vector from its two 3D vector components.
    ChSpatialVec(const ChVector3d& angIn, const ChVector3d& linIn) {
        segment(0, 3) = angIn.eigen();
        segment(3, 3) = linIn.eigen();
    }

    /// Constructor from Eigen expressions.
    template <typename OtherDerived>
    ChSpatialVec(const Eigen::MatrixBase<OtherDerived>& other) : ChVector6(other) {}

    /// Access the "angular" component.
    ChVector3d& ang() { return *((ChVector3d*)(data())); }

    /// Const access to the "angular" component.
    const ChVector3d& ang() const { return *((ChVector3d*)(data())); }

    /// Access the "linear" component.
    ChVector3d lin() { return *((ChVector3d*)(data() + 3)); }

    /// Const access to the "linear" component.
    const ChVector3d& lin() const { return *((ChVector3d*)(data() + 3)); }

    /// Assign Eigen expressions to ChSpatialVec.
    template <typename OtherDerived>
    ChSpatialVec& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->ChVector6::operator=(other);
        return *this;
    }

    /// Operator for sign change.
    const ChSpatialVec& operator+() const { return *this; }
    /// Operator for sign change.
    ChSpatialVec operator-() const { return this->ChVector6::operator-(); }

    /// Operator for vector sum.
    ChSpatialVec operator+(const ChSpatialVec& v) const { return ChSpatialVec(this->ChVector6::operator+(v)); }
    /// Operator for vector difference.
    ChSpatialVec operator-(const ChSpatialVec& v) const { return ChSpatialVec(this->ChVector6::operator-(v)); }

    /// Operator for vector increment.
    ChSpatialVec& operator+=(const ChSpatialVec& v) {
        this->ChVector6::operator+=(v);
        return *this;
    }
    /// Operator for vector decrement.
    ChSpatialVec& operator-=(const ChSpatialVec& v) {
        this->ChVector6::operator-=(v);
        return *this;
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// -----------------------------------------------------------------------------

/// Definition of a spatial matrix.
/// A ChSpatialMat encapsulates four 3x3 matrices and represents the 6x6 matrix:
/// <pre>
///   [ A00 | A01 ]
///   [ ----+---- ]
///   [ A10 | A11 ]
/// </pre>
class ChSpatialMat {
  public:
    ChSpatialMat() {}
    ChSpatialMat(const ChMatrix33d& A00, const ChMatrix33d& A01, const ChMatrix33d& A10, const ChMatrix33d& A11)
        : m_A00(A00), m_A01(A01), m_A10(A10), m_A11(A11) {}

    ChMatrix33d& A00() { return m_A00; }
    ChMatrix33d& A01() { return m_A01; }
    ChMatrix33d& A10() { return m_A10; }
    ChMatrix33d& A11() { return m_A11; }
    const ChMatrix33d& A00() const { return m_A00; }
    const ChMatrix33d& A01() const { return m_A01; }
    const ChMatrix33d& A10() const { return m_A10; }
    const ChMatrix33d& A11() const { return m_A11; }

    ChSpatialMat& operator+=(const ChSpatialMat& mat);
    ChSpatialMat& operator-=(const ChSpatialMat& mat);
    ChSpatialMat operator+(const ChSpatialMat& mat) const;
    ChSpatialMat operator-(const ChSpatialMat& mat) const;

    /// View this spatial matrix as a 6x6 Eigen matrix (debugging and testing).
    ChMatrixNM<double, 6, 6> eigen() const;

    /// Construct a spatial matrix by applying a symmetric spatial shift to the given spatial matrix.
    ///    S1 = ~Phi * S * Phi
    /// where Phi is a ChShiftMat.
    static ChSpatialMat constructFrom(const ChSpatialMat& S, const ChShiftMat& Phi);

    /// Construct a spatial matrix from a velocity matrix.
    ///    S2 = H * D * ~H
    /// where H is a ChVelMat<N> and D is a symmetric ChMatrixNN<N,N>.
    template <int N>
    static ChSpatialMat constructFrom(const ChVelMat<N>& H, const ChMatrixNN<N>& D);

  private:
    ChMatrix33d m_A00;
    ChMatrix33d m_A01;
    ChMatrix33d m_A10;
    ChMatrix33d m_A11;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// -----------------------------------------------------------------------------

/// Definition of the helper "shift matrix".
/// A ChShiftMat encapsulates a single 3D vector 'l' and represents the 6x6 matrix:
/// <pre>
///   [ I3           | 0  ]
///   [ -------------+--- ]
///   [ -crossMat(l) | I3 ]
/// </pre>
/// where I3 the the 3x3 identity matrix.
class ChShiftMat {
  public:
    ChShiftMat() {}
    ChShiftMat(const ChVector3d& l) : m_l(l) {}

    ChVector3d& l() { return m_l; }
    const ChVector3d& l() const { return m_l; }

    void setZero() { m_l.SetNull(); }

    const ChShiftMatT& operator~() const { return *((const ChShiftMatT*)this); }

    /// View this shift matrix as a 6x6 Eigen matrix (debugging and testing).
    ChMatrixNM<double, 6, 6> eigen() const;

  private:
    ChVector3d m_l;
};

// -----------------------------------------------------------------------------

/// Definition of the transposed "shift matrix".
/// ChShiftMatT is a very special case with limited functionality. Its primary feature is that it represents the
/// transposed without doing any work.
class ChShiftMatT {
  public:
    const ChVector3d& l() const { return m_l; }

  private:
    ChShiftMatT() {}
    ChVector3d m_l;
};

// -----------------------------------------------------------------------------

/// Defintion of the helper "velocity matrix" class.
/// A ChVelMat encapsulates two matrices, each with 3 rows and an arbitrary number of columns N (1 <= N <= 6).
template <int N>
class ChVelMat {
  public:
    /// Construct a ChVelMat from its two 3xN component matrices.
    ChVelMat(const ChMatrix3N<N>& ang, const ChMatrix3N<N>& lin) : m_lin(lin), m_ang(ang) {}
    ChVelMat() {}

    ChMatrix3N<N>& ang() { return m_ang; }
    const ChMatrix3N<N>& ang() const { return m_ang; }
    ChMatrix3N<N>& lin() { return m_lin; }
    const ChMatrix3N<N>& lin() const { return m_lin; }

    ChVelMat<N> operator+(const ChVelMat<N>& mat) const { return ChVelMat<N>(m_ang + mat.ang(), m_lin + mat.lin()); }
    ChVelMat<N> operator-(const ChVelMat<N>& mat) const { return ChVelMat<N>(m_ang - mat.ang(), m_lin - mat.lin()); }
    const ChVelMatT<N>& operator~() const { return *((const ChVelMatT<N>*)this); }

    /// View this velocity matrix as a 6xN Eigen matrix (debugging and testing).
    ChMatrix6N<N> eigen() const;

  private:
    ChMatrix3N<N> m_ang;
    ChMatrix3N<N> m_lin;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// -----------------------------------------------------------------------------

/// Definition of the transposed "velocity matrix".
/// ChVelMatT is a very special case with limited functionality. Its primary feature is that it represents the
/// transposed without doing any work.
template <int N>
class ChVelMatT {
  public:
    const ChMatrix3N<N>& ang() const { return m_ang; }
    const ChMatrix3N<N>& lin() const { return m_lin; }

  private:
    ChVelMatT() {}
    ChMatrix3N<N> m_ang;
    ChMatrix3N<N> m_lin;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// =============================================================================

inline ChMatrixNM<double, 6, 6> ChSpatialMat::eigen() const {
    ChMatrixNM<double, 6, 6> S_e;
    S_e.block(0, 0, 3, 3) = m_A00;
    S_e.block(0, 3, 3, 3) = m_A01;
    S_e.block(3, 0, 3, 3) = m_A10;
    S_e.block(3, 3, 3, 3) = m_A11;
    return S_e;
}

inline ChSpatialMat ChSpatialMat::constructFrom(const ChSpatialMat& S, const ChShiftMat& Phi) {
    auto x = ChStarMatrix33<>(Phi.l());
    return ChSpatialMat(S.A00() + x * S.A10() - S.A01() * x - x * S.A11() * x, S.A01() + x * S.A11(),
                        S.A10() - S.A11() * x, S.A11());
}

template <int N>
inline ChSpatialMat ChSpatialMat::constructFrom(const ChVelMat<N>& H, const ChMatrixNN<N>& D) {
    return ChSpatialMat(H.ang() * D * H.ang().transpose(), H.ang() * D * H.lin().transpose(),
                        H.lin() * D * H.ang().transpose(), H.lin() * D * H.lin().transpose());
}

inline ChSpatialMat& ChSpatialMat::operator+=(const ChSpatialMat& mat) {
    m_A00 += mat.A00();
    m_A01 += mat.A01();
    m_A10 += mat.A10();
    m_A11 += mat.A11();

    return *this;
}

inline ChSpatialMat& ChSpatialMat::operator-=(const ChSpatialMat& mat) {
    m_A00 -= mat.A00();
    m_A01 -= mat.A01();
    m_A10 -= mat.A10();
    m_A11 -= mat.A11();

    return *this;
}

inline ChSpatialMat ChSpatialMat::operator+(const ChSpatialMat& mat) const {
    return ChSpatialMat(m_A00 + mat.A00(), m_A01 + mat.A01(), m_A10 + mat.A10(), m_A11 + mat.A11());
}

inline ChSpatialMat ChSpatialMat::operator-(const ChSpatialMat& mat) const {
    return ChSpatialMat(m_A00 - mat.A00(), m_A01 - mat.A01(), m_A10 - mat.A10(), m_A11 - mat.A11());
}

// -----------------------------------------------------------------------------

inline ChMatrixNM<double, 6, 6> ChShiftMat::eigen() const {
    ChMatrixNM<double, 6, 6> P_e;
    P_e.block(0, 0, 3, 3).setIdentity();
    P_e.block(0, 3, 3, 3).setZero();
    P_e.block(3, 0, 3, 3) = ChStarMatrix33<>(-m_l);
    P_e.block(3, 3, 3, 3).setIdentity();
    return P_e;
}

// -----------------------------------------------------------------------------

template <int N>
inline ChMatrix6N<N> ChVelMat<N>::eigen() const {
    ChMatrix6N<N> H_e;
    H_e.block(0, 0, 3, N) = m_ang;
    H_e.block(3, 0, 3, N) = m_lin;
    return H_e;
}

// -----------------------------------------------------------------------------

// These are various multiplication operators involving spatial objects.
// The following operations are defined:
//	-	ChSpatialVec = ChSpatialMat * ChSpatialVec
//	-	ChSpatialVec = ChShiftMat * ChSpatialVec
//	-	ChSpatialVec = ChShiftMatT * ChSpatialVec
//	-	ChSpatialMat = ChSpatialMat * ChShiftMat
//	-	ChSpatialMat = ChShiftMatT * ChSpatialMat
//	-	ChSpatialVec = ChVelMat<N> * ChVectorN<N>
//  -   ChSpatialVec = ChVelMat<N> * Eigen::Ref<ChVectorN<N>>
//	-	ChVectorN<N> = ChVelMatT<N> * ChSpatialVec
//	-	ChVelMat<N> = ChVelMat<N> * ChMatrixNN<N,N>
//	-	ChVelMat<N> = ChSpatialMat * ChVelMat<N>
//	-	ChVelMat<N> = ChMatrixNN<3,3> * ChVelMat<N>
//	-	ChSpatialMat = ChVelMat<N> * ChVelMatT<N>
//	-	ChMatrixNN<N,N> = ChVelMatT<N> * ChVelMat<N>

inline ChSpatialVec operator*(const ChSpatialMat& S, const ChSpatialVec& V) {
    return ChSpatialVec(S.A00() * V.ang() + S.A01() * V.lin(), S.A10() * V.ang() + S.A11() * V.lin());
}

inline ChSpatialVec operator*(const ChShiftMat& Phi, const ChSpatialVec& V) {
    return ChSpatialVec(V.ang(), V.lin() + V.ang() % Phi.l());
}

inline ChSpatialVec operator*(const ChShiftMatT& PhiT, const ChSpatialVec& V) {
    return ChSpatialVec(V.ang() + PhiT.l() % V.lin(), V.lin());
}

inline ChSpatialMat operator*(const ChSpatialMat& S, const ChShiftMat& Phi) {
    auto x = ChStarMatrix33<>(Phi.l());
    return ChSpatialMat(S.A00() - S.A01() * x, S.A01(), S.A10() - S.A11() * x, S.A11());
}

inline ChSpatialMat operator*(const ChShiftMatT& PhiT, const ChSpatialMat& S) {
    auto x = ChStarMatrix33<>(PhiT.l());
    return ChSpatialMat(S.A00() + x * S.A10(), S.A01() + x * S.A11(), S.A10(), S.A11());
}

template <int N>
inline ChSpatialVec operator*(const ChVelMat<N>& H, const ChVectorN<double, N>& vec) {
    return ChSpatialVec(H.ang() * vec, H.lin() * vec);
}

template <int N>
inline ChSpatialVec operator*(const ChVelMat<N>& H, Eigen::Ref<ChVectorN<double, N>> vec) {
    return ChSpatialVec(H.ang() * vec, H.lin() * vec);
}

template <int N>
inline ChVectorN<double, N> operator*(const ChVelMatT<N>& H, const ChSpatialVec& V) {
    return H.ang().transpose() * V.ang().eigen() + H.lin().transpose() * V.lin().eigen();
}

template <int N>
inline ChVelMat<N> operator*(const ChVelMat<N>& H, const ChMatrixNN<N>& mat) {
    return ChVelMat<N>(H.ang() * mat, H.lin() * mat);
}

template <int N>
inline ChVelMat<N> operator*(const ChSpatialMat& S, const ChVelMat<N>& H) {
    return ChVelMat<N>(S.A00() * H.ang() + S.A01() * H.lin(), S.A10() * H.ang() + S.A11() * H.lin());
}

template <int N>
inline ChVelMat<N> operator*(const ChMatrix33d& M, const ChVelMat<N>& H) {
    return ChVelMat<N>(M * H.ang(), M * H.lin());
}

template <int N>
inline ChSpatialMat operator*(const ChVelMat<N>& H1, const ChVelMatT<N>& H2) {
    return ChSpatialMat(H1.ang() * H2.ang().transpose(), H1.ang() * H2.lin().transpose(),
                        H1.lin() * H2.ang().transpose(), H1.lin() * H2.lin().transpose());
}

template <int N>
inline ChMatrixNN<N> operator*(const ChVelMatT<N>& H1, const ChVelMat<N>& H2) {
    return H1.ang().transpose() * H2.ang() + H1.lin().transpose() * H2.lin();
}

// -----------------------------------------------------------------------------

/// Insertion of a ChSpatialMat to output stream.
inline std::ostream& operator<<(std::ostream& out, const ChSpatialMat& S) {
    for (int i = 0; i < 3; i++)
        out << S.A00().row(i) << " " << S.A01().row(i) << std::endl;
    for (int i = 0; i < 3; i++)
        out << S.A10().row(i) << " " << S.A11().row(i) << std::endl;

    return out;
}

/// Insertion of a ChShiftMat to output stream.
inline std::ostream& operator<<(std::ostream& out, const ChShiftMat& P) {
    auto zero = ChMatrix33d::Zero();
    auto eye = ChMatrix33d::Identity();
    auto x = ChStarMatrix33d(-P.l());

    for (int i = 0; i < 3; i++)
        out << eye.row(i) << " " << zero.row(i) << std::endl;
    for (int i = 0; i < 3; i++)
        out << x.row(i) << " " << eye.row(i) << std::endl;

    return out;
}

/// Insertion of a ChVelMat to output stream.
template <int N>
inline std::ostream& operator<<(std::ostream& out, const ChVelMat<N>& H) {
    out << H.ang() << std::endl;
    out << H.lin() << std::endl;

    return out;
}

/// @} chrono_soa

}  // namespace soa
}  // namespace chrono

#endif
