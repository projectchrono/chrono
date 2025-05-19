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

#ifndef CH_VECTOR3_H
#define CH_VECTOR3_H

#include <algorithm>
#include <cmath>
#include <limits>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChClassFactory.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/serialization/ChArchive.h"
#include "chrono/serialization/ChOutputASCII.h"

namespace chrono {

/// Definition of general purpose 3d vector variables, such as points in 3D.
/// This class implements the vectorial algebra in 3D (Gibbs products).
/// ChVector3 is templated by precision, with default 'double'.
///
/// Further info at the @ref mathematical_objects manual page.
template <class Real = double>
class ChVector3 {
  public:
    // CONSTRUCTORS

    ChVector3();
    ChVector3(Real x, Real y, Real z);
    ChVector3(Real a);
    ChVector3(const ChVector3<Real>& other);

    /// Copy constructor with type change.
    template <class RealB>
    ChVector3(const ChVector3<RealB>& other);

    /// Access to components
    Real& x() { return m_data[0]; }
    Real& y() { return m_data[1]; }
    Real& z() { return m_data[2]; }
    const Real& x() const { return m_data[0]; }
    const Real& y() const { return m_data[1]; }
    const Real& z() const { return m_data[2]; }

    /// Access to underlying array storage.
    Real* data() { return m_data; }
    const Real* data() const { return m_data; }

    // EIGEN INTER-OPERABILITY

    /// Construct a 3d vector from an Eigen vector expression.
    template <typename Derived>
    ChVector3(const Eigen::MatrixBase<Derived>& vec,
              typename std::enable_if<(Derived::MaxRowsAtCompileTime == 1 || Derived::MaxColsAtCompileTime == 1),
                                      Derived>::type* = 0) {
        m_data[0] = vec(0);
        m_data[1] = vec(1);
        m_data[2] = vec(2);
    }

    /// View this 3d vector as an Eigen vector.
    Eigen::Map<Eigen::Matrix<Real, 3, 1>> eigen() { return Eigen::Map<Eigen::Matrix<Real, 3, 1>>(m_data); }
    Eigen::Map<const Eigen::Matrix<Real, 3, 1>> eigen() const {
        return Eigen::Map<const Eigen::Matrix<Real, 3, 1>>(m_data);
    }

    /// Assign an Eigen vector expression to this 3d vector.
    template <typename Derived>
    ChVector3& operator=(const Eigen::MatrixBase<Derived>& vec) {
        m_data[0] = vec(0);
        m_data[1] = vec(1);
        m_data[2] = vec(2);
        return *this;
    }

    // SET FUNCTIONS

    /// Set the three values of the vector at once.
    void Set(Real x, Real y, Real z);

    /// Set the vector as a copy of another vector.
    void Set(const ChVector3<Real>& v);

    /// Set all the vector components ts to the same scalar.
    void Set(Real s);

    /// Set the vector to the null vector.
    void SetNull();

    /// Return true if this vector is the null vector.
    bool IsNull() const;

    /// Return true if this vector is equal to another vector.
    bool Equals(const ChVector3<Real>& other) const;

    /// Return true if this vector is equal to another vector, within a tolerance 'tol'.
    bool Equals(const ChVector3<Real>& other, Real tol) const;

    // VECTOR NORMS

    /// Compute the euclidean norm of the vector, that is its length or magnitude.
    Real Length() const;

    /// Compute the squared euclidean norm of the vector.
    Real Length2() const;

    /// Compute the infinity norm of the vector, that is the maximum absolute value of one of its elements.
    Real LengthInf() const;

    // OPERATORS OVERLOADING
    //
    // Note: c++ automatically creates temporary objects to store intermediate
    // results in long formulas, such as a= b*c*d, so the usage of operators
    // may give slower results than a wise (less readable however) usage of
    // Dot(), Cross() etc.. Also pay attention to C++ operator precedence rules!

    /// Subscript operator.
    Real& operator[](unsigned index);
    const Real& operator[](unsigned index) const;

    /// Assignment operator (copy from another vector).
    ChVector3<Real>& operator=(const ChVector3<Real>& other);

    /// Assignment operator (copy from another vector) with type change.
    template <class RealB>
    ChVector3<Real>& operator=(const ChVector3<RealB>& other);

    /// Operators for sign change.
    ChVector3<Real> operator+() const;
    ChVector3<Real> operator-() const;

    /// Operator for vector sum.
    ChVector3<Real> operator+(const ChVector3<Real>& other) const;
    ChVector3<Real>& operator+=(const ChVector3<Real>& other);

    /// Operator for vector difference.
    ChVector3<Real> operator-(const ChVector3<Real>& other) const;
    ChVector3<Real>& operator-=(const ChVector3<Real>& other);

    /// Operator for element-wise multiplication.
    /// Note that this is neither dot product nor cross product.
    ChVector3<Real> operator*(const ChVector3<Real>& other) const;
    ChVector3<Real>& operator*=(const ChVector3<Real>& other);

    /// Operator for element-wise division.
    /// Note that 3D vector algebra is a skew field, non-divisional algebra,
    /// so this division operation is just an element-by element division.
    ChVector3<Real> operator/(const ChVector3<Real>& other) const;
    ChVector3<Real>& operator/=(const ChVector3<Real>& other);

    /// Operator for scaling the vector by a scalar value, as V*s
    ChVector3<Real> operator*(Real s) const;
    ChVector3<Real>& operator*=(Real s);

    /// Operator for scaling the vector by inverse of a scalar value, as v/s
    ChVector3<Real> operator/(Real v) const;
    ChVector3<Real>& operator/=(Real v);

    /// Operator for dot product: A^B means the scalar dot-product A*B
    /// Note: pay attention to operator low precedence (see C++ precedence rules!)
    Real operator^(const ChVector3<Real>& other) const;

    /// Operator for cross product: A%B means the vector cross-product AxB
    /// Note: pay attention to operator low precedence (see C++ precedence rules!)
    ChVector3<Real> operator%(const ChVector3<Real>& other) const;
    ChVector3<Real>& operator%=(const ChVector3<Real>& other);

    /// Component-wise comparison operators
    bool operator<=(const ChVector3<Real>& other) const;
    bool operator>=(const ChVector3<Real>& other) const;
    bool operator<(const ChVector3<Real>& other) const;
    bool operator>(const ChVector3<Real>& other) const;
    bool operator==(const ChVector3<Real>& other) const;
    bool operator!=(const ChVector3<Real>& other) const;

    // FUNCTIONS

    /// Set this vector to the sum of A and B: this = A + B
    void Add(const ChVector3<Real>& A, const ChVector3<Real>& B);

    /// Set this vector to the difference of A and B: this = A - B
    void Sub(const ChVector3<Real>& A, const ChVector3<Real>& B);

    /// Set this vector to the product of a vector A and scalar s: this = A * s
    void Mul(const ChVector3<Real>& A, Real s);

    /// Scale this vector by a scalar: this *= s
    void Scale(Real s);

    /// Set this vector to the cross product of A and B: this = A x B
    void Cross(const ChVector3<Real>& A, const ChVector3<Real>& B);

    /// Return the cross product with another vector: result = this x other
    ChVector3<Real> Cross(const ChVector3<Real> other) const;

    /// Return the dot product with another vector: result = this ^ B
    Real Dot(const ChVector3<Real>& B) const;

    /// Normalize this vector in place, so that its euclidean length is 1.
    /// Return false if the original vector had zero length (in which case the vector
    /// is set to [1,0,0]) and return true otherwise.
    bool Normalize();

    /// Return a normalized copy of this vector, with euclidean length = 1.
    /// Not to be confused with Normalize() which normalizes in place.
    ChVector3<Real> GetNormalized() const;

    /// Impose a new length to the vector, keeping the direction unchanged.
    void SetLength(Real s);

    /// Output three orthonormal vectors considering this vector along X axis.
    /// Optionally, the \a z_sugg vector can be used to suggest the Z axis.
    /// It is recommended to set \a y_sugg to be not parallel to this vector.
    /// The Z axis will be orthogonal to X and \a y_sugg.
    /// Rely on Gram-Schmidt orthonormalization.
    void GetDirectionAxesAsX(ChVector3<Real>& Vx,
                             ChVector3<Real>& Vy,
                             ChVector3<Real>& Vz,
                             ChVector3<Real> y_sugg = ChVector3<Real>(0, 1, 0)) const;

    /// Output three orthonormal vectors considering this vector along Y axis.
    /// Optionally, the \a z_sugg vector can be used to suggest the Z axis.
    /// It is recommended to set \a z_sugg to be not parallel to this vector.
    /// Rely on Gram-Schmidt orthonormalization.
    void GetDirectionAxesAsY(ChVector3<Real>& Vx,
                             ChVector3<Real>& Vy,
                             ChVector3<Real>& Vz,
                             ChVector3<Real> z_sugg = ChVector3<Real>(0, 0, 1)) const;

    /// Output three orthonormal vectors considering this vector along Y axis.
    /// Optionally, the \a x_sugg vector can be used to suggest the X axis.
    /// It is recommended to set \a x_sugg to be not parallel to this vector.
    /// Rely on Gram-Schmidt orthonormalization.
    void GetDirectionAxesAsZ(ChVector3<Real>& Vx,
                             ChVector3<Real>& Vy,
                             ChVector3<Real>& Vz,
                             ChVector3<Real> x_sugg = ChVector3<Real>(1, 0, 0)) const;

    /// Return the index of the largest component in absolute value.
    int GetMaxComponent() const;

    /// Return a unit vector orthogonal to this vector
    ChVector3<Real> GetOrthogonalVector() const;

    /// Method to allow serialization of transient m_data to archives.
    void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient m_data from archives.
    void ArchiveIn(ChArchiveIn& archive_in);

  private:
    Real m_data[3];

    /// Declaration of friend classes
    template <typename RealB>
    friend class ChVector3;
};

CH_CLASS_VERSION(ChVector3<double>, 0)

// -----------------------------------------------------------------------------

/// Alias for double-precision vectors.
/// <pre>
/// Instead of writing
///    ChVector3<double> v;
/// or
///    ChVector3d v;
/// you can use:
///    ChVector3d v;
/// </pre>
typedef ChVector3<double> ChVector3d;

/// Alias for single-precision vectors.
/// <pre>
/// Instead of writing
///    ChVector3<float> v;
/// you can use:
///    ChVector3f v;
/// </pre>
typedef ChVector3<float> ChVector3f;

/// Alias for integer vectors.
/// <pre>
/// Instead of writing
///    ChVector3<int> v;
/// you can use:
///    ChVector3i v;
/// </pre>
typedef ChVector3<int> ChVector3i;

/// Alias for bool vectors.
/// <pre>
/// Instead of writing
///    ChVector3<bool> v;
/// you can use:
///    ChVector3b v;
/// </pre>
typedef ChVector3<bool> ChVector3b;

// -----------------------------------------------------------------------------

/// Definition of a wrench (force + torque).
/// The wrench is assumed to be represented in some frame with the force application point at the frame origin.
template <class Real = double>
struct ChWrench {
    ChVector3<Real> force;
    ChVector3<Real> torque;
};

/// Alias for double-precision wrenches.
typedef ChWrench<double> ChWrenchd;

/// Alias for single-precision wrenches.
typedef ChWrench<float> ChWrenchf;

// -----------------------------------------------------------------------------
// CONSTANTS

ChApi extern const ChVector3d VNULL;
ChApi extern const ChVector3d VECT_X;
ChApi extern const ChVector3d VECT_Y;
ChApi extern const ChVector3d VECT_Z;

// -----------------------------------------------------------------------------
// STATIC VECTOR MATH OPERATIONS

// These functions are here for users who prefer to use global functions instead of ChVector3 member functions.

template <class RealA, class RealB>
RealA Vdot(const ChVector3<RealA>& va, const ChVector3<RealB>& vb) {
    return (RealA)((va.x() * vb.x()) + (va.y() * vb.y()) + (va.z() * vb.z()));
}

template <class RealA>
void Vset(ChVector3<RealA>& v, RealA mx, RealA my, RealA mz) {
    v.x() = mx;
    v.y() = my;
    v.z() = mz;
}

template <class RealA, class RealB>
ChVector3<RealA> Vadd(const ChVector3<RealA>& va, const ChVector3<RealB>& vb) {
    ChVector3<RealA> result;
    result.x() = va.x() + vb.x();
    result.y() = va.y() + vb.y();
    result.z() = va.z() + vb.z();
    return result;
}

template <class RealA, class RealB>
ChVector3<RealA> Vsub(const ChVector3<RealA>& va, const ChVector3<RealB>& vb) {
    ChVector3<RealA> result;
    result.x() = va.x() - vb.x();
    result.y() = va.y() - vb.y();
    result.z() = va.z() - vb.z();
    return result;
}

template <class RealA, class RealB>
ChVector3<RealA> Vcross(const ChVector3<RealA>& va, const ChVector3<RealB>& vb) {
    ChVector3<RealA> result;
    result.x() = (va.y() * vb.z()) - (va.z() * vb.y());
    result.y() = (va.z() * vb.x()) - (va.x() * vb.z());
    result.z() = (va.x() * vb.y()) - (va.y() * vb.x());
    return result;
}

template <class RealA, class RealB>
ChVector3<RealA> Vmul(const ChVector3<RealA>& va, RealB fact) {
    ChVector3<RealA> result;
    result.x() = va.x() * (RealA)fact;
    result.y() = va.y() * (RealA)fact;
    result.z() = va.z() * (RealA)fact;
    return result;
}

template <class RealA>
RealA Vlength(const ChVector3<RealA>& va) {
    return (RealA)va.Length();
}

template <class RealA>
ChVector3<RealA> Vnorm(const ChVector3<RealA>& va) {
    ChVector3<RealA> result(va);
    result.Normalize();
    return result;
}

template <class RealA, class RealB>
bool Vequal(const ChVector3<RealA>& va, const ChVector3<RealB>& vb) {
    return (va == vb);
}

template <class RealA>
bool Vnotnull(const ChVector3<RealA>& va) {
    return (va.x() != 0 || va.y() != 0 || va.z() != 0);
}

template <class RealA>
ChVector3<RealA> Vmin(const ChVector3<RealA>& va, const ChVector3<RealA>& vb) {
    ChVector3<RealA> result;
    result.x() = std::min(va.x(), vb.x());
    result.y() = std::min(va.y(), vb.y());
    result.z() = std::min(va.z(), vb.z());
    return result;
}

template <class RealA>
ChVector3<RealA> Vmax(const ChVector3<RealA>& va, const ChVector3<RealA>& vb) {
    ChVector3<RealA> result;
    result.x() = std::max(va.x(), vb.x());
    result.y() = std::max(va.y(), vb.y());
    result.z() = std::max(va.z(), vb.z());
    return result;
}

// Gets the zenith angle of a unit vector respect to YZ plane  ***OBSOLETE
template <class RealA>
double VangleYZplane(const ChVector3<RealA>& va) {
    return std::asin(Vdot(va, ChVector3<RealA>(1, 0, 0)));
}

// Gets the zenith angle of a unit vector respect to YZ plane  ***OBSOLETE
template <class RealA>
double VangleYZplaneNorm(const ChVector3<RealA>& va) {
    return std::acos(Vdot(va, ChVector3<RealA>(1, 0, 0)));
}

// Gets the angle of the projection on the YZ plane respect to
// the Y vector, as spinning about X.
template <class RealA>
double VangleRX(const ChVector3<RealA>& va) {
    ChVector3<RealA> vproj;
    vproj.x() = 0;
    vproj.y() = va.y();
    vproj.z() = va.z();
    vproj = Vnorm(vproj);
    if (vproj.x() == 1)
        return 0;
    return std::acos(vproj.y());
}

// The reverse of the two previous functions, gets the vector
// given the angle above the normal to YZ plane and the angle
// of rotation on X
template <class RealA>
ChVector3<RealA> VfromPolar(double norm_angle, double pol_angle) {
    ChVector3d res;
    double projlen;
    res.x() = std::cos(norm_angle);  // 1) rot 'norm.angle'about z
    res.y() = std::sin(norm_angle);
    res.z() = 0;
    projlen = res.y();
    res.y() = projlen * std::cos(pol_angle);
    res.z() = projlen * std::sin(pol_angle);
    return res;
}

/// Insertion of a 3D vector to output stream.
template <typename Real>
inline std::ostream& operator<<(std::ostream& out, const ChVector3<Real>& v) {
    out << v.x() << "  " << v.y() << "  " << v.z();
    return out;
}

// =============================================================================
// IMPLEMENTATION OF ChVector3<Real> methods
// =============================================================================

// -----------------------------------------------------------------------------
// Constructors

template <class Real>
inline ChVector3<Real>::ChVector3() {
    m_data[0] = 0;
    m_data[1] = 0;
    m_data[2] = 0;
}

template <class Real>
inline ChVector3<Real>::ChVector3(Real a) {
    m_data[0] = a;
    m_data[1] = a;
    m_data[2] = a;
}

template <class Real>
inline ChVector3<Real>::ChVector3(Real x, Real y, Real z) {
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;
}

template <class Real>
inline ChVector3<Real>::ChVector3(const ChVector3<Real>& other) {
    m_data[0] = other.m_data[0];
    m_data[1] = other.m_data[1];
    m_data[2] = other.m_data[2];
}

template <class Real>
template <class RealB>
inline ChVector3<Real>::ChVector3(const ChVector3<RealB>& other) {
    m_data[0] = static_cast<Real>(other.m_data[0]);
    m_data[1] = static_cast<Real>(other.m_data[1]);
    m_data[2] = static_cast<Real>(other.m_data[2]);
}

// -----------------------------------------------------------------------------
// Subscript operators

template <class Real>
inline Real& ChVector3<Real>::operator[](unsigned index) {
    assert(index < 3);
    return m_data[index];
}

template <class Real>
inline const Real& ChVector3<Real>::operator[](unsigned index) const {
    assert(index < 3);
    return m_data[index];
}

// -----------------------------------------------------------------------------
// Assignments

template <class Real>
inline ChVector3<Real>& ChVector3<Real>::operator=(const ChVector3<Real>& other) {
    if (&other == this)
        return *this;
    m_data[0] = other.m_data[0];
    m_data[1] = other.m_data[1];
    m_data[2] = other.m_data[2];
    return *this;
}

template <class Real>
template <class RealB>
inline ChVector3<Real>& ChVector3<Real>::operator=(const ChVector3<RealB>& other) {
    m_data[0] = static_cast<Real>(other.m_data[0]);
    m_data[1] = static_cast<Real>(other.m_data[1]);
    m_data[2] = static_cast<Real>(other.m_data[2]);
    return *this;
}

// -----------------------------------------------------------------------------
// Sign operators

template <class Real>
inline ChVector3<Real> ChVector3<Real>::operator+() const {
    return *this;
}

template <class Real>
inline ChVector3<Real> ChVector3<Real>::operator-() const {
    return ChVector3<Real>(-m_data[0], -m_data[1], -m_data[2]);
}

// -----------------------------------------------------------------------------
// Arithmetic operations

template <class Real>
inline ChVector3<Real> ChVector3<Real>::operator+(const ChVector3<Real>& other) const {
    ChVector3<Real> v;

    v.m_data[0] = m_data[0] + other.m_data[0];
    v.m_data[1] = m_data[1] + other.m_data[1];
    v.m_data[2] = m_data[2] + other.m_data[2];

    return v;
}

template <class Real>
inline ChVector3<Real> ChVector3<Real>::operator-(const ChVector3<Real>& other) const {
    ChVector3<Real> v;

    v.m_data[0] = m_data[0] - other.m_data[0];
    v.m_data[1] = m_data[1] - other.m_data[1];
    v.m_data[2] = m_data[2] - other.m_data[2];

    return v;
}

template <class Real>
inline ChVector3<Real> ChVector3<Real>::operator*(const ChVector3<Real>& other) const {
    ChVector3<Real> v;

    v.m_data[0] = m_data[0] * other.m_data[0];
    v.m_data[1] = m_data[1] * other.m_data[1];
    v.m_data[2] = m_data[2] * other.m_data[2];

    return v;
}

template <class Real>
inline ChVector3<Real> ChVector3<Real>::operator/(const ChVector3<Real>& other) const {
    ChVector3<Real> v;

    v.m_data[0] = m_data[0] / other.m_data[0];
    v.m_data[1] = m_data[1] / other.m_data[1];
    v.m_data[2] = m_data[2] / other.m_data[2];

    return v;
}

template <class Real>
inline ChVector3<Real> ChVector3<Real>::operator*(Real s) const {
    ChVector3<Real> v;

    v.m_data[0] = m_data[0] * s;
    v.m_data[1] = m_data[1] * s;
    v.m_data[2] = m_data[2] * s;

    return v;
}

template <class Real>
inline ChVector3<Real> ChVector3<Real>::operator/(Real s) const {
    Real oos = 1 / s;
    ChVector3<Real> v;

    v.m_data[0] = m_data[0] * oos;
    v.m_data[1] = m_data[1] * oos;
    v.m_data[2] = m_data[2] * oos;

    return v;
}

template <class Real>
inline ChVector3<Real>& ChVector3<Real>::operator+=(const ChVector3<Real>& other) {
    m_data[0] += other.m_data[0];
    m_data[1] += other.m_data[1];
    m_data[2] += other.m_data[2];

    return *this;
}

template <class Real>
inline ChVector3<Real>& ChVector3<Real>::operator-=(const ChVector3<Real>& other) {
    m_data[0] -= other.m_data[0];
    m_data[1] -= other.m_data[1];
    m_data[2] -= other.m_data[2];

    return *this;
}

template <class Real>
inline ChVector3<Real>& ChVector3<Real>::operator*=(const ChVector3<Real>& other) {
    m_data[0] *= other.m_data[0];
    m_data[1] *= other.m_data[1];
    m_data[2] *= other.m_data[2];

    return *this;
}

template <class Real>
inline ChVector3<Real>& ChVector3<Real>::operator/=(const ChVector3<Real>& other) {
    m_data[0] /= other.m_data[0];
    m_data[1] /= other.m_data[1];
    m_data[2] /= other.m_data[2];

    return *this;
}

template <class Real>
inline ChVector3<Real>& ChVector3<Real>::operator*=(Real s) {
    m_data[0] *= s;
    m_data[1] *= s;
    m_data[2] *= s;

    return *this;
}

template <class Real>
inline ChVector3<Real>& ChVector3<Real>::operator/=(Real s) {
    Real oos = 1 / s;

    m_data[0] *= oos;
    m_data[1] *= oos;
    m_data[2] *= oos;

    return *this;
}

// -----------------------------------------------------------------------------
// Vector operations

template <class Real>
inline Real ChVector3<Real>::operator^(const ChVector3<Real>& other) const {
    return this->Dot(other);
}

template <class Real>
ChVector3<Real> ChVector3<Real>::operator%(const ChVector3<Real>& other) const {
    ChVector3<Real> v;
    v.Cross(*this, other);
    return v;
}

template <class Real>
inline ChVector3<Real>& ChVector3<Real>::operator%=(const ChVector3<Real>& other) {
    this->Cross(*this, other);
    return *this;
}

// -----------------------------------------------------------------------------
// Comparison operations

template <class Real>
inline bool ChVector3<Real>::operator<=(const ChVector3<Real>& other) const {
    return m_data[0] <= other.m_data[0] && m_data[1] <= other.m_data[1] && m_data[2] <= other.m_data[2];
}

template <class Real>
inline bool ChVector3<Real>::operator>=(const ChVector3<Real>& other) const {
    return m_data[0] >= other.m_data[0] && m_data[1] >= other.m_data[1] && m_data[2] >= other.m_data[2];
}

template <class Real>
inline bool ChVector3<Real>::operator<(const ChVector3<Real>& other) const {
    return m_data[0] < other.m_data[0] && m_data[1] < other.m_data[1] && m_data[2] < other.m_data[2];
}

template <class Real>
inline bool ChVector3<Real>::operator>(const ChVector3<Real>& other) const {
    return m_data[0] > other.m_data[0] && m_data[1] > other.m_data[1] && m_data[2] > other.m_data[2];
}

template <class Real>
inline bool ChVector3<Real>::operator==(const ChVector3<Real>& other) const {
    return other.m_data[0] == m_data[0] && other.m_data[1] == m_data[1] && other.m_data[2] == m_data[2];
}

template <class Real>
inline bool ChVector3<Real>::operator!=(const ChVector3<Real>& other) const {
    return !(*this == other);
}

// -----------------------------------------------------------------------------
// Functions

template <class Real>
inline void ChVector3<Real>::Set(Real x, Real y, Real z) {
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;
}

template <class Real>
inline void ChVector3<Real>::Set(const ChVector3<Real>& v) {
    m_data[0] = v.m_data[0];
    m_data[1] = v.m_data[1];
    m_data[2] = v.m_data[2];
}

template <class Real>
inline void ChVector3<Real>::Set(Real s) {
    m_data[0] = s;
    m_data[1] = s;
    m_data[2] = s;
}

/// Sets the vector as a null vector
template <class Real>
inline void ChVector3<Real>::SetNull() {
    m_data[0] = 0;
    m_data[1] = 0;
    m_data[2] = 0;
}

template <class Real>
inline bool ChVector3<Real>::IsNull() const {
    return m_data[0] == 0 && m_data[1] == 0 && m_data[2] == 0;
}

template <class Real>
inline bool ChVector3<Real>::Equals(const ChVector3<Real>& other) const {
    return (other.m_data[0] == m_data[0]) && (other.m_data[1] == m_data[1]) && (other.m_data[2] == m_data[2]);
}

template <class Real>
inline bool ChVector3<Real>::Equals(const ChVector3<Real>& other, Real tol) const {
    return (std::abs(other.m_data[0] - m_data[0]) < tol) && (std::abs(other.m_data[1] - m_data[1]) < tol) &&
           (std::abs(other.m_data[2] - m_data[2]) < tol);
}

template <class Real>
inline void ChVector3<Real>::Add(const ChVector3<Real>& A, const ChVector3<Real>& B) {
    m_data[0] = A.m_data[0] + B.m_data[0];
    m_data[1] = A.m_data[1] + B.m_data[1];
    m_data[2] = A.m_data[2] + B.m_data[2];
}

template <class Real>
inline void ChVector3<Real>::Sub(const ChVector3<Real>& A, const ChVector3<Real>& B) {
    m_data[0] = A.m_data[0] - B.m_data[0];
    m_data[1] = A.m_data[1] - B.m_data[1];
    m_data[2] = A.m_data[2] - B.m_data[2];
}

template <class Real>
inline void ChVector3<Real>::Mul(const ChVector3<Real>& A, Real s) {
    m_data[0] = A.m_data[0] * s;
    m_data[1] = A.m_data[1] * s;
    m_data[2] = A.m_data[2] * s;
}

template <class Real>
inline void ChVector3<Real>::Scale(Real s) {
    m_data[0] *= s;
    m_data[1] *= s;
    m_data[2] *= s;
}

template <class Real>
inline void ChVector3<Real>::Cross(const ChVector3<Real>& A, const ChVector3<Real>& B) {
    m_data[0] = (A.m_data[1] * B.m_data[2]) - (A.m_data[2] * B.m_data[1]);
    m_data[1] = (A.m_data[2] * B.m_data[0]) - (A.m_data[0] * B.m_data[2]);
    m_data[2] = (A.m_data[0] * B.m_data[1]) - (A.m_data[1] * B.m_data[0]);
}

template <class Real>
inline ChVector3<Real> ChVector3<Real>::Cross(const ChVector3<Real> other) const {
    ChVector3<Real> v;
    v.Cross(*this, other);
    return v;
}

template <class Real>
inline Real ChVector3<Real>::Dot(const ChVector3<Real>& B) const {
    return (m_data[0] * B.m_data[0]) + (m_data[1] * B.m_data[1]) + (m_data[2] * B.m_data[2]);
}

template <class Real>
inline Real ChVector3<Real>::Length() const {
    return std::sqrt(Length2());
}

template <class Real>
inline Real ChVector3<Real>::Length2() const {
    return this->Dot(*this);
}

template <class Real>
inline Real ChVector3<Real>::LengthInf() const {
    return std::max(std::max(std::abs(m_data[0]), std::abs(m_data[1])), std::abs(m_data[2]));
}

template <class Real>
inline bool ChVector3<Real>::Normalize() {
    Real length = this->Length();
    if (length < std::numeric_limits<Real>::min()) {
        m_data[0] = 1;
        m_data[1] = 0;
        m_data[2] = 0;
        return false;
    }
    this->Scale(1 / length);
    return true;
}

template <class Real>
inline ChVector3<Real> ChVector3<Real>::GetNormalized() const {
    ChVector3<Real> v(*this);
    v.Normalize();
    return v;
}

template <class Real>
inline void ChVector3<Real>::SetLength(Real s) {
    Normalize();
    Scale(s);
}

template <class Real>
inline void ChVector3<Real>::GetDirectionAxesAsX(ChVector3<Real>& Vx,
                                                 ChVector3<Real>& Vy,
                                                 ChVector3<Real>& Vz,
                                                 ChVector3<Real> y_sugg) const {
    Vx = *this;
    bool success = Vx.Normalize();
    if (!success)
        Vx = ChVector3<Real>(1, 0, 0);

    Vz.Cross(Vx, y_sugg);
    success = Vz.Normalize();
    if (!success) {
        char idx = 0;
        while (!success) {
            y_sugg[idx] += 1.0;
            Vz.Cross(Vx, y_sugg);
            success = Vz.Normalize();
            ++idx;
        }
    }

    Vy.Cross(Vz, Vx);
}

template <class Real>
inline void ChVector3<Real>::GetDirectionAxesAsY(ChVector3<Real>& Vx,
                                                 ChVector3<Real>& Vy,
                                                 ChVector3<Real>& Vz,
                                                 ChVector3<Real> z_sugg) const {
    Vy = *this;
    bool success = Vy.Normalize();
    if (!success)
        Vy = ChVector3<Real>(0, 1, 0);

    Vx.Cross(Vy, z_sugg);
    success = Vx.Normalize();
    if (!success) {
        char idx = 0;
        while (!success) {
            z_sugg[idx] += 1.0;
            Vx.Cross(Vy, z_sugg);
            success = Vx.Normalize();
            ++idx;
        }
    }

    Vy.Cross(Vz, Vx);
}

template <class Real>
inline void ChVector3<Real>::GetDirectionAxesAsZ(ChVector3<Real>& Vx,
                                                 ChVector3<Real>& Vy,
                                                 ChVector3<Real>& Vz,
                                                 ChVector3<Real> x_sugg) const {
    Vz = *this;
    bool success = Vz.Normalize();
    if (!success)
        Vz = ChVector3<Real>(0, 0, 1);

    Vy.Cross(Vz, x_sugg);
    success = Vy.Normalize();

    if (!success) {
        char idx = 0;
        while (!success) {
            x_sugg[idx] += 1.0;
            Vy.Cross(Vz, x_sugg);
            success = Vy.Normalize();
            ++idx;
        }
    }

    Vx.Cross(Vy, Vz);
}

template <class Real>
inline int ChVector3<Real>::GetMaxComponent() const {
    int idx = 0;
    Real max = std::abs(m_data[0]);
    if (std::abs(m_data[1]) > max) {
        idx = 1;
        max = m_data[1];
    }
    if (std::abs(m_data[2]) > max) {
        idx = 2;
        max = m_data[2];
    }
    return idx;
}

template <class Real>
inline ChVector3<Real> ChVector3<Real>::GetOrthogonalVector() const {
    int idx1 = this->GetMaxComponent();
    int idx2 = (idx1 + 1) % 3;  // cycle to the next component
    int idx3 = (idx2 + 1) % 3;  // cycle to the next component

    // Construct v2 by rotating in the plane containing the maximum component
    ChVector3<Real> v2(-m_data[idx2], m_data[idx1], m_data[idx3]);

    // Construct the normal vector
    ChVector3<Real> ortho = Cross(v2);
    ortho.Normalize();
    return ortho;
}

// -----------------------------------------------------------------------------
// Streaming operations

template <class Real>
inline void ChVector3<Real>::ArchiveOut(ChArchiveOut& archive_out) {
    // suggested: use versioning
    archive_out.VersionWrite<ChVector3d>();  // must use specialized template (any)
    // stream out all member m_data
    archive_out << CHNVP(m_data[0], "x");
    archive_out << CHNVP(m_data[1], "y");
    archive_out << CHNVP(m_data[2], "z");
}

template <class Real>
inline void ChVector3<Real>::ArchiveIn(ChArchiveIn& archive_in) {
    // suggested: use versioning
    /*int version =*/archive_in.VersionRead<ChVector3d>();  // must use specialized template (any)
    // stream in all member m_data
    archive_in >> CHNVP(m_data[0], "x");
    archive_in >> CHNVP(m_data[1], "y");
    archive_in >> CHNVP(m_data[2], "z");
}

// -----------------------------------------------------------------------------
// Reversed operators

/// Operator for scaling the vector by a scalar value, as s*V.
template <class Real>
ChVector3<Real> operator*(Real s, const ChVector3<Real>& V) {
    return ChVector3<Real>(V.x() * s, V.y() * s, V.z() * s);
}

/// Operator for scaling an integer vector by a double scalar, as s*V.
ChApi ChVector3d operator*(double s, const ChVector3i& V);

}  // end namespace chrono

#endif
