// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Definition of a 2-D vector.
//
// =============================================================================

#ifndef CHVECTOR2_H
#define CHVECTOR2_H

#include <cmath>
#include <iomanip>
#include <iostream>

#include "chrono/core/ChMathematics.h"
#include "chrono/serialization/ChArchive.h"

namespace chrono {

/// Definition of a general purpose 2d vector.
/// ChVector2 is templated by precision, with default 'double'.
template <class Real = double>
class ChVector2 {
  public:
    // CONSTRUCTORS

    ChVector2();
    ChVector2(Real x, Real y);
    ChVector2(Real a);
    ChVector2(const ChVector2<Real>& other);

    /// Copy constructor with type change.
    template <class RealB>
    ChVector2(const ChVector2<RealB>& other);

    /// Access to components
    Real& x() { return data[0]; }
    Real& y() { return data[1]; }
    const Real& x() const { return data[0]; }
    const Real& y() const { return data[1]; }

    // SET FUNCTIONS

    /// Set the two values of the vector at once.
    void Set(Real x, Real y);

    /// Sets the vector as a copy of another vector.
    void Set(const ChVector2<Real>& v);

    /// Set all the vector components ts to the same scalar.
    void Set(Real s);

    /// Set the vector to the null vector.
    void SetNull();

    /// Return true if this vector is the null vector.
    bool IsNull() const;

    /// Return true if this vector is equal to another vector.
    bool Equals(const ChVector2<Real>& other) const;

    /// Return true if this vector is equal to another vector, within a tolerance 'tol'.
    bool Equals(const ChVector2<Real>& other, Real tol) const;

    // VECTOR NORMS

    /// Compute the euclidean norm of the vector, that is its length or magnitude.
    Real Length() const;

    /// Compute the squared euclidean norm of the vector.
    Real Length2() const;

    /// Compute the infinite norm of the vector,
    /// that is the maximum absolute value of one of its elements.
    Real LengthInf() const;

    // OPERATORS OVERLOADING

    /// Subscript operator.
    Real& operator[](unsigned index);
    const Real& operator[](unsigned index) const;

    /// Assignment operator (copy from another vector).
    ChVector2<Real>& operator=(const ChVector2<Real>& other);

    /// Assignment operator (copy from another vector) with type change.
    template <class RealB>
    ChVector2<Real>& operator=(const ChVector2<RealB>& other);

    /// Operators for sign change.
    ChVector2<Real> operator+() const;
    ChVector2<Real> operator-() const;

    /// Operator for vector sum.
    ChVector2<Real> operator+(const ChVector2<Real>& other) const;
    ChVector2<Real>& operator+=(const ChVector2<Real>& other);

    /// Operator for vector difference.
    ChVector2<Real> operator-(const ChVector2<Real>& other) const;
    ChVector2<Real>& operator-=(const ChVector2<Real>& other);

    /// Operator for element-wise multiplication.
    /// Note that this is not the dot product.
    ChVector2<Real> operator*(const ChVector2<Real>& other) const;
    ChVector2<Real>& operator*=(const ChVector2<Real>& other);

    /// Operator for element-wise division.
    ChVector2<Real> operator/(const ChVector2<Real>& other) const;
    ChVector2<Real>& operator/=(const ChVector2<Real>& other);

    /// Operator for scaling the vector by a scalar value, as V*s
    ChVector2<Real> operator*(const Real s) const;
    ChVector2<Real>& operator*=(const Real s);

    /// Operator for scaling the vector by inverse of a scalar value, as v/s
    ChVector2<Real> operator/(const Real v) const;
    ChVector2<Real>& operator/=(const Real v);

    /// Operator for dot product: A^B means the scalar dot-product A*B
    /// Note: pay attention to operator low precedence (see C++ precedence rules!)
    Real operator^(const ChVector2<Real>& other) const;

    /// Component-wise comparison operators
    bool operator<=(const ChVector2<Real>& other) const;
    bool operator>=(const ChVector2<Real>& other) const;
    bool operator<(const ChVector2<Real>& other) const;
    bool operator>(const ChVector2<Real>& other) const;
    bool operator==(const ChVector2<Real>& other) const;
    bool operator!=(const ChVector2<Real>& other) const;

    // FUNCTIONS

    /// Set this vector to the sum of A and B: this = A + B
    void Add(const ChVector2<Real>& A, const ChVector2<Real>& B);

    /// Set this vector to the difference of A and B: this = A - B
    void Sub(const ChVector2<Real>& A, const ChVector2<Real>& B);

    /// Set this vector to the product of a vector A and scalar s: this = A * s
    void Mul(const ChVector2<Real>& A, const Real s);

    /// Scale this vector by a scalar: this *= s
    void Scale(const Real s);

    /// Return the dot product with another vector: result = this ^ B
    Real Dot(const ChVector2<Real>& B) const;

    /// Normalize this vector in place, so that its euclidean length is 1.
    /// Return false if the original vector had zero length (in which case the vector
    /// set to [1,0,0]) and return true otherwise.
    bool Normalize();

    /// Return a normalized copy of this vector, with euclidean length = 1.
    /// Not to be confused with Normalize() which normalizes in place.
    ChVector2<Real> GetNormalized() const;

    /// Impose a new length to the vector, keeping the direction unchanged.
    void SetLength(Real s);

    /// Apply a 2D rotation of given angle (positive counterclockwise).
    void Rotate(Real angle);

    /// Return the index of the largest component in absolute value.
    int GetMaxComponent() const;

    /// Return a unit vector orthogonal to this vector
    ChVector2<Real> GetOrthogonalVector() const;

    // STREAMING

    /// Method to allow serialization of transient data in archives.
    void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow de serialization of transient data from archives.
    void ArchiveIN(ChArchiveIn& marchive);

  private:
    Real data[2];

    /// Declaration of friend classes
    template <typename RealB>
    friend class ChVector2;
};

CH_CLASS_VERSION(ChVector2<double>, 0)

// -----------------------------------------------------------------------------
// STATIC VECTOR MATH OPERATIONS

template <class RealA, class RealB>
RealA Vdot(const ChVector2<RealA> va, const ChVector2<RealB> vb) {
    return (RealA)(va.x() * vb.x() + va.y() * vb.y());
}

template <class RealA>
void Vset(ChVector2<RealA>& v, RealA mx, RealA my) {
    v.x() = mx;
    v.y() = my;
}

template <class RealA, class RealB>
ChVector2<RealA> Vadd(const ChVector2<RealA>& va, const ChVector2<RealB>& vb) {
    return ChVector2<RealA>(va.x() + vb.x(), va.y() + vb.y());
}

template <class RealA, class RealB>
ChVector2<RealA> Vsub(const ChVector2<RealA>& va, const ChVector2<RealB>& vb) {
    return ChVector2<RealA>(va.x() - vb.x(), va.y() - vb.y());
}

template <class RealA, class RealB>
ChVector2<RealA> Vmul(const ChVector2<RealA>& va, RealB fact) {
    return ChVector2<RealA>(va.x() * (RealA)fact, va.y() * (RealA)fact);
}

template <class RealA>
RealA Vlength(const ChVector2<RealA>& va) {
    return (RealA)va.Length();
}

template <class RealA>
ChVector2<RealA> Vnorm(const ChVector2<RealA>& va) {
    return va.GetNormalized();
}

template <class RealA, class RealB>
bool Vequal(const ChVector2<RealA>& va, const ChVector2<RealB>& vb) {
    return (va == vb);
}

template <class RealA>
bool Vnotnull(const ChVector2<RealA>& va) {
    return (va.x() != 0 || va.y() != 0);
}

template <class RealA, class RealB>
ChVector2<RealA> Vrot(const ChVector2<RealA>& v, RealB angle) {
    ChVector2<RealA> tmp(v);
    tmp.Rotate((RealA)angle);
    return tmp;
}

// =============================================================================
// IMPLEMENTATION OF ChVector2<Real> methods
// =============================================================================

// -----------------------------------------------------------------------------
// Constructors

template <class Real>
inline ChVector2<Real>::ChVector2() {
    data[0] = 0;
    data[1] = 0;
}

template <class Real>
inline ChVector2<Real>::ChVector2(Real x, Real y) {
    data[0] = x;
    data[1] = y;
}

template <class Real>
inline ChVector2<Real>::ChVector2(Real a) {
    data[0] = a;
    data[1] = a;
}

template <class Real>
inline ChVector2<Real>::ChVector2(const ChVector2<Real>& other) {
    data[0] = other.data[0];
    data[1] = other.data[1];
}

template <class Real>
template <class RealB>
inline ChVector2<Real>::ChVector2(const ChVector2<RealB>& other) {
    data[0] = static_cast<Real>(other.data[0]);
    data[1] = static_cast<Real>(other.data[1]);
}

// -----------------------------------------------------------------------------
// Subscript operators

template <class Real>
inline Real& ChVector2<Real>::operator[](unsigned index) {
    assert(index < 2);
    return data[index];
}

template <class Real>
inline const Real& ChVector2<Real>::operator[](unsigned index) const {
    assert(index < 2);
    return data[index];
}

// -----------------------------------------------------------------------------
// Assignments

template <class Real>
inline ChVector2<Real>& ChVector2<Real>::operator=(const ChVector2<Real>& other) {
    if (&other == this)
        return *this;
    data[0] = other.data[0];
    data[1] = other.data[1];
    return *this;
}

template <class Real>
template <class RealB>
inline ChVector2<Real>& ChVector2<Real>::operator=(const ChVector2<RealB>& other) {
    data[0] = static_cast<Real>(other.data[0]);
    data[1] = static_cast<Real>(other.data[1]);
    return *this;
}

// -----------------------------------------------------------------------------
// Sign operators

template <class Real>
inline ChVector2<Real> ChVector2<Real>::operator+() const {
    return *this;
}

template <class Real>
inline ChVector2<Real> ChVector2<Real>::operator-() const {
    return ChVector2<Real>(-data[0], -data[1]);
}

// -----------------------------------------------------------------------------
// Arithmetic operations

template <class Real>
inline ChVector2<Real> ChVector2<Real>::operator+(const ChVector2<Real>& other) const {
    ChVector2<Real> v;

    v.data[0] = data[0] + other.data[0];
    v.data[1] = data[1] + other.data[1];

    return v;
}

template <class Real>
inline ChVector2<Real> ChVector2<Real>::operator-(const ChVector2<Real>& other) const {
    ChVector2<Real> v;

    v.data[0] = data[0] - other.data[0];
    v.data[1] = data[1] - other.data[1];

    return v;
}

template <class Real>
inline ChVector2<Real> ChVector2<Real>::operator*(const ChVector2<Real>& other) const {
    ChVector2<Real> v;

    v.data[0] = data[0] * other.data[0];
    v.data[1] = data[1] * other.data[1];

    return v;
}

template <class Real>
inline ChVector2<Real> ChVector2<Real>::operator/(const ChVector2<Real>& other) const {
    ChVector2<Real> v;

    v.data[0] = data[0] / other.data[0];
    v.data[1] = data[1] / other.data[1];

    return v;
}

template <class Real>
inline ChVector2<Real> ChVector2<Real>::operator*(Real s) const {
    ChVector2<Real> v;

    v.data[0] = data[0] * s;
    v.data[1] = data[1] * s;

    return v;
}

template <class Real>
inline ChVector2<Real> ChVector2<Real>::operator/(Real s) const {
    Real oos = 1 / s;
    ChVector2<Real> v;

    v.data[0] = data[0] * oos;
    v.data[1] = data[1] * oos;

    return v;
}

template <class Real>
inline ChVector2<Real>& ChVector2<Real>::operator+=(const ChVector2<Real>& other) {
    data[0] += other.data[0];
    data[1] += other.data[1];

    return *this;
}

template <class Real>
inline ChVector2<Real>& ChVector2<Real>::operator-=(const ChVector2<Real>& other) {
    data[0] -= other.data[0];
    data[1] -= other.data[1];

    return *this;
}

template <class Real>
inline ChVector2<Real>& ChVector2<Real>::operator*=(const ChVector2<Real>& other) {
    data[0] *= other.data[0];
    data[1] *= other.data[1];

    return *this;
}

template <class Real>
inline ChVector2<Real>& ChVector2<Real>::operator/=(const ChVector2<Real>& other) {
    data[0] /= other.data[0];
    data[1] /= other.data[1];

    return *this;
}

template <class Real>
inline ChVector2<Real>& ChVector2<Real>::operator*=(Real s) {
    data[0] *= s;
    data[1] *= s;

    return *this;
}

template <class Real>
inline ChVector2<Real>& ChVector2<Real>::operator/=(Real s) {
    Real oos = 1 / s;

    data[0] *= oos;
    data[1] *= oos;

    return *this;
}

// -----------------------------------------------------------------------------
// Vector operations

template <class Real>
inline Real ChVector2<Real>::operator^(const ChVector2<Real>& other) const {
    return this->Dot(other);
}

// -----------------------------------------------------------------------------
// Comparison operations

template <class Real>
inline bool ChVector2<Real>::operator<=(const ChVector2<Real>& other) const {
    return data[0] <= other.data[0] && data[1] <= other.data[1];
}

template <class Real>
inline bool ChVector2<Real>::operator>=(const ChVector2<Real>& other) const {
    return data[0] >= other.data[0] && data[1] >= other.data[1];
}

template <class Real>
inline bool ChVector2<Real>::operator<(const ChVector2<Real>& other) const {
    return data[0] < other.data[0] && data[1] < other.data[1];
}

template <class Real>
inline bool ChVector2<Real>::operator>(const ChVector2<Real>& other) const {
    return data[0] > other.data[0] && data[1] > other.data[1];
}

template <class Real>
inline bool ChVector2<Real>::operator==(const ChVector2<Real>& other) const {
    return other.data[0] == data[0] && other.data[1] == data[1];
}

template <class Real>
inline bool ChVector2<Real>::operator!=(const ChVector2<Real>& other) const {
    return !(*this == other);
}

// -----------------------------------------------------------------------------
// Functions

template <class Real>
inline void ChVector2<Real>::Set(const Real x, const Real y) {
    data[0] = x;
    data[1] = y;
}

template <class Real>
inline void ChVector2<Real>::Set(const ChVector2<Real>& v) {
    data[0] = v.data[0];
    data[1] = v.data[1];
}

template <class Real>
inline void ChVector2<Real>::Set(const Real s) {
    data[0] = s;
    data[1] = s;
}

/// Sets the vector as a null vector
template <class Real>
inline void ChVector2<Real>::SetNull() {
    data[0] = 0;
    data[1] = 0;
}

template <class Real>
inline bool ChVector2<Real>::IsNull() const {
    return data[0] == 0 && data[1] == 0;
}

template <class Real>
inline bool ChVector2<Real>::Equals(const ChVector2<Real>& other) const {
    return (other.data[0] == data[0]) && (other.data[1] == data[1]);
}

template <class Real>
inline bool ChVector2<Real>::Equals(const ChVector2<Real>& other, Real tol) const {
    return (fabs(other.data[0] - data[0]) < tol) && (fabs(other.data[1] - data[1]) < tol);
}

template <class Real>
inline void ChVector2<Real>::Add(const ChVector2<Real>& A, const ChVector2<Real>& B) {
    data[0] = A.data[0] + B.data[0];
    data[1] = A.data[1] + B.data[1];
}

template <class Real>
inline void ChVector2<Real>::Sub(const ChVector2<Real>& A, const ChVector2<Real>& B) {
    data[0] = A.data[0] - B.data[0];
    data[1] = A.data[1] - B.data[1];
}

template <class Real>
inline void ChVector2<Real>::Mul(const ChVector2<Real>& A, const Real s) {
    data[0] = A.data[0] * s;
    data[1] = A.data[1] * s;
}

template <class Real>
inline void ChVector2<Real>::Scale(const Real s) {
    data[0] *= s;
    data[1] *= s;
}

template <class Real>
inline Real ChVector2<Real>::Dot(const ChVector2<Real>& B) const {
    return (data[0] * B.data[0]) + (data[1] * B.data[1]);
}

template <class Real>
inline Real ChVector2<Real>::Length() const {
    return sqrt(Length2());
}

template <class Real>
inline Real ChVector2<Real>::Length2() const {
    return this->Dot(*this);
}

template <class Real>
inline Real ChVector2<Real>::LengthInf() const {
    return ChMax(fabs(data[0]), fabs(data[1]));
}

template <class Real>
inline bool ChVector2<Real>::Normalize() {
    Real length = this->Length();
    if (length < CH_NANOTOL) {
        data[0] = 1;
        data[1] = 0;
        return false;
    }
    this->Scale(1 / length);
    return true;
}

template <class Real>
inline ChVector2<Real> ChVector2<Real>::GetNormalized() const {
    ChVector2<Real> v(*this);
    v.Normalize();
    return v;
}

template <class Real>
inline void ChVector2<Real>::SetLength(Real s) {
    Normalize();
    Scale(s);
}

template <class Real>
inline void ChVector2<Real>::Rotate(Real angle) {
    Real ca = std::cos(angle);
    Real sa = std::sin(angle);
    Real tmp = data[0] * ca - data[1] * sa;
    data[1] = data[0] * sa + data[1] * ca;
    data[0] = tmp;
}

template <class Real>
inline int ChVector2<Real>::GetMaxComponent() const {
    return (fabs(data[0]) > fabs(data[1])) ? 0 : 1;
}

template <class Real>
inline ChVector2<Real> ChVector2<Real>::GetOrthogonalVector() const {
    ChVector2<Real> ortho(-data[1], data[0]);
    ortho.Normalize();
    return ortho;
}

// -----------------------------------------------------------------------------
// Streaming operations

template <class Real>
inline void ChVector2<Real>::ArchiveOUT(ChArchiveOut& marchive) {
    // suggested: use versioning
    marchive.VersionWrite<ChVector2<double>>();  // must use specialized template (any)
    // stream out all member data
    marchive << CHNVP(data[0]);
    marchive << CHNVP(data[1]);
}

template <class Real>
inline void ChVector2<Real>::ArchiveIN(ChArchiveIn& marchive) {
    // suggested: use versioning
    int version = marchive.VersionRead<ChVector2<double>>();  // must use specialized template (any)
    // stream in all member data
    marchive >> CHNVP(data[0]);
    marchive >> CHNVP(data[1]);
}

// -----------------------------------------------------------------------------
// Reversed operators

/// Operator for scaling the vector by a scalar value, as s*V
template <class Real>
ChVector2<Real> operator*(const Real s, const ChVector2<Real>& V) {
    return ChVector2<Real>(V.x() * s, V.y() * s);
}

}  // end namespace chrono

#endif
