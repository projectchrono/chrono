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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHVECTOR_H
#define CHVECTOR_H

#include <cmath>
#include <iostream>
#include <iomanip>

#include "chrono/core/ChMathematics.h"
#include "chrono/core/ChClassFactory.h"
#include "chrono/serialization/ChArchive.h"

namespace chrono {

/// Definition of general purpose 3d vector variables, such as points in 3D.
/// This class implements the vectorial algebra in 3D (Gibbs products).
/// ChVector is templated by precision, with default 'double'.
///
/// Further info at the @ref mathematical_objects manual page.
template <class Real = double>
class ChVector {
  public:
    // CONSTRUCTORS

    ChVector();
    ChVector(Real x, Real y, Real z);
    ChVector(Real a);
    ChVector(const ChVector<Real>& other);

    /// Copy constructor with type change.
    template <class RealB>
    ChVector(const ChVector<RealB>& other);

    /// Access to components
    Real& x() { return data[0]; }
    Real& y() { return data[1]; }
    Real& z() { return data[2]; }
    const Real& x() const { return data[0]; }
    const Real& y() const { return data[1]; }
    const Real& z() const { return data[2]; }

    // SET FUNCTIONS

    /// Set the three values of the vector at once.
    void Set(Real x, Real y, Real z);

    /// Set the vector as a copy of another vector.
    void Set(const ChVector<Real>& v);

    /// Set all the vector components ts to the same scalar.
    void Set(Real s);

    /// Set the vector to the null vector.
    void SetNull();

    /// Return true if this vector is the null vector.
    bool IsNull() const;

    /// Return true if this vector is equal to another vector.
    bool Equals(const ChVector<Real>& other) const;

    /// Return true if this vector is equal to another vector, within a tolerance 'tol'.
    bool Equals(const ChVector<Real>& other, Real tol) const;

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
    ChVector<Real>& operator=(const ChVector<Real>& other);

    /// Assignment operator (copy from another vector) with type change.
    template <class RealB>
    ChVector<Real>& operator=(const ChVector<RealB>& other);

    /// Operators for sign change.
    ChVector<Real> operator+() const;
    ChVector<Real> operator-() const;

    /// Operator for vector sum.
    ChVector<Real> operator+(const ChVector<Real>& other) const;
    ChVector<Real>& operator+=(const ChVector<Real>& other);

    /// Operator for vector difference.
    ChVector<Real> operator-(const ChVector<Real>& other) const;
    ChVector<Real>& operator-=(const ChVector<Real>& other);

    /// Operator for element-wise multiplication.
    /// Note that this is neither dot product nor cross product.
    ChVector<Real> operator*(const ChVector<Real>& other) const;
    ChVector<Real>& operator*=(const ChVector<Real>& other);

    /// Operator for element-wise division.
    /// Note that 3D vector algebra is a skew field, non-divisional algebra,
    /// so this division operation is just an element-by element division.
    ChVector<Real> operator/(const ChVector<Real>& other) const;
    ChVector<Real>& operator/=(const ChVector<Real>& other);

    /// Operator for scaling the vector by a scalar value, as V*s
    ChVector<Real> operator*(Real s) const;
    ChVector<Real>& operator*=(Real s);

    /// Operator for scaling the vector by inverse of a scalar value, as v/s
    ChVector<Real> operator/(Real v) const;
    ChVector<Real>& operator/=(Real v);

    /// Operator for dot product: A^B means the scalar dot-product A*B
    /// Note: pay attention to operator low precedence (see C++ precedence rules!)
    Real operator^(const ChVector<Real>& other) const;

    /// Operator for cross product: A%B means the vector cross-product AxB
    /// Note: pay attention to operator low precedence (see C++ precedence rules!)
    ChVector<Real> operator%(const ChVector<Real>& other) const;
    ChVector<Real>& operator%=(const ChVector<Real>& other);

    /// Component-wise comparison operators
    bool operator<=(const ChVector<Real>& other) const;
    bool operator>=(const ChVector<Real>& other) const;
    bool operator<(const ChVector<Real>& other) const;
    bool operator>(const ChVector<Real>& other) const;
    bool operator==(const ChVector<Real>& other) const;
    bool operator!=(const ChVector<Real>& other) const;

    // FUNCTIONS

    /// Set this vector to the sum of A and B: this = A + B
    void Add(const ChVector<Real>& A, const ChVector<Real>& B);

    /// Set this vector to the difference of A and B: this = A - B
    void Sub(const ChVector<Real>& A, const ChVector<Real>& B);

    /// Set this vector to the product of a vector A and scalar s: this = A * s
    void Mul(const ChVector<Real>& A, Real s);

    /// Scale this vector by a scalar: this *= s
    void Scale(Real s);

    /// Set this vector to the cross product of A and B: this = A x B
    void Cross(const ChVector<Real>& A, const ChVector<Real>& B);

    /// Return the cross product with another vector: result = this x other
    ChVector<Real> Cross(const ChVector<Real> other) const;

    /// Return the dot product with another vector: result = this ^ B
    Real Dot(const ChVector<Real>& B) const;

    /// Normalize this vector in place, so that its euclidean length is 1.
    /// Return false if the original vector had zero length (in which case the vector
    /// is set to [1,0,0]) and return true otherwise.
    bool Normalize();

    /// Return a normalized copy of this vector, with euclidean length = 1.
    /// Not to be confused with Normalize() which normalizes in place.
    ChVector<Real> GetNormalized() const;

    /// Impose a new length to the vector, keeping the direction unchanged.
    void SetLength(Real s);

    /// Use the Gram-Schmidt orthonormalization to find the three
    /// orthogonal vectors of a coordinate system whose X axis is this vector.
    /// Vsingular (optional) sets the normal to the plane on which Dz must lie.
    void DirToDxDyDz(ChVector<Real>& Vx,
                     ChVector<Real>& Vy,
                     ChVector<Real>& Vz,
                     const ChVector<Real>& Vsingular = ChVector<Real>(0, 1, 0)) const;

    /// Return the index of the largest component in absolute value.
    int GetMaxComponent() const;

    /// Return a unit vector orthogonal to this vector
    ChVector<Real> GetOrthogonalVector() const;

    //
    // STREAMING
    //

    /// Method to allow serialization of transient data in archives.
    void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow de serialization of transient data from archives.
    void ArchiveIN(ChArchiveIn& marchive);

  private:
    Real data[3];

    /// Declaration of friend classes
    template <typename RealB>
    friend class ChVector;
};

CH_CLASS_VERSION(ChVector<double>, 0)

// -----------------------------------------------------------------------------

/// Shortcut for faster use of typical double-precision vectors.
/// <pre>
/// Instead of writing
///    ChVector<double> foo;
/// or
///    ChVector<> foo;
/// you can use the shorter version
///    Vector foo;
/// </pre>
typedef ChVector<double> Vector;

/// Shortcut for faster use of typical single-precision vectors.
/// <pre>
/// Instead of writing
///    ChVector<float> foo;
/// you can use the shorter version
///    Vector foo;
/// </pre>
typedef ChVector<float> VectorF;

// -----------------------------------------------------------------------------
// CONSTANTS

ChApi extern const ChVector<double> VNULL;
ChApi extern const ChVector<double> VECT_X;
ChApi extern const ChVector<double> VECT_Y;
ChApi extern const ChVector<double> VECT_Z;

// -----------------------------------------------------------------------------
// STATIC VECTOR MATH OPERATIONS
//
// These functions are here for people which prefer to use static
// functions instead of ChVector class' member functions.
// NOTE: sometimes a wise adoption of the following functions may
// give faster results rather than using overloaded operators +/-/* in
// the vector class.
// For best readability of our code, it is suggested not to use
// these functions - use the member functions or operators of
// the ChVector class instead.

template <class RealA, class RealB>
RealA Vdot(const ChVector<RealA>& va, const ChVector<RealB>& vb) {
    return (RealA)((va.x() * vb.x()) + (va.y() * vb.y()) + (va.z() * vb.z()));
}

template <class RealA>
void Vset(ChVector<RealA>& v, RealA mx, RealA my, RealA mz) {
    v.x() = mx;
    v.y() = my;
    v.z() = mz;
}

template <class RealA, class RealB>
ChVector<RealA> Vadd(const ChVector<RealA>& va, const ChVector<RealB>& vb) {
    ChVector<RealA> result;
    result.x() = va.x() + vb.x();
    result.y() = va.y() + vb.y();
    result.z() = va.z() + vb.z();
    return result;
}

template <class RealA, class RealB>
ChVector<RealA> Vsub(const ChVector<RealA>& va, const ChVector<RealB>& vb) {
    ChVector<RealA> result;
    result.x() = va.x() - vb.x();
    result.y() = va.y() - vb.y();
    result.z() = va.z() - vb.z();
    return result;
}

template <class RealA, class RealB>
ChVector<RealA> Vcross(const ChVector<RealA>& va, const ChVector<RealB>& vb) {
    ChVector<RealA> result;
    result.x() = (va.y() * vb.z()) - (va.z() * vb.y());
    result.y() = (va.z() * vb.x()) - (va.x() * vb.z());
    result.z() = (va.x() * vb.y()) - (va.y() * vb.x());
    return result;
}

template <class RealA, class RealB>
ChVector<RealA> Vmul(const ChVector<RealA>& va, RealB fact) {
    ChVector<RealA> result;
    result.x() = va.x() * (RealA)fact;
    result.y() = va.y() * (RealA)fact;
    result.z() = va.z() * (RealA)fact;
    return result;
}

template <class RealA>
RealA Vlength(const ChVector<RealA>& va) {
    return (RealA)va.Length();
}

template <class RealA>
ChVector<RealA> Vnorm(const ChVector<RealA>& va) {
    ChVector<RealA> result(va);
    result.Normalize();
    return result;
}

template <class RealA, class RealB>
bool Vequal(const ChVector<RealA>& va, const ChVector<RealB>& vb) {
    return (va == vb);
}

template <class RealA>
bool Vnotnull(const ChVector<RealA>& va) {
    return (va.x() != 0 || va.y() != 0 || va.z() != 0);
}

// Gets the zenith angle of a unit vector respect to YZ plane  ***OBSOLETE
template <class RealA>
double VangleYZplane(const ChVector<RealA>& va) {
    return asin(Vdot(va, ChVector<RealA>(1, 0, 0)));
}

// Gets the zenith angle of a unit vector respect to YZ plane  ***OBSOLETE
template <class RealA>
double VangleYZplaneNorm(const ChVector<RealA>& va) {
    return acos(Vdot(va, ChVector<RealA>(1, 0, 0)));
}

// Gets the angle of the projection on the YZ plane respect to
// the Y vector, as spinning about X.
template <class RealA>
double VangleRX(const ChVector<RealA>& va) {
    Vector vproj;
    vproj.x() = 0;
    vproj.y() = va.y();
    vproj.z() = va.z();
    vproj = Vnorm(vproj);
    if (vproj.x() == 1)
        return 0;
    return acos(vproj.y());
}

// The reverse of the two previous functions, gets the vector
// given the angle above the normal to YZ plane and the angle
// of rotation on X
template <class RealA>
ChVector<RealA> VfromPolar(double norm_angle, double pol_angle) {
    ChVector<> res;
    double projlen;
    res.x() = cos(norm_angle);  // 1) rot 'norm.angle'about z
    res.y() = sin(norm_angle);
    res.z() = 0;
    projlen = res.y();
    res.y() = projlen * cos(pol_angle);
    res.z() = projlen * sin(pol_angle);
    return res;
}

// From non-normalized x direction, to versors DxDyDz.
// Vsingular sets the normal to the plane on which Dz must lie.
template <class RealA>
void XdirToDxDyDz(const ChVector<RealA>& Vxdir,
                  const ChVector<RealA>& Vsingular,
                  ChVector<RealA>& Vx,
                  ChVector<RealA>& Vy,
                  ChVector<RealA>& Vz) {
    ChVector<RealA> mVnull(0, 0, 0);
    double zlen;

    if (Vequal(Vxdir, mVnull))
        Vx = ChVector<RealA>(1, 0, 0);
    else
        Vx = Vnorm(Vxdir);

    Vz = Vcross(Vx, Vsingular);
    zlen = Vlength(Vz);

    // If close to singularity, change reference vector
    if (zlen < 0.0001) {
        ChVector<> mVsingular;
        if (fabs(Vsingular.z()) < 0.9)
            mVsingular = ChVector<RealA>(0, 0, 1);
        if (fabs(Vsingular.y()) < 0.9)
            mVsingular = ChVector<RealA>(0, 1, 0);
        if (fabs(Vsingular.x()) < 0.9)
            mVsingular = ChVector<RealA>(1, 0, 0);
        Vz = Vcross(Vx, mVsingular);
        zlen = Vlength(Vz);  // now should be nonzero length.
    }

    // normalize Vz
    Vz = Vmul(Vz, 1.0 / zlen);
    // compute Vy
    Vy = Vcross(Vz, Vx);
}

// =============================================================================
// IMPLEMENTATION OF ChVector<Real> methods
// =============================================================================

// -----------------------------------------------------------------------------
// Constructors

template <class Real>
inline ChVector<Real>::ChVector() {
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
}

template <class Real>
inline ChVector<Real>::ChVector(Real a) {
    data[0] = a;
    data[1] = a;
    data[2] = a;
}

template <class Real>
inline ChVector<Real>::ChVector(Real x, Real y, Real z) {
    data[0] = x;
    data[1] = y;
    data[2] = z;
}

template <class Real>
inline ChVector<Real>::ChVector(const ChVector<Real>& other) {
    data[0] = other.data[0];
    data[1] = other.data[1];
    data[2] = other.data[2];
}

template <class Real>
template <class RealB>
inline ChVector<Real>::ChVector(const ChVector<RealB>& other) {
    data[0] = static_cast<Real>(other.data[0]);
    data[1] = static_cast<Real>(other.data[1]);
    data[2] = static_cast<Real>(other.data[2]);
}

// -----------------------------------------------------------------------------
// Subscript operators

template <class Real>
inline Real& ChVector<Real>::operator[](unsigned index) {
    assert(index < 3);
    return data[index];
}

template <class Real>
inline const Real& ChVector<Real>::operator[](unsigned index) const {
    assert(index < 3);
    return data[index];
}

// -----------------------------------------------------------------------------
// Assignments

template <class Real>
inline ChVector<Real>& ChVector<Real>::operator=(const ChVector<Real>& other) {
    if (&other == this)
        return *this;
    data[0] = other.data[0];
    data[1] = other.data[1];
    data[2] = other.data[2];
    return *this;
}

template <class Real>
template <class RealB>
inline ChVector<Real>& ChVector<Real>::operator=(const ChVector<RealB>& other) {
    data[0] = static_cast<Real>(other.data[0]);
    data[1] = static_cast<Real>(other.data[1]);
    data[2] = static_cast<Real>(other.data[2]);
    return *this;
}

// -----------------------------------------------------------------------------
// Sign operators

template <class Real>
inline ChVector<Real> ChVector<Real>::operator+() const {
    return *this;
}

template <class Real>
inline ChVector<Real> ChVector<Real>::operator-() const {
    return ChVector<Real>(-data[0], -data[1], -data[2]);
}

// -----------------------------------------------------------------------------
// Arithmetic operations

template <class Real>
inline ChVector<Real> ChVector<Real>::operator+(const ChVector<Real>& other) const {
    ChVector<Real> v;

    v.data[0] = data[0] + other.data[0];
    v.data[1] = data[1] + other.data[1];
    v.data[2] = data[2] + other.data[2];

    return v;
}

template <class Real>
inline ChVector<Real> ChVector<Real>::operator-(const ChVector<Real>& other) const {
    ChVector<Real> v;

    v.data[0] = data[0] - other.data[0];
    v.data[1] = data[1] - other.data[1];
    v.data[2] = data[2] - other.data[2];

    return v;
}

template <class Real>
inline ChVector<Real> ChVector<Real>::operator*(const ChVector<Real>& other) const {
    ChVector<Real> v;

    v.data[0] = data[0] * other.data[0];
    v.data[1] = data[1] * other.data[1];
    v.data[2] = data[2] * other.data[2];

    return v;
}

template <class Real>
inline ChVector<Real> ChVector<Real>::operator/(const ChVector<Real>& other) const {
    ChVector<Real> v;

    v.data[0] = data[0] / other.data[0];
    v.data[1] = data[1] / other.data[1];
    v.data[2] = data[2] / other.data[2];

    return v;
}

template <class Real>
inline ChVector<Real> ChVector<Real>::operator*(Real s) const {
    ChVector<Real> v;

    v.data[0] = data[0] * s;
    v.data[1] = data[1] * s;
    v.data[2] = data[2] * s;

    return v;
}

template <class Real>
inline ChVector<Real> ChVector<Real>::operator/(Real s) const {
    Real oos = 1 / s;
    ChVector<Real> v;

    v.data[0] = data[0] * oos;
    v.data[1] = data[1] * oos;
    v.data[2] = data[2] * oos;

    return v;
}

template <class Real>
inline ChVector<Real>& ChVector<Real>::operator+=(const ChVector<Real>& other) {
    data[0] += other.data[0];
    data[1] += other.data[1];
    data[2] += other.data[2];

    return *this;
}

template <class Real>
inline ChVector<Real>& ChVector<Real>::operator-=(const ChVector<Real>& other) {
    data[0] -= other.data[0];
    data[1] -= other.data[1];
    data[2] -= other.data[2];

    return *this;
}

template <class Real>
inline ChVector<Real>& ChVector<Real>::operator*=(const ChVector<Real>& other) {
    data[0] *= other.data[0];
    data[1] *= other.data[1];
    data[2] *= other.data[2];

    return *this;
}

template <class Real>
inline ChVector<Real>& ChVector<Real>::operator/=(const ChVector<Real>& other) {
    data[0] /= other.data[0];
    data[1] /= other.data[1];
    data[2] /= other.data[2];

    return *this;
}

template <class Real>
inline ChVector<Real>& ChVector<Real>::operator*=(Real s) {
    data[0] *= s;
    data[1] *= s;
    data[2] *= s;

    return *this;
}

template <class Real>
inline ChVector<Real>& ChVector<Real>::operator/=(Real s) {
    Real oos = 1 / s;

    data[0] *= oos;
    data[1] *= oos;
    data[2] *= oos;

    return *this;
}

// -----------------------------------------------------------------------------
// Vector operations

template <class Real>
inline Real ChVector<Real>::operator^(const ChVector<Real>& other) const {
    return this->Dot(other);
}

template <class Real>
ChVector<Real> ChVector<Real>::operator%(const ChVector<Real>& other) const {
    ChVector<Real> v;
    v.Cross(*this, other);
    return v;
}

template <class Real>
inline ChVector<Real>& ChVector<Real>::operator%=(const ChVector<Real>& other) {
    this->Cross(*this, other);
    return *this;
}

// -----------------------------------------------------------------------------
// Comparison operations

template <class Real>
inline bool ChVector<Real>::operator<=(const ChVector<Real>& other) const {
    return data[0] <= other.data[0] && data[1] <= other.data[1] && data[2] <= other.data[2];
}

template <class Real>
inline bool ChVector<Real>::operator>=(const ChVector<Real>& other) const {
    return data[0] >= other.data[0] && data[1] >= other.data[1] && data[2] >= other.data[2];
}

template <class Real>
inline bool ChVector<Real>::operator<(const ChVector<Real>& other) const {
    return data[0] < other.data[0] && data[1] < other.data[1] && data[2] < other.data[2];
}

template <class Real>
inline bool ChVector<Real>::operator>(const ChVector<Real>& other) const {
    return data[0] > other.data[0] && data[1] > other.data[1] && data[2] > other.data[2];
}

template <class Real>
inline bool ChVector<Real>::operator==(const ChVector<Real>& other) const {
    return other.data[0] == data[0] && other.data[1] == data[1] && other.data[2] == data[2];
}

template <class Real>
inline bool ChVector<Real>::operator!=(const ChVector<Real>& other) const {
    return !(*this == other);
}

// -----------------------------------------------------------------------------
// Functions

template <class Real>
inline void ChVector<Real>::Set(Real x, Real y, Real z) {
    data[0] = x;
    data[1] = y;
    data[2] = z;
}

template <class Real>
inline void ChVector<Real>::Set(const ChVector<Real>& v) {
    data[0] = v.data[0];
    data[1] = v.data[1];
    data[2] = v.data[2];
}

template <class Real>
inline void ChVector<Real>::Set(Real s) {
    data[0] = s;
    data[1] = s;
    data[2] = s;
}

/// Sets the vector as a null vector
template <class Real>
inline void ChVector<Real>::SetNull() {
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
}

template <class Real>
inline bool ChVector<Real>::IsNull() const {
    return data[0] == 0 && data[1] == 0 && data[2] == 0;
}

template <class Real>
inline bool ChVector<Real>::Equals(const ChVector<Real>& other) const {
    return (other.data[0] == data[0]) && (other.data[1] == data[1]) && (other.data[2] == data[2]);
}

template <class Real>
inline bool ChVector<Real>::Equals(const ChVector<Real>& other, Real tol) const {
    return (fabs(other.data[0] - data[0]) < tol) && (fabs(other.data[1] - data[1]) < tol) &&
           (fabs(other.data[2] - data[2]) < tol);
}

template <class Real>
inline void ChVector<Real>::Add(const ChVector<Real>& A, const ChVector<Real>& B) {
    data[0] = A.data[0] + B.data[0];
    data[1] = A.data[1] + B.data[1];
    data[2] = A.data[2] + B.data[2];
}

template <class Real>
inline void ChVector<Real>::Sub(const ChVector<Real>& A, const ChVector<Real>& B) {
    data[0] = A.data[0] - B.data[0];
    data[1] = A.data[1] - B.data[1];
    data[2] = A.data[2] - B.data[2];
}

template <class Real>
inline void ChVector<Real>::Mul(const ChVector<Real>& A, Real s) {
    data[0] = A.data[0] * s;
    data[1] = A.data[1] * s;
    data[2] = A.data[2] * s;
}

template <class Real>
inline void ChVector<Real>::Scale(Real s) {
    data[0] *= s;
    data[1] *= s;
    data[2] *= s;
}

template <class Real>
inline void ChVector<Real>::Cross(const ChVector<Real>& A, const ChVector<Real>& B) {
    data[0] = (A.data[1] * B.data[2]) - (A.data[2] * B.data[1]);
    data[1] = (A.data[2] * B.data[0]) - (A.data[0] * B.data[2]);
    data[2] = (A.data[0] * B.data[1]) - (A.data[1] * B.data[0]);
}

template <class Real>
inline ChVector<Real> ChVector<Real>::Cross(const ChVector<Real> other) const {
    ChVector<Real> v;
    v.Cross(*this, other);
    return v;
}

template <class Real>
inline Real ChVector<Real>::Dot(const ChVector<Real>& B) const {
    return (data[0] * B.data[0]) + (data[1] * B.data[1]) + (data[2] * B.data[2]);
}

template <class Real>
inline Real ChVector<Real>::Length() const {
    return sqrt(Length2());
}

template <class Real>
inline Real ChVector<Real>::Length2() const {
    return this->Dot(*this);
}

template <class Real>
inline Real ChVector<Real>::LengthInf() const {
    return ChMax(ChMax(fabs(data[0]), fabs(data[1])), fabs(data[2]));
}

template <class Real>
inline bool ChVector<Real>::Normalize() {
    Real length = this->Length();
    if (length < CH_NANOTOL) {
        data[0] = 1;
        data[1] = 0;
        data[2] = 0;
        return false;
    }
    this->Scale(1 / length);
    return true;
}

template <class Real>
inline ChVector<Real> ChVector<Real>::GetNormalized() const {
    ChVector<Real> v(*this);
    v.Normalize();
    return v;
}

template <class Real>
inline void ChVector<Real>::SetLength(Real s) {
    Normalize();
    Scale(s);
}

template <class Real>
inline void ChVector<Real>::DirToDxDyDz(ChVector<Real>& Vx,
                                        ChVector<Real>& Vy,
                                        ChVector<Real>& Vz,
                                        const ChVector<Real>& Vsingular) const {
    // set Vx.
    if (this->IsNull())
        Vx = ChVector<Real>(1, 0, 0);
    else
        Vx = this->GetNormalized();

    Vz.Cross(Vx, Vsingular);
    Real zlen = Vz.Length();

    // if near singularity, change the singularity reference vector.
    if (zlen < 0.0001) {
        ChVector<Real> mVsingular;

        if (fabs(Vsingular.data[0]) < 0.9)
            mVsingular = ChVector<Real>(1, 0, 0);
        else if (fabs(Vsingular.data[1]) < 0.9)
            mVsingular = ChVector<Real>(0, 1, 0);
        else if (fabs(Vsingular.data[2]) < 0.9)
            mVsingular = ChVector<Real>(0, 0, 1);

        Vz.Cross(Vx, mVsingular);
        zlen = Vz.Length();  // now should be nonzero length.
    }

    // normalize Vz.
    Vz.Scale(1.0 / zlen);

    // compute Vy.
    Vy.Cross(Vz, Vx);
}

template <class Real>
inline int ChVector<Real>::GetMaxComponent() const {
    int idx = 0;
    Real max = fabs(data[0]);
    if (fabs(data[1]) > max) {
        idx = 1;
        max = data[1];
    }
    if (fabs(data[2]) > max) {
        idx = 2;
        max = data[2];
    }
    return idx;
}

template <class Real>
inline ChVector<Real> ChVector<Real>::GetOrthogonalVector() const {
    int idx1 = this->GetMaxComponent();
    int idx2 = (idx1 + 1) % 3;  // cycle to the next component
    int idx3 = (idx2 + 1) % 3;  // cycle to the next component

    // Construct v2 by rotating in the plane containing the maximum component
    ChVector<Real> v2(-data[idx2], data[idx1], data[idx3]);

    // Construct the normal vector
    ChVector<Real> ortho = Cross(v2);
    ortho.Normalize();
    return ortho;
}

// -----------------------------------------------------------------------------
// Streaming operations

template <class Real>
inline void ChVector<Real>::ArchiveOUT(ChArchiveOut& marchive) {
    // suggested: use versioning
    marchive.VersionWrite<ChVector<double>>();  // must use specialized template (any)
    // stream out all member data
    marchive << CHNVP(data[0]);
    marchive << CHNVP(data[1]);
    marchive << CHNVP(data[2]);
}

template <class Real>
inline void ChVector<Real>::ArchiveIN(ChArchiveIn& marchive) {
    // suggested: use versioning
    int version = marchive.VersionRead<ChVector<double>>();  // must use specialized template (any)
    // stream in all member data
    marchive >> CHNVP(data[0]);
    marchive >> CHNVP(data[1]);
    marchive >> CHNVP(data[2]);
}

// -----------------------------------------------------------------------------
// Reversed operators

/// Operator for scaling the vector by a scalar value, as s*V
template <class Real>
ChVector<Real> operator*(Real s, const ChVector<Real>& V) {
    return ChVector<Real>(V.x() * s, V.y() * s, V.z() * s);
}

}  // end namespace chrono

#endif
