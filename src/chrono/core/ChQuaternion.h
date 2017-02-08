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

#ifndef CHQUATERNION_H
#define CHQUATERNION_H

#include "chrono/core/ChVector.h"
#include "chrono/core/ChApiCE.h"

namespace chrono {

/// Definitions of various angle sets for conversions.
enum class AngleSet {
    ANGLE_AXIS,
    EULERO,
    CARDANO,
    HPB,
    RXYZ,
    RODRIGUEZ,
    QUATERNION,
};

///
/// QUATERNION:
///
/// Class defining quaternion objects, that is four-dimensional
/// numbers, also known as Eulero's parameters.
/// Quaternions are very handy when used to represent rotations in 3d.
///
///  The quaternion object comes either with the template "ChQuaternion<type>" mode,
/// either in the 'shortcut' flavour, that is "Quaternion", which assumes
/// the type of the four scalars is double precision, so it is faster to type.
///  For example, for a declaration, you can write "ChQuaternion<double> foo;",
/// as well as "Quaternion foo;" (less typing effort for the second..)
///
/// Further info at the @ref manual_ChQuaternion  manual page.

template <class Real = double>
class ChQuaternion {
  public:
    Real e0;
    Real e1;
    Real e2;
    Real e3;

    //
    // CONSTRUCTORS
    //

    /// Default constructor.
    /// Note, it is a null quaternion {0,0,0,0}, not a {1,0,0,0} unit quaternion.
    ChQuaternion() : e0(0), e1(0), e2(0), e3(0) {}

    /// Constructor from four scalars. The first is the real part, others are i,j,k imaginary parts
    ChQuaternion(const Real ne0, const Real ne1, const Real ne2, const Real ne3) : e0(ne0), e1(ne1), e2(ne2), e3(ne3) {}

    /// Constructor from real part, and vector with i,j,k imaginary part.
    ChQuaternion(const Real ns, const ChVector<Real>& nv) : e0(ns), e1(nv.x), e2(nv.y), e3(nv.z) {}

    /// Copy constructor
    ChQuaternion(const ChQuaternion<Real>& other) : e0(other.e0), e1(other.e1), e2(other.e2), e3(other.e3) {}

    /// Copy constructor between vectors float<->double etc
    template <class RealB>
    ChQuaternion(const ChQuaternion<RealB>& other)
        : e0((Real)other.e0), e1((Real)other.e1), e2((Real)other.e2), e3((Real)other.e3) {}

    //
    // OPERATORS OVERLOADING
    //
    // Note: c++ automatically creates temporary objects to store intermediate
    // results in long formulas, such as a= b*c*d, so the usage of operators
    // may give slower results than a wise (less readable however) usage of
    // Dot(), Cross() etc.. Also pay attention to C++ operator precedence rules!

    /// Assignment operator: copy from another quaternion
    ChQuaternion<Real>& operator=(const ChQuaternion<Real>& other) {
        if (&other == this)
            return *this;
        e0 = other.e0;
        e1 = other.e1;
        e2 = other.e2;
        e3 = other.e3;
        return *this;
    }

    /// Operator for sign change
    ChQuaternion<Real> operator-() const { return ChQuaternion<Real>(-e0, -e1, -e2, -e3); }

    /// Operator for making a conjugate quaternion (the original is not changed)
    /// A conjugate quaternion has the vectorial part with changed sign
    ChQuaternion<Real> operator!() const { return ChQuaternion<Real>(e0, -e1, -e2, -e3); }

    /// Operator for quaternion sum
    ChQuaternion<Real> operator+(const ChQuaternion<Real>& other) const {
        return ChQuaternion<Real>(e0 + other.e0, e1 + other.e1, e2 + other.e2, e3 + other.e3);
    }
    ChQuaternion<Real>& operator+=(const ChQuaternion<Real>& other) {
        e0 += other.e0;
        e1 += other.e1;
        e2 += other.e2;
        e3 += other.e3;
        return *this;
    }

    /// Operator for quaternion difference
    ChQuaternion<Real> operator-(const ChQuaternion<Real>& other) const {
        return ChQuaternion<Real>(e0 - other.e0, e1 - other.e1, e2 - other.e2, e3 - other.e3);
    }
    ChQuaternion<Real>& operator-=(const ChQuaternion<Real>& other) {
        e0 -= other.e0;
        e1 -= other.e1;
        e2 -= other.e2;
        e3 -= other.e3;
        return *this;
    }

    /// NOTE
    /// The following * and *= operators had a different behaviour prior to 13/9/2014,
    /// but we assume no one used * *= in that previous form (element-by-element
    /// product). Now * operator will be used for classical quaternion product,
    /// as the old % operator. This is be more congruent with the * operator for ChFrames etc.

    /// Operator for quaternion product: A*B means the typical quaternion product.
    /// Note: since unit quaternions can represent rotations, the product can represent a
    /// concatenation of rotations as:
    ///    frame_rotation_2to0 = frame_rotation_1to0 * frame_rotation_2to1
    /// Note: pay attention to operator low precedence (see C++ precedence rules!)
    /// Note: quaternion product is not commutative.
    ChQuaternion<Real> operator*(const ChQuaternion<Real>& other) const {
        ChQuaternion<Real> mr;
        mr.Cross(*this, other);
        return mr;
    }

    /// Operator for quaternion product and assignment:
    /// A*=B means A'=A*B, with typical quaternion product.
    /// Note: since unit quaternions can represent rotations, the product can represent a
    /// post-concatenation of a rotation in a kinematic chain.
    /// Note: quaternion product is not commutative.
    ChQuaternion<Real>& operator*=(const ChQuaternion<Real>& other) {
        this->Cross(*this, other);
        return *this;
    }

    /// Operator for 'specular' quaternion product: A>>B = B*A .
    /// Note: since unit quaternions can represent rotations, the product can represent a
    /// concatenation of rotations as:
    ///    frame_rotation_2to0 = frame_rotation_2to1 >> frame_rotation_1to0
    /// Note: pay attention to operator low precedence (see C++ precedence rules!)
    /// Note: quaternion product is not commutative.
    ChQuaternion<Real> operator>>(const ChQuaternion<Real>& other) const {
        ChQuaternion<Real> mr;
        mr.Cross(other, *this);
        return mr;
    }

    /// Operator for quaternion 'specular' product and assignment:
    /// A>>=B means A'=A>>B, or A'=B*A with typical quaternion product.
    /// Note: since unit quaternions can represent rotations, the product can represent a
    /// pre-concatenation of a rotation in a kinematic chain.
    /// Note: quaternion product is not commutative.
    ChQuaternion<Real>& operator>>=(const ChQuaternion<Real>& other) {
        this->Cross(other, *this);
        return *this;
    }

    // Operator for scaling the quaternion by a scalar value, as q*s
    ChQuaternion<Real> operator*(const Real v) const { return ChQuaternion<Real>(e0 * v, e1 * v, e2 * v, e3 * v); }
    ChQuaternion<Real>& operator*=(const Real v) {
        e0 *= v;
        e1 *= v;
        e2 *= v;
        e3 *= v;
        return *this;
    }

    /// Operator for element-wise division (note that
    /// this is NOT the quaternion division operation!)
    ChQuaternion<Real> operator/(const ChQuaternion<Real>& other) const {
        return ChQuaternion<Real>(e0 / other.e0, e1 / other.e1, e2 / other.e2, e3 / other.e3);
    }
    ChQuaternion<Real>& operator/=(const ChQuaternion<Real>& other) {
        e0 /= other.e0;
        e1 /= other.e1;
        e2 /= other.e2;
        e3 /= other.e3;
        return *this;
    }

    /// Operator for scaling the quaternion by inverse of a scalar value, as q/s
    ChQuaternion<Real> operator/(const Real v) const {
        Real i = 1 / v;
        return ChQuaternion<Real>(e0 * i, e1 * i, e2 * i, e3 * i);
    }
    ChQuaternion<Real>& operator/=(const Real v) {
        Real i = 1 / v;
        e0 *= i;
        e1 *= i;
        e2 *= i;
        e3 *= i;
        return *this;
    }

    /// Operator for quaternion product: A%B means the typical quaternion product AxB
    /// Note: DEPRECATED, use the * operator instead.
    ChQuaternion<Real> operator%(const ChQuaternion<Real>& other) const {
        ChQuaternion<Real> mr;
        mr.Cross(*this, other);
        return mr;
    }
    /// Note: DEPRECATED, use the *= operator instead.
    ChQuaternion<Real>& operator%=(const ChQuaternion<Real>& other) {
        this->Cross(*this, other);
        return *this;
    }

    /// Operator for dot product: A^B means the scalar dot-product A*B
    /// Note: pay attention to operator low precedence (see C++ precedence rules!)
    Real operator^(const ChQuaternion<Real>& other) const { return Dot(*this, other); }

    bool operator<=(const ChQuaternion<Real>& other) const {
        return e0 <= other.e0 && e1 <= other.e1 && e2 <= other.e2 && e3 <= other.e3;
    };
    bool operator>=(const ChQuaternion<Real>& other) const {
        return e0 >= other.e0 && e1 >= other.e1 && e2 >= other.e2 && e3 >= other.e3;
    };

    bool operator==(const ChQuaternion<Real>& other) const {
        return other.e0 == e0 && other.e1 == e1 && other.e2 == e2 && other.e3 == e3;
    }
    bool operator!=(const ChQuaternion<Real>& other) const {
        return other.e0 != e0 || other.e1 != e1 || other.e2 != e2 || other.e3 != e3;
    }

    //
    // FUNCTIONS
    //

    /// Sets the four values of the quaternion at once
    void Set(const Real ne0, const Real ne1, const Real ne2, const Real ne3) {
        e0 = ne0;
        e1 = ne1;
        e2 = ne2;
        e3 = ne3;
    }

    /// Sets the quaternion as a copy of another quaternion
    void Set(const ChQuaternion<Real>& n) {
        e0 = n.e0;
        e1 = n.e1;
        e2 = n.e2;
        e3 = n.e3;
    }

    /// Sets the quaternion with four components as a sample scalar
    void Set(const Real p) {
        e0 = p;
        e1 = p;
        e2 = p;
        e3 = p;
    }

    /// Sets the quaternion as a null quaternion
    void SetNull() { e0 = e1 = e2 = e3 = 0; }
    /// Sets the quaternion as a unit quaternion
    void SetUnit() {
        e0 = 1;
        e1 = e2 = e3 = 0;
    }

    /// Sets the scalar part only
    void SetScalar(const Real s) { e0 = s; }
    /// Sets the vectorial part only
    void SetVector(const ChVector<Real>& mv) {
        e1 = mv.x;
        e2 = mv.y;
        e3 = mv.z;
    }

    /// Gets the vectorial part only
    ChVector<Real> GetVector() { return ChVector<Real>(e1, e2, e3); }

    /// Returns true if quaternion is identical to other quaternion
    bool Equals(const ChQuaternion<Real>& other) const {
        return (other.e0 == e0) && (other.e1 == e1) && (other.e2 == e2) && (other.e3 == e3);
    }

    /// Returns true if quaternion equals another quaternion, within a tolerance 'tol'
    bool Equals(const ChQuaternion<Real>& other, Real tol) const {
        return (fabs(other.e0 - e0) < tol) && (fabs(other.e1 - e1) < tol) && (fabs(other.e2 - e2) < tol) &&
               (fabs(other.e3 - e3) < tol);
    }

    /// The quaternion becomes the sum of the two quaternions A and B:
    /// this=A+B
    void Add(const ChQuaternion<Real>& A, const ChQuaternion<Real>& B) {
        e0 = A.e0 + B.e0;
        e1 = A.e1 + B.e1;
        e2 = A.e2 + B.e2;
        e3 = A.e3 + B.e3;
    }

    /// The quaternion becomes the difference of the two quaternions A and B:
    /// this=A-B
    void Sub(const ChQuaternion<Real>& A, const ChQuaternion<Real>& B) {
        e0 = A.e0 - B.e0;
        e1 = A.e1 - B.e1;
        e2 = A.e2 - B.e2;
        e3 = A.e3 - B.e3;
    }

    /// The quaternion becomes the quaternion product of the two quaternions A and B:
    /// following the classic Hamilton rule:  this=AxB
    /// This is the true, typical quaternion product. It is NOT commutative.
    void Cross(const ChQuaternion<Real>& qa, const ChQuaternion<Real>& qb) {
        this->e0 = qa.e0 * qb.e0 - qa.e1 * qb.e1 - qa.e2 * qb.e2 - qa.e3 * qb.e3;
        this->e1 = qa.e0 * qb.e1 + qa.e1 * qb.e0 - qa.e3 * qb.e2 + qa.e2 * qb.e3;
        this->e2 = qa.e0 * qb.e2 + qa.e2 * qb.e0 + qa.e3 * qb.e1 - qa.e1 * qb.e3;
        this->e3 = qa.e0 * qb.e3 + qa.e3 * qb.e0 - qa.e2 * qb.e1 + qa.e1 * qb.e2;
    }

    /// The result is dot product of the two quaternion A and B:
    /// result=A*B. This kind of product is not used a lot, anyway...
    Real Dot(const ChQuaternion<Real>& A, const ChQuaternion<Real>& B) const {
        return (A.e0 * B.e0) + (A.e1 * B.e1) + (A.e2 * B.e2) + (A.e3 * B.e3);
    };

    /// The quaternion becomes the product of a quaternion A and a scalar v:
    /// this=A*v
    void Mul(const ChQuaternion<Real>& A, const Real v) {
        e0 = A.e0 * v;
        e1 = A.e1 * v;
        e2 = A.e2 * v;
        e3 = A.e3 * v;
    }

    /// The quaternion is multiplied by a scalar factor 's'
    /// this*=v
    void Scale(const Real v) {
        e0 *= v;
        e1 *= v;
        e2 *= v;
        e3 *= v;
    }

    /// Computes the euclidean norm of the quaternion,
    /// that is its length or magnitude
    Real Length() const { return sqrt(e0 * e0 + e1 * e1 + e2 * e2 + e3 * e3); }

    /// Computes the euclidean norm of the quaternion, squared
    /// (i.e. as Length(), but skipping the square root)
    Real Length2() const { return (e0 * e0 + e1 * e1 + e2 * e2 + e3 * e3); }

    /// Computes the infinite norm of the quaternion, that
    /// is the maximum absolute value of one of its elements
    Real LengthInf() const { return ChMax(ChMax(ChMax(fabs(e0), fabs(e1)), fabs(e2)), fabs(e3)); }

    /// Normalize this quaternion, so that its euclidean length is 1.
    /// Returns false if original quaternion had zero length (in such a case
    /// it will be defaulted as 1,0,0,0) otherwise returns true for success.
    bool Normalize() {
        Real mlength = this->Length();
        if (mlength < CH_NANOTOL) {
            e0 = 1;
            e1 = e2 = e3 = 0;
            return false;
        }
        this->Scale(1 / mlength);
        return true;
    }

    /// Return a normalized copy of this quaternion, with euclidean length =1.
    /// Not to be confused with Normalize(), that normalizes in place.
    ChQuaternion<Real> GetNormalized() const {
        ChQuaternion<Real> mret(*this);
        mret.Normalize();
        return mret;
    }

    /// The quaternion is set as the conjugate of A quaternion
    void Conjugate(const ChQuaternion<Real>& A) {
        e0 = A.e0;
        e1 = -A.e1;
        e2 = -A.e2;
        e3 = -A.e3;
    }

    /// The quaternion itself is conjugated (its vectorial part changes sign)
    void Conjugate() { Conjugate(*this); }

    /// Returns a conjugated version of this quaternion.
    ChQuaternion<Real> GetConjugate() const { return ChQuaternion<Real>(e0, -e1, -e2, -e3); }

    /// Returns the inverse of the quaternion
    ChQuaternion<Real> GetInverse() const { 
        ChQuaternion<Real> invq = this->GetConjugate();
        invq.Scale(1.0 / this->Length());
        return invq;
    }

    //
    // TRANSFORMATIONS
    //

    /// Rotates the vector A on the basis of this quaternion: res=p*[0,A]*p'
    /// (speed-optimized version). Endomorphism assumes p is already normalized.
    ChVector<Real> Rotate(const ChVector<Real>& A) const {
        Real e0e0 = e0 * e0;
        Real e1e1 = e1 * e1;
        Real e2e2 = e2 * e2;
        Real e3e3 = e3 * e3;
        Real e0e1 = e0 * e1;
        Real e0e2 = e0 * e2;
        Real e0e3 = e0 * e3;
        Real e1e2 = e1 * e2;
        Real e1e3 = e1 * e3;
        Real e2e3 = e2 * e3;
        return ChVector<Real>(((e0e0 + e1e1) * 2 - 1) * A.x + ((e1e2 - e0e3) * 2) * A.y + ((e1e3 + e0e2) * 2) * A.z,
                              ((e1e2 + e0e3) * 2) * A.x + ((e0e0 + e2e2) * 2 - 1) * A.y + ((e2e3 - e0e1) * 2) * A.z,
                              ((e1e3 - e0e2) * 2) * A.x + ((e2e3 + e0e1) * 2) * A.y + ((e0e0 + e3e3) * 2 - 1) * A.z);
    }

    /// Rotates the vector A on the basis of conjugate of this quaternion: res=p'*[0,A]*p
    /// (speed-optimized version).  Endomorphism assumes p is already normalized.
    ChVector<Real> RotateBack(const ChVector<Real>& A) const {
        Real e0e0 = e0 * e0;
        Real e1e1 = e1 * e1;
        Real e2e2 = e2 * e2;
        Real e3e3 = e3 * e3;
        Real e0e1 = -e0 * e1;
        Real e0e2 = -e0 * e2;
        Real e0e3 = -e0 * e3;
        Real e1e2 = e1 * e2;
        Real e1e3 = e1 * e3;
        Real e2e3 = e2 * e3;
        return ChVector<Real>(((e0e0 + e1e1) * 2 - 1) * A.x + ((e1e2 - e0e3) * 2) * A.y + ((e1e3 + e0e2) * 2) * A.z,
                              ((e1e2 + e0e3) * 2) * A.x + ((e0e0 + e2e2) * 2 - 1) * A.y + ((e2e3 - e0e1) * 2) * A.z,
                              ((e1e3 - e0e2) * 2) * A.x + ((e2e3 + e0e1) * 2) * A.y + ((e0e0 + e3e3) * 2 - 1) * A.z);
    }

    //
    // CONVERSIONS
    //

    /// Sets the quaternion from a rotation vector (ie. a 3D axis of rotation with length as agle of rotation)
    /// defined in absolute coords. 
    /// If you need distinct axis and angle, use rather Q_from_AngAxis()
    void Q_from_Rotv(const ChVector<Real>& angle_axis) {
        const Real theta_squared = angle_axis.x * angle_axis.x + angle_axis.y * angle_axis.y + angle_axis.z * angle_axis.z;
        // For non-zero rotation:
        if (theta_squared > Real(0.0)) {
            const Real theta = sqrt(theta_squared);
            const Real half_theta = theta * Real(0.5);
            const Real k = sin(half_theta) / theta;
            this->e0 = cos(half_theta);
            this->e1 = angle_axis.x * k;
            this->e2 = angle_axis.y * k;
            this->e3 = angle_axis.z * k;
        } else {
            // For almost zero rotation:
            const Real k(0.5);
            this->e0 = Real(1.0);
            this->e1 = angle_axis.x * k;
            this->e2 = angle_axis.y * k;
            this->e3 = angle_axis.z * k;
        }
    }

    /// Gets the rotation vector (ie. a 3D axis of rotation with length as agle of rotation)
    /// from a quaternion. 
    /// If you need distinct axis and angle, use rather Q_to_AngAxis()
    ChVector<Real> Q_to_Rotv() {
        ChVector<Real> angle_axis;
        const Real sin_squared = e1 * e1 + e2 * e2 + e3 * e3;
        // For non-zero rotation
        if (sin_squared > Real(0.0)) {
            const Real sin_theta = sqrt(sin_squared);
            const Real k = Real(2.0) * atan2(sin_theta, e0) / sin_theta;
            angle_axis.x = e1 * k;
            angle_axis.y = e2 * k;
            angle_axis.z = e3 * k;
        } else {
            // For almost zero rotation
            const Real k(2.0);
            angle_axis.x = e1 * k;
            angle_axis.y = e2 * k;
            angle_axis.z = e3 * k;
        }
        return angle_axis;
    }

    /// Sets the quaternion from an agle of rotation and an axis,
    /// defined in _absolute_ coords. The axis is supposed to be fixed, i.e.
    /// it is constant during rotation! NOTE, axismust be normalized!
    /// If you need directly the rotation vector=axis * angle, use rather Q_from_Rotv()
    void Q_from_AngAxis(const Real angle, const ChVector<Real>& axis) {
        Real halfang = ((Real)0.5 * angle);
        Real sinhalf = sin(halfang);
        this->e0 = cos(halfang);
        this->e1 = axis.x * sinhalf;
        this->e2 = axis.y * sinhalf;
        this->e3 = axis.z * sinhalf;
    }

	/// Sets the quaternion from an agle of rotation about X axis
    void Q_from_AngX(const Real angleX) { Q_from_AngAxis(angleX, ChVector<Real>(1, 0, 0)); }

    /// Sets the quaternion from an agle of rotation about Y axis
    void Q_from_AngY(const Real angleY) { Q_from_AngAxis(angleY, ChVector<Real>(0, 1, 0)); }

    /// Sets the quaternion from an agle of rotation about Z axis
    void Q_from_AngZ(const Real angleZ) { Q_from_AngAxis(angleZ, ChVector<Real>(0, 0, 1)); }

    /// Converts the quaternion to an agle of rotation and an axis,
    /// defined in _absolute_ coords. Resulting angle and axis must be passed as parameters
    /// If you need directly the rotation vector=axis * angle, use rather Q_to_Rotv()
    void Q_to_AngAxis(Real& a_angle, ChVector<Real>& a_axis) const {
        Real arg, invsine;
        if (fabs(e0) < 0.99999999) {
            arg = acos(e0);
            a_angle = 2 * arg;
            invsine = 1 / sin(arg);
            a_axis.x = invsine * e1;
            a_axis.y = invsine * e2;
            a_axis.z = invsine * e3;
            a_axis.Normalize();
        } else {
            a_axis.x = 1;
            a_axis.y = 0;
            a_axis.z = 0;
            a_angle = 0;
        }
    }

    /// Sets the quaternion from three angles (NASA angle set) heading,
    /// bank and attitude
    void Q_from_NasaAngles(const ChVector<Real>& mang) {
        Real c1 = cos(mang.z / 2);
        Real s1 = sin(mang.z / 2);
        Real c2 = cos(mang.x / 2);
        Real s2 = sin(mang.x / 2);
        Real c3 = cos(mang.y / 2);
        Real s3 = sin(mang.y / 2);

        Real c1c2 = c1 * c2;
        Real s1s2 = s1 * s2;

        e0 = c1c2 * c3 + s1s2 * s3;
        e1 = c1c2 * s3 - s1s2 * c3;
        e2 = c1 * s2 * c3 + s1 * c2 * s3;
        e3 = s1 * c2 * c3 - c1 * s2 * s3;
    }

    /// Converts the quaternion to three angles (NASA angle set)  heading,
    /// bank and attitude
    ChVector<Real> Q_to_NasaAngles() {
        ChVector<Real> mnasa;
        Real sqw = e0 * e0;
        Real sqx = e1 * e1;
        Real sqy = e2 * e2;
        Real sqz = e3 * e3;
        // heading
        mnasa.z = atan2(2 * (e1 * e2 + e3 * e0), (sqx - sqy - sqz + sqw));
        // bank
        mnasa.y = atan2(2 * (e2 * e3 + e1 * e0), (-sqx - sqy + sqz + sqw));
        // attitude
        mnasa.x = asin(-2 * (e1 * e3 - e2 * e0));
        return mnasa;
    }

    /// Sets the quaternion dq/dt. Inputs:  the vector of angular speed
    /// w specified in _absolute_ coords, and the rotation expressed
    /// as a quaternion q.
    void Qdt_from_Wabs(const ChVector<Real>& w, const ChQuaternion<Real>& q) {
        ChQuaternion<Real> qwo(0, w);
        this->Cross(qwo, q);
        this->Scale((Real)0.5);  // {q_dt} = 1/2 {0,w}*{q}
    }

    /// Sets the quaternion dq/dt. Inputs:  the vector of angular speed
    /// w specified in _relative coords, and the rotation expressed
    /// as a quaternion q.
    void Qdt_from_Wrel(const ChVector<Real>& w, const ChQuaternion<Real>& q) {
        ChQuaternion<Real> qwl(0, w);
        this->Cross(q, qwl);
        this->Scale((Real)0.5);  // {q_dt} = 1/2 {q}*{0,w_rel}
    }

    /// Computes the vector of angular speed 'w' specified in _absolute_ coords,
    /// from the quaternion dq/dt and the rotation expressed as a quaternion q.
    void Qdt_to_Wabs(ChVector<Real>& w, const ChQuaternion<Real>& q) {
        ChQuaternion<Real> qwo;
        qwo.Cross(*this, q.GetConjugate());
        w = qwo.GetVector();
        w.Scale(2);
    }

    /// Computes the vector of angular speed 'w' specified in _relative_ coords,
    /// from the quaternion dq/dt and the rotation expressed as a quaternion q.
    void Qdt_to_Wrel(ChVector<Real>& w, const ChQuaternion<Real>& q) {
        ChQuaternion<Real> qwl;
        qwl.Cross(q.GetConjugate(), *this);
        w = qwl.GetVector();
        w.Scale(2);
    }

    /// Sets the quaternion ddq/dtdt. Inputs: the vector of angular acceleration
    /// 'a' specified in _absolute_ coords, the rotation expressed
    /// as a quaternion q, the rotation speed as a quaternion 'q_dt'.
    void Qdtdt_from_Aabs(const ChVector<Real>& a, const ChQuaternion<Real>& q, const ChQuaternion<Real>& q_dt) {
        ChQuaternion<Real> qao(0, a);
        ChQuaternion<Real> qwo;
        ChQuaternion<Real> qtmpa;
        ChQuaternion<Real> qtmpb;
        qwo.Cross(q_dt, q.GetConjugate());
        qtmpb.Cross(qwo, q_dt);
        qtmpa.Cross(qao, q);
        qtmpa.Scale((Real)0.5);
        this->Add(qtmpa, qtmpb);
    }

    /// Sets the quaternion ddq/dtdt. Inputs: the vector of angular acceleration
    /// 'a' specified in _relative_ coords, the rotation expressed
    /// as a quaternion q, the rotation speed as a quaternion 'q_dt'.
    void Qdtdt_from_Arel(const ChVector<Real>& a, const ChQuaternion<Real>& q, const ChQuaternion<Real>& q_dt) {
        ChQuaternion<Real> qal(0, a);
        ChQuaternion<Real> qwl;
        ChQuaternion<Real> qtmpa;
        ChQuaternion<Real> qtmpb;
        qwl.Cross(q.GetConjugate(), q_dt);
        qtmpb.Cross(q_dt, qwl);
        qtmpa.Cross(q, qal);
        qtmpa.Scale((Real)0.5);
        this->Add(qtmpa, qtmpb);
    }

    /// Sets the quaternion dq/dt. Inputs:  the axis of rotation 'axis'
    /// (assuming it is already normalized and in _absolute_ coords),
    /// the angular speed 'angle_dt' (scalar value), and the
    /// rotation expressed as a quaternion 'q'.
    void Qdt_from_AngAxis(const ChQuaternion<Real>& q, const Real angle_dt, const ChVector<Real>& axis) {
        this->Qdt_from_Wabs(angle_dt * axis, q);
    }

    /// Sets the quaternion ddq/dtdt. Inputs: the axis of ang.acceleration 'axis'
    /// (assuming it is already normalized and in _absolute_ coords),
    /// the angular acceleration 'angle_dtdt' (scalar value), the
    /// rotation expressed as a quaternion 'quat' and th rotation speed 'q_dt'.
    void Qdtdt_from_AngAxis(const ChQuaternion<Real>& q,
                            const ChQuaternion<Real>& q_dt,
                            const Real angle_dtdt,
                            const ChVector<Real>& axis) {
        this->Qdtdt_from_Aabs(angle_dtdt * axis, q, q_dt);
    }

    /// Given the immaginary (vectorial) {e1 e2 e3} part of a quaternion,
    /// tries to set the entire quaternion q = {e0, e1, e2, e3}.
    /// Also for q_dt and q_dtdt
    /// Note: singularities may happen!
    void ImmQ_complete(const ChVector<Real>& qimm) {
        SetVector(qimm);
        e0 = sqrt(1 - e1 * e1 - e2 * e2 - e3 * e3);
    }
    void ImmQ_dt_complete(const ChQuaternion<Real>& mq, const ChVector<Real>& qimm_dt) {
        SetVector(qimm_dt);
        e0 = (-mq.e1 * e1 - mq.e2 * e2 - mq.e3 * e3) / mq.e0;
    }
    void ImmQ_dtdt_complete(const ChQuaternion<Real>& mq,
                            const ChQuaternion<Real>& mqdt,
                            const ChVector<Real>& qimm_dtdt) {
        SetVector(qimm_dtdt);
        e0 = (-mq.e1 * e1 - mq.e2 * e2 - mq.e3 * e3 - mqdt.e0 * mqdt.e0 - mqdt.e1 * mqdt.e1 - mqdt.e2 * mqdt.e2 -
              mqdt.e3 * mqdt.e3) /
             mq.e0;
    }

    /// Gets the X axis of a coordsystem, given the quaternion which represents
    /// the alignment of the coordsystem. Note that it is assumed that the
    /// quaternion is already normalized.
    ChVector<Real> GetXaxis() const {
        return ChVector<Real>((e0 * e0 + e1 * e1) * 2 - 1, (e1 * e2 + e0 * e3) * 2, (e1 * e3 - e0 * e2) * 2);
    }

    /// Gets the Y axis of a coordsystem, given the quaternion which represents
    /// the alignment of the coordsystem. Note that it is assumed that the
    /// quaternion is already normalized.
    ChVector<Real> GetYaxis() const {
        return ChVector<Real>((e1 * e2 - e0 * e3) * 2, (e0 * e0 + e2 * e2) * 2 - 1, (e2 * e3 + e0 * e1) * 2);
    }

    /// Gets the Z axis of a coordsystem, given the quaternion which represents
    /// the alignment of the coordsystem. Note that it is assumed that the
    /// quaternion is already normalized.
    ChVector<Real> GetZaxis() const {
        return ChVector<Real>((e1 * e3 + e0 * e2) * 2, (e2 * e3 - e0 * e1) * 2, (e0 * e0 + e3 * e3) * 2 - 1);
    }

    //
    // STREAMING
    //

    void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite(1);
        // stream out all member data
        marchive << CHNVP(e0);
        marchive << CHNVP(e1);
        marchive << CHNVP(e2);
        marchive << CHNVP(e3);
    }

    /// Method to allow de serialization of transient data from archives.
    void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // stream in all member data
        marchive >> CHNVP(e0);
        marchive >> CHNVP(e1);
        marchive >> CHNVP(e2);
        marchive >> CHNVP(e3);
    }
};

/// Shortcut for faster use of typical double-precision quaternion.
///  Instead of writing    "ChQuaternion<double> foo;"   you can write
///  the shorter version   "Quaternion foo;"
///
typedef ChQuaternion<double> Quaternion;

/// Shortcut for faster use of typical single-precision quaternion.
///
typedef ChQuaternion<float> QuaternionF;

//
// STATIC QUATERNION MATH OPERATIONS
//
//  These functions are here for people which prefer to use static
// functions instead of ChQuaternion class' member functions.
// NOTE: sometimes a wise adoption of the following functions may
// give faster results than using overloaded operators +/-/* in
// the quaternion class.
//  Also, these functions are here for backward compatibility with
// old parts of chrono code.
//  For best readability of our code, it is suggested not to use
// these functions - use the member functions or operators of
// the ChQuaternion class instead!

ChApi double Qlength(const ChQuaternion<double>& q);

ChApi ChQuaternion<double> Qadd(const ChQuaternion<double>& qa, const ChQuaternion<double>& qb);

ChApi ChQuaternion<double> Qsub(const ChQuaternion<double>& qa, const ChQuaternion<double>& qb);

ChApi ChQuaternion<double> Qscale(const ChQuaternion<double>& q, double fact);

/// Return the norm two of the quaternion. Euler's parameters have norm = 1
ChApi ChQuaternion<double> Qnorm(const ChQuaternion<double>& q);

/// Get the quaternion from an agle of rotation and an axis, defined in _abs_ coords.
/// The axis is supposed to be fixed, i.e. it is constant during rotation.
/// The 'axis' vector must be normalized.
ChApi ChQuaternion<double> Q_from_AngAxis(double angle, const ChVector<double>& axis);

/// Get the quaternion from a source vector and a destination vector which specifies
/// the rotation from one to the other.  The vectors do not need to be normalized.
ChApi ChQuaternion<double> Q_from_Vect_to_Vect(const ChVector<double>& fr_vect, const ChVector<double>& to_vect);

ChApi ChQuaternion<double> Q_from_NasaAngles(const ChVector<double>& RxRyRz);

ChApi ChVector<double> Q_to_NasaAngles(const ChQuaternion<double>& mq);

ChApi ChQuaternion<double> Q_from_AngZ(double angleZ);

ChApi ChQuaternion<double> Q_from_AngX(double angleX);

ChApi ChQuaternion<double> Q_from_AngY(double angleY);

ChApi void Q_to_AngAxis(const ChQuaternion<double>& quat, double& angle, ChVector<double>& axis);

/// Get the quaternion time derivative from the vector of angular speed, with w specified in _local_ coords.
ChApi ChQuaternion<double> Qdt_from_Wrel(const ChVector<double>& w, const Quaternion& q);

/// Get the quaternion time derivative from the vector of angular speed, with w specified in _absolute_ coords.
ChApi ChQuaternion<double> Qdt_from_Wabs(const ChVector<double>& w, const Quaternion& q);

/// Get the time derivative from a quaternion, a speed of rotation and an axis, defined in _abs_ coords.
ChApi ChQuaternion<double> Qdt_from_AngAxis(const ChQuaternion<double>& quat, double angle_dt, const ChVector<double>& axis);

/// Get the quaternion first derivative from the vector of angular acceleration with a specified in _absolute_ coords.
ChApi ChQuaternion<double> Qdtdt_from_Aabs(const ChVector<double>& a, const ChQuaternion<double>& q, const ChQuaternion<double>& q_dt);

///	Get the quaternion second derivative from the vector of angular acceleration with a specified in _relative_ coords.
ChApi ChQuaternion<double> Qdtdt_from_Arel(const ChVector<double>& a, const ChQuaternion<double>& q, const ChQuaternion<double>& q_dt);

/// Get the second time derivative from a quaternion, an angular acceleration and an axis, defined in _abs_ coords.
ChApi ChQuaternion<double> Qdtdt_from_AngAxis(double angle_dtdt, const ChVector<double>& axis, const ChQuaternion<double>& q, const ChQuaternion<double>& q_dt);

/// Return the conjugate of the quaternion [s,v1,v2,v3] is [s,-v1,-v2,-v3]
ChApi ChQuaternion<double> Qconjugate(const ChQuaternion<double>& q);

/// Return the product of two quaternions. It is non-commutative (like cross product in vectors).
ChApi ChQuaternion<double> Qcross(const ChQuaternion<double>& qa, const ChQuaternion<double>& qb);

/// Check if two quaternions are equal.
ChApi bool Qequal(const ChQuaternion<double>& qa, const ChQuaternion<double>& qb);

/// Check if quaternion is not null.
ChApi bool Qnotnull(const ChQuaternion<double>& qa);

/// Given the imaginary (vectorial) {e1 e2 e3} part of a quaternion,
/// find the entire quaternion q = {e0, e1, e2, e3}.
/// Note: singularities are possible.
ChApi ChQuaternion<double> ImmQ_complete(const ChVector<double>& qimm);

/// Given the imaginary (vectorial) {e1 e2 e3} part of a quaternion time derivative,
/// find the entire quaternion q = {e0, e1, e2, e3}.
/// Note: singularities are possible.
ChApi ChQuaternion<double> ImmQ_dt_complete(const ChQuaternion<double>& mq, const ChVector<double>& qimm_dt);

/// Given the imaginary (vectorial) {e1 e2 e3} part of a quaternion second time derivative,
/// find the entire quaternion q = {e0, e1, e2, e3}.
/// Note: singularities are possible.
ChApi ChQuaternion<double> ImmQ_dtdt_complete(const ChQuaternion<double>& mq, const ChQuaternion<double>& mqdt, const ChVector<double>& qimm_dtdt);

ChApi ChVector<double> VaxisXfromQuat(const ChQuaternion<double>& quat);

// Angle conversion utilities

ChApi ChVector<double> Quat_to_Angle(AngleSet angset, const ChQuaternion<double>& mquat);
ChApi ChVector<double> Angle_to_Angle(AngleSet setfrom, AngleSet setto, const ChVector<double>& mangles);
ChApi ChQuaternion<double> Angle_to_Quat(AngleSet angset, const ChVector<double>& mangles);
ChApi ChQuaternion<double> AngleDT_to_QuatDT(AngleSet angset, const ChVector<double>& mangles, const ChQuaternion<double>& q);
ChApi ChQuaternion<double> AngleDTDT_to_QuatDTDT(AngleSet angset, const ChVector<double>& mangles, const ChQuaternion<double>& q);

// CONSTANTS

ChApi extern const ChQuaternion<double> QNULL;
ChApi extern const ChQuaternion<double> QUNIT;

}  // end namespace chrono

#endif
