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

/// Class defining quaternion objects, that is four-dimensional numbers, also known as Euler parameters.
/// Quaternions are very useful when used to represent rotations in 3d.
///
/// Further info at the @ref manual_ChQuaternion  manual page.
template <class Real = double>
class ChQuaternion {
  public:
    /// Default constructor.
    /// Note that this constructs a null quaternion {0,0,0,0}, not a {1,0,0,0} unit quaternion.
    ChQuaternion();

    /// Constructor from four scalars. The first is the real part, others are i,j,k imaginary parts
    ChQuaternion(Real e0, Real e1, Real e2, Real e3);

    /// Constructor from real part, and vector with i,j,k imaginary part.
    ChQuaternion(Real s, const ChVector<Real>& v);

    /// Copy constructor
    ChQuaternion(const ChQuaternion<Real>& other);

    /// Copy constructor with type change.
    template <class RealB>
    ChQuaternion(const ChQuaternion<RealB>& other);

    /// Access to components
    Real& e0() { return data[0]; }
    Real& e1() { return data[1]; }
    Real& e2() { return data[2]; }
    Real& e3() { return data[3]; }
    const Real& e0() const { return data[0]; }
    const Real& e1() const { return data[1]; }
    const Real& e2() const { return data[2]; }
    const Real& e3() const { return data[3]; }

    // SET & GET FUNCTIONS

    /// Sets the four values of the quaternion at once
    void Set(Real e0, Real e1, Real e2, Real e3);

    /// Sets the quaternion as a copy of another quaternion
    void Set(const ChQuaternion<Real>& q);

    /// Sets the quaternion with four components as a sample scalar
    void Set(Real s);

    /// Sets the quaternion as a null quaternion
    void SetNull();

    /// Sets the quaternion as a unit quaternion
    void SetUnit();

    /// Sets the scalar part only
    void SetScalar(Real s);

    /// Sets the vectorial part only
    void SetVector(const ChVector<Real>& v);

    /// Return true if quaternion is identical to other quaternion
    bool Equals(const ChQuaternion<Real>& other) const;

    /// Return true if quaternion equals another quaternion, within a tolerance 'tol'
    bool Equals(const ChQuaternion<Real>& other, Real tol) const;

    /// Gets the vectorial part only
    ChVector<Real> GetVector() const;

    /// Get the X axis of a coordsystem, given the quaternion which represents
    /// the alignment of the coordsystem. Note that it is assumed that the
    /// quaternion is already normalized.
    ChVector<Real> GetXaxis() const;

    /// Get the Y axis of a coordsystem, given the quaternion which represents
    /// the alignment of the coordsystem. Note that it is assumed that the
    /// quaternion is already normalized.
    ChVector<Real> GetYaxis() const;

    /// Get the Z axis of a coordsystem, given the quaternion which represents
    /// the alignment of the coordsystem. Note that it is assumed that the
    /// quaternion is already normalized.
    ChVector<Real> GetZaxis() const;

    // QUATERNION NORMS

    /// Compute the euclidean norm of the quaternion, that is its length or magnitude.
    Real Length() const;

    /// Compute the squared euclidean norm of the quaternion.
    Real Length2() const;

    /// Compute the infinity norm of the quaternion, that is the maximum absolute value of one of its elements.
    Real LengthInf() const;

    // OPERATORS OVERLOADING

    /// Subscript operator.
    Real& operator[](unsigned index);
    const Real& operator[](unsigned index) const;

    /// Assignment operator: copy from another quaternion.
    ChQuaternion<Real>& operator=(const ChQuaternion<Real>& other);

    /// Operator for sign change.
    ChQuaternion<Real> operator+() const;
    ChQuaternion<Real> operator-() const;

    /// Operator for making a conjugate quaternion (the original is not changed).
    /// A conjugate quaternion has the vectorial part with changed sign.
    ChQuaternion<Real> operator!() const;

    /// Operator for quaternion sum.
    ChQuaternion<Real> operator+(const ChQuaternion<Real>& other) const;
    ChQuaternion<Real>& operator+=(const ChQuaternion<Real>& other);

    /// Operator for quaternion difference.
    ChQuaternion<Real> operator-(const ChQuaternion<Real>& other) const;
    ChQuaternion<Real>& operator-=(const ChQuaternion<Real>& other);

    // NOTE
    // The following * and *= operators had a different behaviour prior to 13/9/2014,
    // but we assume no one used * *= in that previous form (element-by-element product).
    // Now * operator will be used for classical quaternion product, as the old % operator.
    // This is to be more consistent with the * operator for ChFrames etc.

    /// Operator for quaternion product: A*B means the typical quaternion product.
    /// <pre>
    /// Notes:
    /// - since unit quaternions can represent rotations, the product can represent a
    ///   concatenation of rotations as:
    ///        frame_rotation_2to0 = frame_rotation_1to0 * frame_rotation_2to1
    /// - pay attention to operator low precedence (see C++ precedence rules!)
    /// - quaternion product is not commutative.
    /// </pre>
    ChQuaternion<Real> operator*(const ChQuaternion<Real>& other) const;

    /// Operator for quaternion product and assignment:
    /// A*=B means A'=A*B, with typical quaternion product.
    /// <pre>
    /// Notes:
    /// - since unit quaternions can represent rotations, the product can represent a
    ///   post-concatenation of a rotation in a kinematic chain.
    /// - quaternion product is not commutative.
    /// </pre>
    ChQuaternion<Real>& operator*=(const ChQuaternion<Real>& other);

    /// Operator for 'specular' quaternion product: A>>B = B*A.
    /// <pre>
    /// Notes:
    /// - since unit quaternions can represent rotations, the product can represent a
    ///   concatenation of rotations as:
    ///       frame_rotation_2to0 = frame_rotation_2to1 >> frame_rotation_1to0
    /// - pay attention to operator low precedence (see C++ precedence rules!)
    /// - quaternion product is not commutative.
    /// </pre>
    ChQuaternion<Real> operator>>(const ChQuaternion<Real>& other) const;

    /// Operator for quaternion 'specular' product and assignment:
    /// A>>=B means A'=A>>B, or A'=B*A with typical quaternion product.
    /// <pre>
    /// Notes:
    /// - since unit quaternions can represent rotations, the product can represent a
    ///   pre-concatenation of a rotation in a kinematic chain.
    /// - quaternion product is not commutative.
    /// </pre>
    ChQuaternion<Real>& operator>>=(const ChQuaternion<Real>& other);

    // Operator for scaling the quaternion by a scalar value, as q*s.
    ChQuaternion<Real> operator*(Real s) const;
    ChQuaternion<Real>& operator*=(Real s);

    /// Operator for element-wise division.
    /// Note that this is NOT the quaternion division operation.
    ChQuaternion<Real> operator/(const ChQuaternion<Real>& other) const;
    ChQuaternion<Real>& operator/=(const ChQuaternion<Real>& other);

    /// Operator for scaling the quaternion by inverse of a scalar value, as q/s.
    ChQuaternion<Real> operator/(Real s) const;
    ChQuaternion<Real>& operator/=(Real s);

    /// Operator for quaternion product: A%B means the typical quaternion product AxB.
    /// Note: DEPRECATED, use the * operator instead.
    ChQuaternion<Real> operator%(const ChQuaternion<Real>& other) const;
    ChQuaternion<Real>& operator%=(const ChQuaternion<Real>& other);

    /// Operator for dot product: A^B means the scalar dot-product A*B.
    /// Note: pay attention to operator low precedence (see C++ precedence rules!)
    Real operator^(const ChQuaternion<Real>& other) const;

    /// Component-wise comparison operators.
    bool operator<=(const ChQuaternion<Real>& other) const;
    bool operator>=(const ChQuaternion<Real>& other) const;
    bool operator<(const ChQuaternion<Real>& other) const;
    bool operator>(const ChQuaternion<Real>& other) const;
    bool operator==(const ChQuaternion<Real>& other) const;
    bool operator!=(const ChQuaternion<Real>& other) const;

    // FUNCTIONS

    /// Set this quaternion to the sum of A and B: this = A + B.
    void Add(const ChQuaternion<Real>& A, const ChQuaternion<Real>& B);

    /// Set this quaternion to the difference of A and B: this = A - B.
    void Sub(const ChQuaternion<Real>& A, const ChQuaternion<Real>& B);

    /// Set this quaternion to the quaternion product of the two quaternions A and B,
    /// following the classic Hamilton rule:  this = AxB.
    /// This is the true, typical quaternion product. It is NOT commutative.
    void Cross(const ChQuaternion<Real>& qa, const ChQuaternion<Real>& qb);

    /// Return the dot product with another quaternion: result = this ^ B.
    Real Dot(const ChQuaternion<Real>& B) const;

    /// Set this quaternion to the product of a quaternion A and scalar s: this = A * s.
    void Mul(const ChQuaternion<Real>& A, Real s);

    /// Scale this quaternion by a scalar: this *= s.
    void Scale(Real s);

    /// Normalize this quaternion in place, so that its euclidean length is 1.
    /// Return false if the original quaternion had zero length (in which case the quaternion
    /// is set to [1,0,0,0]) and return true otherwise.
    bool Normalize();

    /// Return a normalized copy of this quaternion, with euclidean length = 1.
    /// Not to be confused with Normalize() which normalizes in place.
    ChQuaternion<Real> GetNormalized() const;

    /// Set this quaternion to the conjugate of the A quaternion.
    void Conjugate(const ChQuaternion<Real>& A);

    /// Conjugate this quaternion in place (its vectorial part changes sign).
    void Conjugate();

    /// Return a conjugated version of this quaternion.
    ChQuaternion<Real> GetConjugate() const;

    /// Return the inverse of this quaternion.
    ChQuaternion<Real> GetInverse() const;

    // TRANSFORMATIONS

    /// Rotate the vector A on the basis of this quaternion: res=p*[0,A]*p'
    /// (speed-optimized version). Endomorphism assumes p is already normalized.
    ChVector<Real> Rotate(const ChVector<Real>& A) const;

    /// Rotate the vector A on the basis of conjugate of this quaternion: res=p'*[0,A]*p
    /// (speed-optimized version).  Endomorphism assumes p is already normalized.
    ChVector<Real> RotateBack(const ChVector<Real>& A) const;

    // CONVERSIONS

    /// Set the quaternion from a rotation vector (ie. a 3D axis of rotation with length as agle of rotation)
    /// defined in absolute coords.
    /// If you need distinct axis and angle, use Q_from_AngAxis().
    void Q_from_Rotv(const ChVector<Real>& angle_axis);

    /// Get the rotation vector (ie. a 3D axis of rotation with length as agle of rotation) from a quaternion.
    /// If you need distinct axis and angle, use rather Q_to_AngAxis().
    ChVector<Real> Q_to_Rotv();

    /// Set the quaternion from an agle of rotation and an axis, defined in absolute coords.
    /// The axis is supposed to be fixed, i.e. it is constant during rotation!
    /// NOTE, axis must be normalized!
    /// If you need directly the rotation vector=axis * angle, use Q_from_Rotv().
    void Q_from_AngAxis(Real angle, const ChVector<Real>& axis);

    /// Set the quaternion from an agle of rotation about X axis.
    void Q_from_AngX(Real angleX) { Q_from_AngAxis(angleX, ChVector<Real>(1, 0, 0)); }

    /// Set the quaternion from an agle of rotation about Y axis.
    void Q_from_AngY(Real angleY) { Q_from_AngAxis(angleY, ChVector<Real>(0, 1, 0)); }

    /// Set the quaternion from an agle of rotation about Z axis.
    void Q_from_AngZ(Real angleZ) { Q_from_AngAxis(angleZ, ChVector<Real>(0, 0, 1)); }

    /// Convert the quaternion to an agle of rotation and an axis, defined in absolute coords.
    /// Resulting angle and axis must be passed as parameters.
    /// If you need directly the rotation vector=axis * angle, use Q_to_Rotv().
    void Q_to_AngAxis(Real& a_angle, ChVector<Real>& a_axis) const;

    /// Set the quaternion from three angles (NASA angle set) heading, bank, and attitude.
    void Q_from_NasaAngles(const ChVector<Real>& ang);

    /// Convert the quaternion to three angles (NASA angle set) heading, bank and attitude.
    ChVector<Real> Q_to_NasaAngles();

    /// Set the quaternion dq/dt. Inputs: the vector of angular speed w specified in absolute coords,
    /// and the rotation expressed as a quaternion q.
    void Qdt_from_Wabs(const ChVector<Real>& w, const ChQuaternion<Real>& q);

    /// Set the quaternion dq/dt. Inputs: the vector of angular speed w specified in relative coords,
    /// and the rotation expressed as a quaternion q.
    void Qdt_from_Wrel(const ChVector<Real>& w, const ChQuaternion<Real>& q);

    /// Compute the vector of angular speed 'w' specified in absolute coords,
    /// from the quaternion dq/dt and the rotation expressed as a quaternion q.
    void Qdt_to_Wabs(ChVector<Real>& w, const ChQuaternion<Real>& q);

    /// Compute the vector of angular speed 'w' specified in relative coords,
    /// from the quaternion dq/dt and the rotation expressed as a quaternion q.
    void Qdt_to_Wrel(ChVector<Real>& w, const ChQuaternion<Real>& q);

    /// Set the quaternion ddq/dtdt. Inputs: the vector of angular acceleration 'a' specified
    /// in absolute coords, the rotation expressed as a quaternion q, the rotation speed
    /// as a quaternion 'q_dt'.
    void Qdtdt_from_Aabs(const ChVector<Real>& a, const ChQuaternion<Real>& q, const ChQuaternion<Real>& q_dt);

    /// Set the quaternion ddq/dtdt. Inputs: the vector of angular acceleration 'a' specified
    /// in relative coords, the rotation expressed as a quaternion q, the rotation speed
    /// as a quaternion 'q_dt'.
    void Qdtdt_from_Arel(const ChVector<Real>& a, const ChQuaternion<Real>& q, const ChQuaternion<Real>& q_dt);

    /// Set the quaternion dq/dt. Inputs:  the axis of rotation 'axis' (assuming it is already normalized
    /// and expressed in absolute coords), the angular speed 'angle_dt' (scalar value), and the
    /// rotation expressed as a quaternion 'q'.
    void Qdt_from_AngAxis(const ChQuaternion<Real>& q, Real angle_dt, const ChVector<Real>& axis);

    /// Set the quaternion ddq/dtdt. Inputs: the axis of ang. acceleration 'axis' (assuming it is already
    /// normalized and expressed in absolute coords), the angular acceleration 'angle_dtdt' (scalar value),
    /// the rotation expressed as a quaternion 'quat' and th rotation speed 'q_dt'.
    void Qdtdt_from_AngAxis(const ChQuaternion<Real>& q,
                            const ChQuaternion<Real>& q_dt,
                            Real angle_dtdt,
                            const ChVector<Real>& axis);

    /// Given the imaginary (vectorial) {e1 e2 e3} part of a quaternion,
    /// tries to set the entire quaternion q = {e0, e1, e2, e3}.  Also for q_dt and q_dtdt.
    /// Note: singularities may happen!
    void ImmQ_complete(const ChVector<Real>& qimm);
    void ImmQ_dt_complete(const ChQuaternion<Real>& q, const ChVector<Real>& qimm_dt);
    void ImmQ_dtdt_complete(const ChQuaternion<Real>& q,
                            const ChQuaternion<Real>& qdt,
                            const ChVector<Real>& qimm_dtdt);

    // STREAMING

    /// Method to allow serialization of transient data in archives.
    void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow de-serialization of transient data from archives.
    void ArchiveIN(ChArchiveIn& marchive);

  private:
    /// Data in the order e0, e1, e2, e3
    Real data[4];

    /// Declaration of friend classes
    template <typename RealB>
    friend class ChQuaternion;
};

CH_CLASS_VERSION(ChQuaternion<double>, 0)

// -----------------------------------------------------------------------------

/// Shortcut for faster use of typical double-precision quaternion.
/// <pre>
/// Instead of writing
///    ChQuaternion<double> foo;
/// or
///    ChQuaternion<> foo;
/// you can use the shorter version
///    Quaternion foo;
/// </pre>
typedef ChQuaternion<double> Quaternion;

/// Shortcut for faster use of typical single-precision quaternion.
/// <pre>
/// Instead of writing
///    ChQuaternion<float> foo;
/// you can use the shorter version
///    Quaternion foo;
/// </pre>
typedef ChQuaternion<float> QuaternionF;

// -----------------------------------------------------------------------------
// CONSTANTS

ChApi extern const ChQuaternion<double> QNULL;
ChApi extern const ChQuaternion<double> QUNIT;

// -----------------------------------------------------------------------------
// STATIC QUATERNION MATH OPERATIONS
//
// These functions are here for people which prefer to use static functions
// instead of ChQuaternion class' member functions.
// NOTE: sometimes a wise adoption of the following functions may give faster
// results than using overloaded operators +/-/* in the quaternion class.

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
ChApi ChQuaternion<double> Qdt_from_AngAxis(const ChQuaternion<double>& quat,
                                            double angle_dt,
                                            const ChVector<double>& axis);

/// Get the quaternion first derivative from the vector of angular acceleration with a specified in _absolute_ coords.
ChApi ChQuaternion<double> Qdtdt_from_Aabs(const ChVector<double>& a,
                                           const ChQuaternion<double>& q,
                                           const ChQuaternion<double>& q_dt);

///	Get the quaternion second derivative from the vector of angular acceleration with a specified in _relative_ coords.
ChApi ChQuaternion<double> Qdtdt_from_Arel(const ChVector<double>& a,
                                           const ChQuaternion<double>& q,
                                           const ChQuaternion<double>& q_dt);

/// Get the second time derivative from a quaternion, an angular acceleration and an axis, defined in _abs_ coords.
ChApi ChQuaternion<double> Qdtdt_from_AngAxis(double angle_dtdt,
                                              const ChVector<double>& axis,
                                              const ChQuaternion<double>& q,
                                              const ChQuaternion<double>& q_dt);

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
ChApi ChQuaternion<double> ImmQ_dtdt_complete(const ChQuaternion<double>& mq,
                                              const ChQuaternion<double>& mqdt,
                                              const ChVector<double>& qimm_dtdt);

ChApi ChVector<double> VaxisXfromQuat(const ChQuaternion<double>& quat);

/// Angle conversion utilities.
ChApi ChVector<double> Quat_to_Angle(AngleSet angset, const ChQuaternion<double>& mquat);
ChApi ChVector<double> Angle_to_Angle(AngleSet setfrom, AngleSet setto, const ChVector<double>& mangles);
ChApi ChQuaternion<double> Angle_to_Quat(AngleSet angset, const ChVector<double>& mangles);
ChApi ChQuaternion<double> AngleDT_to_QuatDT(AngleSet angset,
                                             const ChVector<double>& mangles,
                                             const ChQuaternion<double>& q);
ChApi ChQuaternion<double> AngleDTDT_to_QuatDTDT(AngleSet angset,
                                                 const ChVector<double>& mangles,
                                                 const ChQuaternion<double>& q);

// =============================================================================
// IMPLEMENTATION OF ChQuaternion<Real> methods
// =============================================================================

// -----------------------------------------------------------------------------
// Constructors

template <class Real>
inline ChQuaternion<Real>::ChQuaternion() {
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
}

template <class Real>
inline ChQuaternion<Real>::ChQuaternion(Real e0, Real e1, Real e2, Real e3) {
    data[0] = e0;
    data[1] = e1;
    data[2] = e2;
    data[3] = e3;
}

template <class Real>
inline ChQuaternion<Real>::ChQuaternion(Real s, const ChVector<Real>& v) {
    data[0] = s;
    data[1] = v.x();
    data[2] = v.y();
    data[3] = v.z();
}

template <class Real>
inline ChQuaternion<Real>::ChQuaternion(const ChQuaternion<Real>& other) {
    data[0] = other.data[0];
    data[1] = other.data[1];
    data[2] = other.data[2];
    data[3] = other.data[3];
}

template <class Real>
template <class RealB>
inline ChQuaternion<Real>::ChQuaternion(const ChQuaternion<RealB>& other) {
    data[0] = static_cast<Real>(other.data[0]);
    data[1] = static_cast<Real>(other.data[1]);
    data[2] = static_cast<Real>(other.data[2]);
    data[3] = static_cast<Real>(other.data[3]);
}

// -----------------------------------------------------------------------------
// Subscript operators

template <class Real>
inline Real& ChQuaternion<Real>::operator[](unsigned index) {
    assert(index < 4);
    return data[index];
}

template <class Real>
inline const Real& ChQuaternion<Real>::operator[](unsigned index) const {
    assert(index < 4);
    return data[index];
}

// -----------------------------------------------------------------------------
// Assignments

template <class Real>
inline ChQuaternion<Real>& ChQuaternion<Real>::operator=(const ChQuaternion<Real>& other) {
    if (&other == this)
        return *this;
    data[0] = other.data[0];
    data[1] = other.data[1];
    data[2] = other.data[2];
    data[3] = other.data[3];
    return *this;
}

// -----------------------------------------------------------------------------
// Sign operators

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::operator+() const {
    return *this;
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::operator-() const {
    return ChQuaternion<Real>(-data[0], -data[1], -data[2], -data[3]);
}

// -----------------------------------------------------------------------------
// Arithmetic & quaternion operations

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::operator!() const {
    return ChQuaternion<Real>(data[0], -data[1], -data[2], -data[3]);
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::operator+(const ChQuaternion<Real>& other) const {
    return ChQuaternion<Real>(data[0] + other.data[0], data[1] + other.data[1], data[2] + other.data[2],
                              data[3] + other.data[3]);
}

template <class Real>
inline ChQuaternion<Real>& ChQuaternion<Real>::operator+=(const ChQuaternion<Real>& other) {
    data[0] += other.data[0];
    data[1] += other.data[1];
    data[2] += other.data[2];
    data[3] += other.data[3];
    return *this;
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::operator-(const ChQuaternion<Real>& other) const {
    return ChQuaternion<Real>(data[0] - other.data[0], data[1] - other.data[1], data[2] - other.data[2],
                              data[3] - other.data[3]);
}

template <class Real>
inline ChQuaternion<Real>& ChQuaternion<Real>::operator-=(const ChQuaternion<Real>& other) {
    data[0] -= other.data[0];
    data[1] -= other.data[1];
    data[2] -= other.data[2];
    data[3] -= other.data[3];
    return *this;
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::operator*(const ChQuaternion<Real>& other) const {
    ChQuaternion<Real> q;
    q.Cross(*this, other);
    return q;
}

template <class Real>
inline ChQuaternion<Real>& ChQuaternion<Real>::operator*=(const ChQuaternion<Real>& other) {
    this->Cross(*this, other);
    return *this;
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::operator>>(const ChQuaternion<Real>& other) const {
    ChQuaternion<Real> q;
    q.Cross(other, *this);
    return q;
}

template <class Real>
inline ChQuaternion<Real>& ChQuaternion<Real>::operator>>=(const ChQuaternion<Real>& other) {
    this->Cross(other, *this);
    return *this;
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::operator*(Real s) const {
    return ChQuaternion<Real>(data[0] * s, data[1] * s, data[2] * s, data[3] * s);
}

template <class Real>
inline ChQuaternion<Real>& ChQuaternion<Real>::operator*=(Real s) {
    data[0] *= s;
    data[1] *= s;
    data[2] *= s;
    data[3] *= s;
    return *this;
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::operator/(const ChQuaternion<Real>& other) const {
    return ChQuaternion<Real>(data[0] / other.data[0], data[1] / other.data[1], data[2] / other.data[2],
                              data[3] / other.data[3]);
}

template <class Real>
inline ChQuaternion<Real>& ChQuaternion<Real>::operator/=(const ChQuaternion<Real>& other) {
    data[0] /= other.data[0];
    data[1] /= other.data[1];
    data[2] /= other.data[2];
    data[3] /= other.data[3];
    return *this;
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::operator/(Real s) const {
    Real oos = 1 / s;
    return ChQuaternion<Real>(data[0] * oos, data[1] * oos, data[2] * oos, data[3] * oos);
}

template <class Real>
inline ChQuaternion<Real>& ChQuaternion<Real>::operator/=(Real s) {
    Real oos = 1 / s;
    data[0] *= oos;
    data[1] *= oos;
    data[2] *= oos;
    data[3] *= oos;
    return *this;
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::operator%(const ChQuaternion<Real>& other) const {
    ChQuaternion<Real> q;
    q.Cross(*this, other);
    return q;
}

template <class Real>
inline ChQuaternion<Real>& ChQuaternion<Real>::operator%=(const ChQuaternion<Real>& other) {
    this->Cross(*this, other);
    return *this;
}

template <class Real>
inline Real ChQuaternion<Real>::operator^(const ChQuaternion<Real>& other) const {
    return this->Dot(other);
}

// -----------------------------------------------------------------------------
// Comparison operations

template <class Real>
inline bool ChQuaternion<Real>::operator<=(const ChQuaternion<Real>& other) const {
    return data[0] <= other.data[0] && data[1] <= other.data[1] && data[2] <= other.data[2] && data[3] <= other.data[3];
}

template <class Real>
inline bool ChQuaternion<Real>::operator>=(const ChQuaternion<Real>& other) const {
    return data[0] >= other.data[0] && data[1] >= other.data[1] && data[2] >= other.data[2] && data[3] >= other.data[3];
}

template <class Real>
inline bool ChQuaternion<Real>::operator<(const ChQuaternion<Real>& other) const {
    return data[0] < other.data[0] && data[1] < other.data[1] && data[2] < other.data[2] && data[3] < other.data[3];
}

template <class Real>
inline bool ChQuaternion<Real>::operator>(const ChQuaternion<Real>& other) const {
    return data[0] > other.data[0] && data[1] > other.data[1] && data[2] > other.data[2] && data[3] > other.data[3];
}

template <class Real>
inline bool ChQuaternion<Real>::operator==(const ChQuaternion<Real>& other) const {
    return other.data[0] == data[0] && other.data[1] == data[1] && other.data[2] == data[2] && other.data[3] == data[3];
}

template <class Real>
inline bool ChQuaternion<Real>::operator!=(const ChQuaternion<Real>& other) const {
    return !(*this == other);
}

// -----------------------------------------------------------------------------
// Functions

template <class Real>
inline void ChQuaternion<Real>::Set(Real e0, Real e1, Real e2, Real e3) {
    data[0] = e0;
    data[1] = e1;
    data[2] = e2;
    data[3] = e3;
}

template <class Real>
inline void ChQuaternion<Real>::Set(const ChQuaternion<Real>& q) {
    data[0] = q.data[0];
    data[1] = q.data[1];
    data[2] = q.data[2];
    data[3] = q.data[3];
}

template <class Real>
inline void ChQuaternion<Real>::Set(Real s) {
    data[0] = s;
    data[1] = s;
    data[2] = s;
    data[3] = s;
}

template <class Real>
inline void ChQuaternion<Real>::SetNull() {
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
}

template <class Real>
inline void ChQuaternion<Real>::SetUnit() {
    data[0] = 1;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
}

template <class Real>
inline void ChQuaternion<Real>::SetScalar(Real s) {
    data[0] = s;
}

template <class Real>
inline void ChQuaternion<Real>::SetVector(const ChVector<Real>& v) {
    data[1] = v.x();
    data[2] = v.y();
    data[3] = v.z();
}

template <class Real>
inline bool ChQuaternion<Real>::Equals(const ChQuaternion<Real>& other) const {
    return (other.data[0] == data[0]) && (other.data[1] == data[1]) && (other.data[2] == data[2]) &&
           (other.data[3] == data[3]);
}

template <class Real>
inline bool ChQuaternion<Real>::Equals(const ChQuaternion<Real>& other, Real tol) const {
    return (fabs(other.data[0] - data[0]) < tol) && (fabs(other.data[1] - data[1]) < tol) &&
           (fabs(other.data[2] - data[2]) < tol) && (fabs(other.data[3] - data[3]) < tol);
}

template <class Real>
inline ChVector<Real> ChQuaternion<Real>::GetVector() const {
    return ChVector<Real>(data[1], data[2], data[3]);
}

template <class Real>
inline ChVector<Real> ChQuaternion<Real>::GetXaxis() const {
    return ChVector<Real>((data[0] * data[0] + data[1] * data[1]) * 2 - 1, (data[1] * data[2] + data[0] * data[3]) * 2,
                          (data[1] * data[3] - data[0] * data[2]) * 2);
}

template <class Real>
inline ChVector<Real> ChQuaternion<Real>::GetYaxis() const {
    return ChVector<Real>((data[1] * data[2] - data[0] * data[3]) * 2, (data[0] * data[0] + data[2] * data[2]) * 2 - 1,
                          (data[2] * data[3] + data[0] * data[1]) * 2);
}

template <class Real>
inline ChVector<Real> ChQuaternion<Real>::GetZaxis() const {
    return ChVector<Real>((data[1] * data[3] + data[0] * data[2]) * 2, (data[2] * data[3] - data[0] * data[1]) * 2,
                          (data[0] * data[0] + data[3] * data[3]) * 2 - 1);
}

template <class Real>
inline Real ChQuaternion<Real>::Length() const {
    return sqrt(Length2());
}

template <class Real>
inline Real ChQuaternion<Real>::Length2() const {
    return this->Dot(*this);
}

template <class Real>
inline Real ChQuaternion<Real>::LengthInf() const {
    Real e0e1 = ChMax(fabs(data[0]), fabs(data[1]));
    Real e0e1e2 = ChMax(e0e1, fabs(data[2]));
    return ChMax(e0e1e2, fabs(data[3]));
}

template <class Real>
inline void ChQuaternion<Real>::Add(const ChQuaternion<Real>& A, const ChQuaternion<Real>& B) {
    data[0] = A.data[0] + B.data[0];
    data[1] = A.data[1] + B.data[1];
    data[2] = A.data[2] + B.data[2];
    data[3] = A.data[3] + B.data[3];
}

template <class Real>
inline void ChQuaternion<Real>::Sub(const ChQuaternion<Real>& A, const ChQuaternion<Real>& B) {
    data[0] = A.data[0] - B.data[0];
    data[1] = A.data[1] - B.data[1];
    data[2] = A.data[2] - B.data[2];
    data[3] = A.data[3] - B.data[3];
}

template <class Real>
inline void ChQuaternion<Real>::Cross(const ChQuaternion<Real>& qa, const ChQuaternion<Real>& qb) {
    data[0] = qa.data[0] * qb.data[0] - qa.data[1] * qb.data[1] - qa.data[2] * qb.data[2] - qa.data[3] * qb.data[3];
    data[1] = qa.data[0] * qb.data[1] + qa.data[1] * qb.data[0] - qa.data[3] * qb.data[2] + qa.data[2] * qb.data[3];
    data[2] = qa.data[0] * qb.data[2] + qa.data[2] * qb.data[0] + qa.data[3] * qb.data[1] - qa.data[1] * qb.data[3];
    data[3] = qa.data[0] * qb.data[3] + qa.data[3] * qb.data[0] - qa.data[2] * qb.data[1] + qa.data[1] * qb.data[2];
}

template <class Real>
inline Real ChQuaternion<Real>::Dot(const ChQuaternion<Real>& B) const {
    return (data[0] * B.data[0]) + (data[1] * B.data[1]) + (data[2] * B.data[2]) + (data[3] * B.data[3]);
}

template <class Real>
inline void ChQuaternion<Real>::Mul(const ChQuaternion<Real>& A, Real s) {
    data[0] = A.data[0] * s;
    data[1] = A.data[1] * s;
    data[2] = A.data[2] * s;
    data[3] = A.data[3] * s;
}

template <class Real>
inline void ChQuaternion<Real>::Scale(Real s) {
    data[0] *= s;
    data[1] *= s;
    data[2] *= s;
    data[3] *= s;
}

template <class Real>
inline bool ChQuaternion<Real>::Normalize() {
    Real length = this->Length();
    if (length < CH_NANOTOL) {
        data[0] = 1;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        return false;
    }
    this->Scale(1 / length);
    return true;
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::GetNormalized() const {
    ChQuaternion<Real> q(*this);
    q.Normalize();
    return q;
}

template <class Real>
inline void ChQuaternion<Real>::Conjugate(const ChQuaternion<Real>& A) {
    data[0] = +A.data[0];
    data[1] = -A.data[1];
    data[2] = -A.data[2];
    data[3] = -A.data[3];
}

template <class Real>
inline void ChQuaternion<Real>::Conjugate() {
    Conjugate(*this);
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::GetConjugate() const {
    return ChQuaternion<Real>(data[0], -data[1], -data[2], -data[3]);
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::GetInverse() const {
    ChQuaternion<Real> invq = this->GetConjugate();
    invq.Scale(1 / this->Length());
    return invq;
}

template <class Real>
inline ChVector<Real> ChQuaternion<Real>::Rotate(const ChVector<Real>& A) const {
    Real e0e0 = data[0] * data[0];
    Real e1e1 = data[1] * data[1];
    Real e2e2 = data[2] * data[2];
    Real e3e3 = data[3] * data[3];
    Real e0e1 = data[0] * data[1];
    Real e0e2 = data[0] * data[2];
    Real e0e3 = data[0] * data[3];
    Real e1e2 = data[1] * data[2];
    Real e1e3 = data[1] * data[3];
    Real e2e3 = data[2] * data[3];
    return ChVector<Real>(((e0e0 + e1e1) * 2 - 1) * A.x() + ((e1e2 - e0e3) * 2) * A.y() + ((e1e3 + e0e2) * 2) * A.z(),
                          ((e1e2 + e0e3) * 2) * A.x() + ((e0e0 + e2e2) * 2 - 1) * A.y() + ((e2e3 - e0e1) * 2) * A.z(),
                          ((e1e3 - e0e2) * 2) * A.x() + ((e2e3 + e0e1) * 2) * A.y() + ((e0e0 + e3e3) * 2 - 1) * A.z());
}

template <class Real>
inline ChVector<Real> ChQuaternion<Real>::RotateBack(const ChVector<Real>& A) const {
    Real e0e0 = +data[0] * data[0];
    Real e1e1 = +data[1] * data[1];
    Real e2e2 = +data[2] * data[2];
    Real e3e3 = +data[3] * data[3];
    Real e0e1 = -data[0] * data[1];
    Real e0e2 = -data[0] * data[2];
    Real e0e3 = -data[0] * data[3];
    Real e1e2 = +data[1] * data[2];
    Real e1e3 = +data[1] * data[3];
    Real e2e3 = +data[2] * data[3];
    return ChVector<Real>(((e0e0 + e1e1) * 2 - 1) * A.x() + ((e1e2 - e0e3) * 2) * A.y() + ((e1e3 + e0e2) * 2) * A.z(),
                          ((e1e2 + e0e3) * 2) * A.x() + ((e0e0 + e2e2) * 2 - 1) * A.y() + ((e2e3 - e0e1) * 2) * A.z(),
                          ((e1e3 - e0e2) * 2) * A.x() + ((e2e3 + e0e1) * 2) * A.y() + ((e0e0 + e3e3) * 2 - 1) * A.z());
}

template <class Real>
inline void ChQuaternion<Real>::Q_from_Rotv(const ChVector<Real>& angle_axis) {
    Real theta_squared = angle_axis.Length2();
    // For non-zero rotation:
    if (theta_squared > 0) {
        Real theta = sqrt(theta_squared);
        Real half_theta = theta / 2;
        Real k = sin(half_theta) / theta;
        data[0] = cos(half_theta);
        data[1] = angle_axis.x() * k;
        data[2] = angle_axis.y() * k;
        data[3] = angle_axis.z() * k;
    } else {
        // For almost zero rotation:
        Real k(0.5);
        data[0] = Real(1.0);
        data[1] = angle_axis.x() * k;
        data[2] = angle_axis.y() * k;
        data[3] = angle_axis.z() * k;
    }
}

template <class Real>
inline ChVector<Real> ChQuaternion<Real>::Q_to_Rotv() {
    ChVector<Real> angle_axis;
    Real sin_squared = data[1] * data[1] + data[2] * data[2] + data[3] * data[3];
    // For non-zero rotation
    if (sin_squared > 0) {
        Real sin_theta = sqrt(sin_squared);
        Real k = 2 * atan2(sin_theta, data[0]) / sin_theta;
        angle_axis.x() = data[1] * k;
        angle_axis.y() = data[2] * k;
        angle_axis.z() = data[3] * k;
    } else {
        // For almost zero rotation
        Real k(2.0);
        angle_axis.x() = data[1] * k;
        angle_axis.y() = data[2] * k;
        angle_axis.z() = data[3] * k;
    }
    return angle_axis;
}

template <class Real>
inline void ChQuaternion<Real>::Q_from_AngAxis(Real angle, const ChVector<Real>& axis) {
    Real halfang = (angle / 2);
    Real sinhalf = sin(halfang);
    data[0] = cos(halfang);
    data[1] = axis.x() * sinhalf;
    data[2] = axis.y() * sinhalf;
    data[3] = axis.z() * sinhalf;
}

template <class Real>
inline void ChQuaternion<Real>::Q_to_AngAxis(Real& a_angle, ChVector<Real>& a_axis) const {
    Real arg;
    Real invsine;
    if (fabs(data[0]) < 0.99999999) {
        arg = acos(data[0]);
        a_angle = 2 * arg;
        invsine = 1 / sin(arg);
        a_axis.x() = invsine * data[1];
        a_axis.y() = invsine * data[2];
        a_axis.z() = invsine * data[3];
        a_axis.Normalize();
    } else {
        a_axis.x() = 1;
        a_axis.y() = 0;
        a_axis.z() = 0;
        a_angle = 0;
    }
}

template <class Real>
inline void ChQuaternion<Real>::Q_from_NasaAngles(const ChVector<Real>& ang) {
    Real c1 = cos(ang.z() / 2);
    Real s1 = sin(ang.z() / 2);
    Real c2 = cos(ang.x() / 2);
    Real s2 = sin(ang.x() / 2);
    Real c3 = cos(ang.y() / 2);
    Real s3 = sin(ang.y() / 2);

    Real c1c2 = c1 * c2;
    Real s1s2 = s1 * s2;

    data[0] = c1c2 * c3 + s1s2 * s3;
    data[1] = c1c2 * s3 - s1s2 * c3;
    data[2] = c1 * s2 * c3 + s1 * c2 * s3;
    data[3] = s1 * c2 * c3 - c1 * s2 * s3;
}

template <class Real>
inline ChVector<Real> ChQuaternion<Real>::Q_to_NasaAngles() {
    ChVector<Real> nasa;
    Real sqw = data[0] * data[0];
    Real sqx = data[1] * data[1];
    Real sqy = data[2] * data[2];
    Real sqz = data[3] * data[3];
    // heading
    nasa.z() = atan2(2 * (data[1] * data[2] + data[3] * data[0]), (sqx - sqy - sqz + sqw));
    // bank
    nasa.y() = atan2(2 * (data[2] * data[3] + data[1] * data[0]), (-sqx - sqy + sqz + sqw));
    // attitude
    nasa.x() = asin(-2 * (data[1] * data[3] - data[2] * data[0]));
    return nasa;
}

template <class Real>
inline void ChQuaternion<Real>::Qdt_from_Wabs(const ChVector<Real>& w, const ChQuaternion<Real>& q) {
    ChQuaternion<Real> qwo(0, w);
    this->Cross(qwo, q);
    this->Scale((Real)0.5);  // {q_dt} = 1/2 {0,w}*{q}
}

template <class Real>
inline void ChQuaternion<Real>::Qdt_from_Wrel(const ChVector<Real>& w, const ChQuaternion<Real>& q) {
    ChQuaternion<Real> qwl(0, w);
    this->Cross(q, qwl);
    this->Scale((Real)0.5);  // {q_dt} = 1/2 {q}*{0,w_rel}
}

template <class Real>
inline void ChQuaternion<Real>::Qdt_to_Wabs(ChVector<Real>& w, const ChQuaternion<Real>& q) {
    ChQuaternion<Real> qwo;
    qwo.Cross(*this, q.GetConjugate());
    w = qwo.GetVector();
    w.Scale(2);
}

template <class Real>
inline void ChQuaternion<Real>::Qdt_to_Wrel(ChVector<Real>& w, const ChQuaternion<Real>& q) {
    ChQuaternion<Real> qwl;
    qwl.Cross(q.GetConjugate(), *this);
    w = qwl.GetVector();
    w.Scale(2);
}

template <class Real>
inline void ChQuaternion<Real>::Qdtdt_from_Aabs(const ChVector<Real>& a,
                                                const ChQuaternion<Real>& q,
                                                const ChQuaternion<Real>& q_dt) {
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

template <class Real>
inline void ChQuaternion<Real>::Qdtdt_from_Arel(const ChVector<Real>& a,
                                                const ChQuaternion<Real>& q,
                                                const ChQuaternion<Real>& q_dt) {
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

template <class Real>
inline void ChQuaternion<Real>::Qdt_from_AngAxis(const ChQuaternion<Real>& q,
                                                 Real angle_dt,
                                                 const ChVector<Real>& axis) {
    this->Qdt_from_Wabs(angle_dt * axis, q);
}

template <class Real>
inline void ChQuaternion<Real>::Qdtdt_from_AngAxis(const ChQuaternion<Real>& q,
                                                   const ChQuaternion<Real>& q_dt,
                                                   Real angle_dtdt,
                                                   const ChVector<Real>& axis) {
    this->Qdtdt_from_Aabs(angle_dtdt * axis, q, q_dt);
}

template <class Real>
inline void ChQuaternion<Real>::ImmQ_complete(const ChVector<Real>& qimm) {
    SetVector(qimm);
    data[0] = sqrt(1 - data[1] * data[1] - data[2] * data[2] - data[3] * data[3]);
}

template <class Real>
inline void ChQuaternion<Real>::ImmQ_dt_complete(const ChQuaternion<Real>& q, const ChVector<Real>& qimm_dt) {
    SetVector(qimm_dt);
    data[0] = (-q.data[1] * data[1] - q.data[2] * data[2] - q.data[3] * data[3]) / q.data[0];
}

template <class Real>
inline void ChQuaternion<Real>::ImmQ_dtdt_complete(const ChQuaternion<Real>& q,
                                                   const ChQuaternion<Real>& qdt,
                                                   const ChVector<Real>& qimm_dtdt) {
    SetVector(qimm_dtdt);
    data[0] = (-q.data[1] * data[1] - q.data[2] * data[2] - q.data[3] * data[3] - qdt.data[0] * qdt.data[0] -
               qdt.data[1] * qdt.data[1] - qdt.data[2] * qdt.data[2] - qdt.data[3] * qdt.data[3]) /
              q.data[0];
}

// -----------------------------------------------------------------------------
// Streaming operations

template <class Real>
inline void ChQuaternion<Real>::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChQuaternion<double>>();  // must use specialized template (any)
    // stream out all member data
    marchive << CHNVP(data[0]);
    marchive << CHNVP(data[1]);
    marchive << CHNVP(data[2]);
    marchive << CHNVP(data[3]);
}

template <class Real>
inline void ChQuaternion<Real>::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChQuaternion<double>>();  // must use specialized template (any)
    // stream in all member data
    marchive >> CHNVP(data[0]);
    marchive >> CHNVP(data[1]);
    marchive >> CHNVP(data[2]);
    marchive >> CHNVP(data[3]);
}

}  // end namespace chrono

#endif
