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

#ifndef CH_QUATERNION_H
#define CH_QUATERNION_H

#include <cmath>
#include <algorithm>
#include <limits>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChVector3.h"
#include "chrono/utils/ChConstants.h"

namespace chrono {

/// Class defining quaternion objects, that is four-dimensional numbers.
/// Unit quaternions can be used to represent rotations in 3d (in which case, they are also known as "Euler
/// parameters").
/// For more details, see the @ref manual_ChQuaternion manual page.
template <class Real = double>
class ChQuaternion {
  public:
    /// Default constructor.
    /// Note that this constructs a null quaternion {0,0,0,0}, not a {1,0,0,0} unit quaternion.
    ChQuaternion();

    /// Constructor from four scalars.
    /// The first is the real part, others are i,j,k imaginary parts
    ChQuaternion(Real e0, Real e1, Real e2, Real e3);

    /// Constructor from real part, and vector with i,j,k imaginary part.
    ChQuaternion(Real s, const ChVector3<Real>& v);

    /// Copy constructor.
    ChQuaternion(const ChQuaternion<Real>& other);

    /// Copy constructor with type change.
    template <class RealB>
    ChQuaternion(const ChQuaternion<RealB>& other);

    /// Access to components
    Real& e0() { return m_data[0]; }
    Real& e1() { return m_data[1]; }
    Real& e2() { return m_data[2]; }
    Real& e3() { return m_data[3]; }
    const Real& e0() const { return m_data[0]; }
    const Real& e1() const { return m_data[1]; }
    const Real& e2() const { return m_data[2]; }
    const Real& e3() const { return m_data[3]; }

    /// Access to underlying array storage.
    Real* data() { return m_data; }
    const Real* data() const { return m_data; }

    // EIGEN INTER-OPERABILITY

    /// Construct a quaternion from an Eigen vector expression.
    template <typename Derived>
    ChQuaternion(const Eigen::MatrixBase<Derived>& vec,
                 typename std::enable_if<(Derived::MaxRowsAtCompileTime == 1 || Derived::MaxColsAtCompileTime == 1),
                                         Derived>::type* = 0) {
        m_data[0] = vec(0);
        m_data[1] = vec(1);
        m_data[2] = vec(2);
        m_data[3] = vec(3);
    }

    /// View this quaternion as an Eigen vector.
    Eigen::Map<Eigen::Matrix<Real, 4, 1>> eigen() { return Eigen::Map<Eigen::Matrix<Real, 4, 1>>(m_data); }
    Eigen::Map<const Eigen::Matrix<Real, 4, 1>> eigen() const {
        return Eigen::Map<const Eigen::Matrix<Real, 4, 1>>(m_data);
    }

    /// Assign an Eigen vector expression to this quaternion.
    template <typename Derived>
    ChQuaternion& operator=(const Eigen::MatrixBase<Derived>& vec) {
        m_data[0] = vec(0);
        m_data[1] = vec(1);
        m_data[2] = vec(2);
        m_data[3] = vec(3);
        return *this;
    }

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
    void SetVector(const ChVector3<Real>& v);

    /// Return true if this quaternion is the null quaternion.
    bool IsNull() const;

    /// Return true if this quaternion is the identity quaternion.
    bool IsIdentity() const;

    /// Return true if quaternion is identical to other quaternion
    bool Equals(const ChQuaternion<Real>& other) const;

    /// Return true if quaternion equals another quaternion, within a tolerance 'tol'
    bool Equals(const ChQuaternion<Real>& other, Real tol) const;

    /// Gets the vectorial part only
    ChVector3<Real> GetVector() const;

    /// Get the X axis of a coordinate system, given the quaternion which its alignment.
    /// It is assumed that the quaternion is normalized (and thus represents a rotation).
    ChVector3<Real> GetAxisX() const;

    /// Get the Y axis of a coordinate system, given the quaternion which its alignment.
    /// It is assumed that the quaternion is normalized (and thus represents a rotation).
    ChVector3<Real> GetAxisY() const;

    /// Get the Z axis of a coordinate system, given the quaternion which its alignment.
    /// It is assumed that the quaternion is normalized (and thus represents a rotation).
    ChVector3<Real> GetAxisZ() const;

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

    /// Operator for quaternion product: A*B means the typical quaternion product.
    /// Notes:
    /// - since unit quaternions can represent rotations, the product can represent a
    ///   concatenation of rotations as:
    ///        frame_rotation_2to0 = frame_rotation_1to0 * frame_rotation_2to1
    /// - pay attention to operator low precedence (see C++ precedence rules!)
    /// - quaternion product is not commutative.
    ChQuaternion<Real> operator*(const ChQuaternion<Real>& other) const;

    /// Operator for quaternion product and assignment:
    /// A*=B means A'=A*B, with typical quaternion product.
    /// Notes:
    /// - since unit quaternions can represent rotations, the product can represent a
    ///   post-concatenation of a rotation in a kinematic chain.
    /// - quaternion product is not commutative.
    ChQuaternion<Real>& operator*=(const ChQuaternion<Real>& other);

    /// Operator for 'specular' quaternion product: A>>B = B*A.
    /// Notes:
    /// - since unit quaternions can represent rotations, the product can represent a
    ///   concatenation of rotations as:
    ///       frame_rotation_2to0 = frame_rotation_2to1 >> frame_rotation_1to0
    /// - pay attention to operator low precedence (see C++ precedence rules!)
    /// - quaternion product is not commutative.
    ChQuaternion<Real> operator>>(const ChQuaternion<Real>& other) const;

    /// Operator for quaternion 'specular' product and assignment:
    /// A>>=B means A'=A>>B, or A'=B*A with typical quaternion product.
    /// Notes:
    /// - since unit quaternions can represent rotations, the product can represent a
    ///   pre-concatenation of a rotation in a kinematic chain.
    /// - quaternion product is not commutative.
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
    ChVector3<Real> Rotate(const ChVector3<Real>& A) const;

    /// Rotate the vector A on the basis of conjugate of this quaternion: res=p'*[0,A]*p
    /// (speed-optimized version).  Endomorphism assumes p is already normalized.
    ChVector3<Real> RotateBack(const ChVector3<Real>& A) const;

    // CONVERSIONS

    /// Set the quaternion from a rotation vector (ie. a 3D axis of rotation with length as angle of rotation)
    /// defined in absolute coords.
    /// If you need distinct axis and angle, use SetFromAngleAxis().
    void SetFromRotVec(const ChVector3<Real>& angle_axis);

    /// Get the rotation vector (ie. a 3D axis of rotation with length as angle of rotation) from a quaternion.
    /// If you need distinct axis and angle, use GetAngleAxis().
    ChVector3<Real> GetRotVec() const;

    /// Set the quaternion from an angle of rotation and an axis, defined in absolute coords.
    /// The axis is supposed to be fixed, i.e. it is constant during rotation!
    /// NOTE, axis must be normalized!
    /// If you need directly the rotation vector=axis * angle, use SetFromRotVec().
    void SetFromAngleAxis(Real angle, const ChVector3<Real>& axis);

    /// Set the quaternion from an angle of rotation about X axis.
    void SetFromAngleX(Real angleX) { SetFromAngleAxis(angleX, ChVector3<Real>(1, 0, 0)); }

    /// Set the quaternion from an angle of rotation about Y axis.
    void SetFromAngleY(Real angleY) { SetFromAngleAxis(angleY, ChVector3<Real>(0, 1, 0)); }

    /// Set the quaternion from an angle of rotation about Z axis.
    void SetFromAngleZ(Real angleZ) { SetFromAngleAxis(angleZ, ChVector3<Real>(0, 0, 1)); }

    /// Convert the quaternion to an angle of rotation and an axis, defined in absolute coords.
    /// Resulting angle and axis must be passed as parameters.
    /// Note that angle is in [-PI....+PI] range. Also remember  (angle, axis) is the same of (-angle,-axis).
    /// If you need directly the rotation vector=axis * angle, use GetRotVec().
    void GetAngleAxis(Real& angle, ChVector3<Real>& axis) const;

    /// Set the quaternion from Cardan angles ZYX (Tait-Bryan sequence Z-Y'-X'', intrinsic rotations).
    /// NASA angles (heading, bank, attitude).
    void SetFromCardanAnglesZYX(const ChVector3<Real>& ang);

    /// Convert the quaternion to Cardan angles ZYX (Tait-Bryan sequence Z-Y'-X'', intrinsic rotations).
    /// NASA angles (heading, bank, attitude).
    ChVector3<Real> GetCardanAnglesZYX() const;

    /// Set the quaternion from Cardan angles XYZ (Tait-Bryan sequence X-Y'-Z'', intrinsic rotations).
    void SetFromCardanAnglesXYZ(const ChVector3<Real>& ang);

    /// Convert the quaternion to Cardan angles XYZ (Tait-Bryan sequence X-Y'-Z'', intrinsic rotations).
    ChVector3<Real> GetCardanAnglesXYZ() const;

    /// Set the quaternion dq/dt. Inputs: the vector of angular speed w specified in absolute coords,
    /// and the rotation expressed as a quaternion q.
    void SetDtFromAngVelAbs(const ChVector3<Real>& w, const ChQuaternion<Real>& q);

    /// Set the quaternion dq/dt. Inputs: the vector of angular speed w specified in relative coords,
    /// and the rotation expressed as a quaternion q.
    void SetDtFromAngVelRel(const ChVector3<Real>& w, const ChQuaternion<Real>& q);

    /// Compute the vector of angular speed 'w' specified in absolute coords,
    /// from this quaternion dq/dt and the rotation expressed as a quaternion q.
    void GetAngVelAbs(ChVector3<Real>& w, const ChQuaternion<Real>& q);

    /// Compute the vector of angular speed 'w' specified in relative coords,
    /// from this quaternion dq/dt and the rotation expressed as a quaternion q.
    void GetAngVelRel(ChVector3<Real>& w, const ChQuaternion<Real>& q);

    /// Set the quaternion ddq/dtdt. Inputs: the vector of angular acceleration 'a' specified
    /// in absolute coords, the rotation expressed as a quaternion q, the rotation speed
    /// as a quaternion 'q_dt'.
    void SetDt2FromAngAccAbs(const ChVector3<Real>& a, const ChQuaternion<Real>& q, const ChQuaternion<Real>& q_dt);

    /// Set the quaternion ddq/dtdt. Inputs: the vector of angular acceleration 'a' specified
    /// in relative coords, the rotation expressed as a quaternion q, the rotation speed
    /// as a quaternion 'q_dt'.
    void SetDt2FromAngAccRel(const ChVector3<Real>& a, const ChQuaternion<Real>& q, const ChQuaternion<Real>& q_dt);

    /// Set the quaternion dq/dt. Inputs:  the axis of rotation 'axis' (assuming it is already normalized
    /// and expressed in absolute coords), the angular speed 'angle_dt' (scalar value), and the
    /// rotation expressed as a quaternion 'q'.
    void SetDtFromAngleAxis(const ChQuaternion<Real>& q, Real angle_dt, const ChVector3<Real>& axis);

    /// Set the quaternion ddq/dtdt. Inputs: the axis of ang. acceleration 'axis' (assuming it is already
    /// normalized and expressed in absolute coords), the angular acceleration 'angle_dtdt' (scalar value),
    /// the rotation expressed as a quaternion 'quat' and th rotation speed 'q_dt'.
    void SetDt2FromAngleAxis(const ChQuaternion<Real>& q,
                             const ChQuaternion<Real>& q_dt,
                             Real angle_dtdt,
                             const ChVector3<Real>& axis);

    // SERIALIZATION

    /// Method to allow serialization of transient m_data to archives.
    void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient m_data from archives.
    void ArchiveIn(ChArchiveIn& archive_in);

  private:
    Real m_data[4];  ///< quaternion data (e0, e1, e2, e3)

    template <typename RealB>
    friend class ChQuaternion;
};

CH_CLASS_VERSION(ChQuaternion<double>, 0)

// -----------------------------------------------------------------------------

/// Alias for double-precision quaternions.
/// <pre>
/// Instead of writing
///    ChQuaternion<double> q;
/// or
///    ChQuaternion<> q;
/// you can use:
///    ChQuaterniond q;
/// </pre>
typedef ChQuaternion<double> ChQuaterniond;

/// Alias for single-precision quaternions.
/// <pre>
/// Instead of writing
///    ChQuaternion<float> q;
/// you can use:
///    ChQuaternionf q;
/// </pre>
typedef ChQuaternion<float> ChQuaternionf;

// -----------------------------------------------------------------------------
// CONSTANTS

/// Constant null quaternion: {0, 0, 0, 0}
ChApi extern const ChQuaterniond QNULL;

/// Constant unit quaternion: {1, 0, 0, 0} ,
/// corresponds to no rotation (diagonal rotation matrix)
ChApi extern const ChQuaterniond QUNIT;

// Constants for rotations of 90 degrees:
ChApi extern const ChQuaterniond Q_ROTATE_Y_TO_X;
ChApi extern const ChQuaterniond Q_ROTATE_Y_TO_Z;
ChApi extern const ChQuaterniond Q_ROTATE_X_TO_Y;
ChApi extern const ChQuaterniond Q_ROTATE_X_TO_Z;
ChApi extern const ChQuaterniond Q_ROTATE_Z_TO_Y;
ChApi extern const ChQuaterniond Q_ROTATE_Z_TO_X;

// Constants for rotations of 180 degrees:
ChApi extern const ChQuaterniond Q_FLIP_AROUND_X;
ChApi extern const ChQuaterniond Q_FLIP_AROUND_Y;
ChApi extern const ChQuaterniond Q_FLIP_AROUND_Z;

// -----------------------------------------------------------------------------
// STATIC QUATERNION MATH OPERATIONS
//
// These functions are here for people which prefer to use static functions
// instead of ChQuaternion class' member functions.
// NOTE: sometimes a wise adoption of the following functions may give faster
// results than using overloaded operators +/-/* in the quaternion class.

/// Check if two quaternions are equal.
ChApi bool Qequal(const ChQuaterniond& qa, const ChQuaterniond& qb);

/// Check if quaternion is not null.
ChApi bool Qnotnull(const ChQuaterniond& qa);

ChApi double Qlength(const ChQuaterniond& q);

ChApi ChQuaterniond Qadd(const ChQuaterniond& qa, const ChQuaterniond& qb);

ChApi ChQuaterniond Qsub(const ChQuaterniond& qa, const ChQuaterniond& qb);

ChApi ChQuaterniond Qscale(const ChQuaterniond& q, double fact);

/// Return the norm two of the quaternion. Euler's parameters have norm = 1
ChApi ChQuaterniond Qnorm(const ChQuaterniond& q);

/// Return the conjugate of the quaternion [s,v1,v2,v3] is [s,-v1,-v2,-v3]
ChApi ChQuaterniond Qconjugate(const ChQuaterniond& q);

/// Return the product of two quaternions. It is non-commutative (like cross product in vectors).
ChApi ChQuaterniond Qcross(const ChQuaterniond& qa, const ChQuaterniond& qb);

/// Get the quaternion time derivative from the vector of angular speed, with w specified in _local_ coords.
ChApi ChQuaterniond QuatDtFromAngVelRel(const ChVector3d& w, const ChQuaterniond& q);

/// Get the quaternion time derivative from the vector of angular speed, with w specified in _absolute_ coords.
ChApi ChQuaterniond QuatDtFromAngVelAbs(const ChVector3d& w, const ChQuaterniond& q);

/// Get the quaternion first derivative from the vector of angular acceleration with a specified in _absolute_ coords.
ChApi ChQuaterniond QuatDt2FromAngAccAbs(const ChVector3d& a, const ChQuaterniond& q, const ChQuaterniond& q_dt);

///	Get the quaternion second derivative from the vector of angular acceleration with a specified in _relative_ coords.
ChApi ChQuaterniond QuatDt2FromAngAccRel(const ChVector3d& a, const ChQuaterniond& q, const ChQuaterniond& q_dt);

/// Given the imaginary (vectorial) {e1 e2 e3} part of a quaternion,
/// find the entire quaternion q = {e0, e1, e2, e3}.
/// Note: singularities are possible.
ChApi ChQuaterniond QuatFromImaginary(const ChVector3d& imm);

/// Given the imaginary (vectorial) {e1 e2 e3} part of a quaternion time derivative,
/// find the entire quaternion q = {e0, e1, e2, e3}.
/// Note: singularities are possible.
ChApi ChQuaterniond QuatDtFromImaginary(const ChVector3d& im_dt, const ChQuaterniond& q);

/// Given the imaginary (vectorial) {e1 e2 e3} part of a quaternion second time derivative,
/// find the entire quaternion q = {e0, e1, e2, e3}.
/// Note: singularities are possible.
ChApi ChQuaterniond QuatDt2FromImaginary(const ChVector3d& im_dtdt, const ChQuaterniond& q, const ChQuaterniond& q_dt);

ChApi ChVector3d VaxisXfromQuat(const ChQuaterniond& quat);

/// Insertion of quaternion to output stream.
template <typename Real>
inline std::ostream& operator<<(std::ostream& out, const ChQuaternion<Real>& q) {
    out << q.e0() << "  " << q.e1() << "  " << q.e2() << "  " << q.e3();
    return out;
}

// =============================================================================
// IMPLEMENTATION OF ChQuaternion<Real> methods
// =============================================================================

// -----------------------------------------------------------------------------
// Constructors

template <class Real>
inline ChQuaternion<Real>::ChQuaternion() {
    m_data[0] = 0;
    m_data[1] = 0;
    m_data[2] = 0;
    m_data[3] = 0;
}

template <class Real>
inline ChQuaternion<Real>::ChQuaternion(Real e0, Real e1, Real e2, Real e3) {
    m_data[0] = e0;
    m_data[1] = e1;
    m_data[2] = e2;
    m_data[3] = e3;
}

template <class Real>
inline ChQuaternion<Real>::ChQuaternion(Real s, const ChVector3<Real>& v) {
    m_data[0] = s;
    m_data[1] = v.x();
    m_data[2] = v.y();
    m_data[3] = v.z();
}

template <class Real>
inline ChQuaternion<Real>::ChQuaternion(const ChQuaternion<Real>& other) {
    m_data[0] = other.m_data[0];
    m_data[1] = other.m_data[1];
    m_data[2] = other.m_data[2];
    m_data[3] = other.m_data[3];
}

template <class Real>
template <class RealB>
inline ChQuaternion<Real>::ChQuaternion(const ChQuaternion<RealB>& other) {
    m_data[0] = static_cast<Real>(other.m_data[0]);
    m_data[1] = static_cast<Real>(other.m_data[1]);
    m_data[2] = static_cast<Real>(other.m_data[2]);
    m_data[3] = static_cast<Real>(other.m_data[3]);
}

// -----------------------------------------------------------------------------
// Subscript operators

template <class Real>
inline Real& ChQuaternion<Real>::operator[](unsigned index) {
    assert(index < 4);
    return m_data[index];
}

template <class Real>
inline const Real& ChQuaternion<Real>::operator[](unsigned index) const {
    assert(index < 4);
    return m_data[index];
}

// -----------------------------------------------------------------------------
// Assignments

template <class Real>
inline ChQuaternion<Real>& ChQuaternion<Real>::operator=(const ChQuaternion<Real>& other) {
    if (&other == this)
        return *this;
    m_data[0] = other.m_data[0];
    m_data[1] = other.m_data[1];
    m_data[2] = other.m_data[2];
    m_data[3] = other.m_data[3];
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
    return ChQuaternion<Real>(-m_data[0], -m_data[1], -m_data[2], -m_data[3]);
}

// -----------------------------------------------------------------------------
// Arithmetic & quaternion operations

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::operator!() const {
    return ChQuaternion<Real>(m_data[0], -m_data[1], -m_data[2], -m_data[3]);
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::operator+(const ChQuaternion<Real>& other) const {
    return ChQuaternion<Real>(m_data[0] + other.m_data[0], m_data[1] + other.m_data[1], m_data[2] + other.m_data[2],
                              m_data[3] + other.m_data[3]);
}

template <class Real>
inline ChQuaternion<Real>& ChQuaternion<Real>::operator+=(const ChQuaternion<Real>& other) {
    m_data[0] += other.m_data[0];
    m_data[1] += other.m_data[1];
    m_data[2] += other.m_data[2];
    m_data[3] += other.m_data[3];
    return *this;
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::operator-(const ChQuaternion<Real>& other) const {
    return ChQuaternion<Real>(m_data[0] - other.m_data[0], m_data[1] - other.m_data[1], m_data[2] - other.m_data[2],
                              m_data[3] - other.m_data[3]);
}

template <class Real>
inline ChQuaternion<Real>& ChQuaternion<Real>::operator-=(const ChQuaternion<Real>& other) {
    m_data[0] -= other.m_data[0];
    m_data[1] -= other.m_data[1];
    m_data[2] -= other.m_data[2];
    m_data[3] -= other.m_data[3];
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
    return ChQuaternion<Real>(m_data[0] * s, m_data[1] * s, m_data[2] * s, m_data[3] * s);
}

template <class Real>
inline ChQuaternion<Real>& ChQuaternion<Real>::operator*=(Real s) {
    m_data[0] *= s;
    m_data[1] *= s;
    m_data[2] *= s;
    m_data[3] *= s;
    return *this;
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::operator/(const ChQuaternion<Real>& other) const {
    return ChQuaternion<Real>(m_data[0] / other.m_data[0], m_data[1] / other.m_data[1], m_data[2] / other.m_data[2],
                              m_data[3] / other.m_data[3]);
}

template <class Real>
inline ChQuaternion<Real>& ChQuaternion<Real>::operator/=(const ChQuaternion<Real>& other) {
    m_data[0] /= other.m_data[0];
    m_data[1] /= other.m_data[1];
    m_data[2] /= other.m_data[2];
    m_data[3] /= other.m_data[3];
    return *this;
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::operator/(Real s) const {
    Real oos = 1 / s;
    return ChQuaternion<Real>(m_data[0] * oos, m_data[1] * oos, m_data[2] * oos, m_data[3] * oos);
}

template <class Real>
inline ChQuaternion<Real>& ChQuaternion<Real>::operator/=(Real s) {
    Real oos = 1 / s;
    m_data[0] *= oos;
    m_data[1] *= oos;
    m_data[2] *= oos;
    m_data[3] *= oos;
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
    return m_data[0] <= other.m_data[0] && m_data[1] <= other.m_data[1] && m_data[2] <= other.m_data[2] &&
           m_data[3] <= other.m_data[3];
}

template <class Real>
inline bool ChQuaternion<Real>::operator>=(const ChQuaternion<Real>& other) const {
    return m_data[0] >= other.m_data[0] && m_data[1] >= other.m_data[1] && m_data[2] >= other.m_data[2] &&
           m_data[3] >= other.m_data[3];
}

template <class Real>
inline bool ChQuaternion<Real>::operator<(const ChQuaternion<Real>& other) const {
    return m_data[0] < other.m_data[0] && m_data[1] < other.m_data[1] && m_data[2] < other.m_data[2] &&
           m_data[3] < other.m_data[3];
}

template <class Real>
inline bool ChQuaternion<Real>::operator>(const ChQuaternion<Real>& other) const {
    return m_data[0] > other.m_data[0] && m_data[1] > other.m_data[1] && m_data[2] > other.m_data[2] &&
           m_data[3] > other.m_data[3];
}

template <class Real>
inline bool ChQuaternion<Real>::operator==(const ChQuaternion<Real>& other) const {
    return other.m_data[0] == m_data[0] && other.m_data[1] == m_data[1] && other.m_data[2] == m_data[2] &&
           other.m_data[3] == m_data[3];
}

template <class Real>
inline bool ChQuaternion<Real>::operator!=(const ChQuaternion<Real>& other) const {
    return !(*this == other);
}

// -----------------------------------------------------------------------------
// Functions

template <class Real>
inline void ChQuaternion<Real>::Set(Real e0, Real e1, Real e2, Real e3) {
    m_data[0] = e0;
    m_data[1] = e1;
    m_data[2] = e2;
    m_data[3] = e3;
}

template <class Real>
inline void ChQuaternion<Real>::Set(const ChQuaternion<Real>& q) {
    m_data[0] = q.m_data[0];
    m_data[1] = q.m_data[1];
    m_data[2] = q.m_data[2];
    m_data[3] = q.m_data[3];
}

template <class Real>
inline void ChQuaternion<Real>::Set(Real s) {
    m_data[0] = s;
    m_data[1] = s;
    m_data[2] = s;
    m_data[3] = s;
}

template <class Real>
inline void ChQuaternion<Real>::SetNull() {
    m_data[0] = 0;
    m_data[1] = 0;
    m_data[2] = 0;
    m_data[3] = 0;
}

template <class Real>
inline void ChQuaternion<Real>::SetUnit() {
    m_data[0] = 1;
    m_data[1] = 0;
    m_data[2] = 0;
    m_data[3] = 0;
}

template <class Real>
inline void ChQuaternion<Real>::SetScalar(Real s) {
    m_data[0] = s;
}

template <class Real>
inline void ChQuaternion<Real>::SetVector(const ChVector3<Real>& v) {
    m_data[1] = v.x();
    m_data[2] = v.y();
    m_data[3] = v.z();
}

template <class Real>
inline bool ChQuaternion<Real>::IsNull() const {
    return m_data[0] == 0 && m_data[1] == 0 && m_data[2] == 0 && m_data[3] == 0;
}

template <class Real>
inline bool ChQuaternion<Real>::IsIdentity() const {
    return m_data[0] == 1 && m_data[1] == 0 && m_data[2] == 0 && m_data[3] == 0;
}

template <class Real>
inline bool ChQuaternion<Real>::Equals(const ChQuaternion<Real>& other) const {
    return (other.m_data[0] == m_data[0]) && (other.m_data[1] == m_data[1]) && (other.m_data[2] == m_data[2]) &&
           (other.m_data[3] == m_data[3]);
}

template <class Real>
inline bool ChQuaternion<Real>::Equals(const ChQuaternion<Real>& other, Real tol) const {
    return (std::abs(other.m_data[0] - m_data[0]) < tol) && (std::abs(other.m_data[1] - m_data[1]) < tol) &&
           (std::abs(other.m_data[2] - m_data[2]) < tol) && (std::abs(other.m_data[3] - m_data[3]) < tol);
}

template <class Real>
inline ChVector3<Real> ChQuaternion<Real>::GetVector() const {
    return ChVector3<Real>(m_data[1], m_data[2], m_data[3]);
}

template <class Real>
inline ChVector3<Real> ChQuaternion<Real>::GetAxisX() const {
    return ChVector3<Real>((m_data[0] * m_data[0] + m_data[1] * m_data[1]) * 2 - 1,
                           (m_data[1] * m_data[2] + m_data[0] * m_data[3]) * 2,
                           (m_data[1] * m_data[3] - m_data[0] * m_data[2]) * 2);
}

template <class Real>
inline ChVector3<Real> ChQuaternion<Real>::GetAxisY() const {
    return ChVector3<Real>((m_data[1] * m_data[2] - m_data[0] * m_data[3]) * 2,
                           (m_data[0] * m_data[0] + m_data[2] * m_data[2]) * 2 - 1,
                           (m_data[2] * m_data[3] + m_data[0] * m_data[1]) * 2);
}

template <class Real>
inline ChVector3<Real> ChQuaternion<Real>::GetAxisZ() const {
    return ChVector3<Real>((m_data[1] * m_data[3] + m_data[0] * m_data[2]) * 2,
                           (m_data[2] * m_data[3] - m_data[0] * m_data[1]) * 2,
                           (m_data[0] * m_data[0] + m_data[3] * m_data[3]) * 2 - 1);
}

template <class Real>
inline Real ChQuaternion<Real>::Length() const {
    return std::sqrt(Length2());
}

template <class Real>
inline Real ChQuaternion<Real>::Length2() const {
    return this->Dot(*this);
}

template <class Real>
inline Real ChQuaternion<Real>::LengthInf() const {
    Real e0e1 = std::max(std::abs(m_data[0]), std::abs(m_data[1]));
    Real e0e1e2 = std::max(e0e1, std::abs(m_data[2]));
    return std::max(e0e1e2, std::abs(m_data[3]));
}

template <class Real>
inline void ChQuaternion<Real>::Add(const ChQuaternion<Real>& A, const ChQuaternion<Real>& B) {
    m_data[0] = A.m_data[0] + B.m_data[0];
    m_data[1] = A.m_data[1] + B.m_data[1];
    m_data[2] = A.m_data[2] + B.m_data[2];
    m_data[3] = A.m_data[3] + B.m_data[3];
}

template <class Real>
inline void ChQuaternion<Real>::Sub(const ChQuaternion<Real>& A, const ChQuaternion<Real>& B) {
    m_data[0] = A.m_data[0] - B.m_data[0];
    m_data[1] = A.m_data[1] - B.m_data[1];
    m_data[2] = A.m_data[2] - B.m_data[2];
    m_data[3] = A.m_data[3] - B.m_data[3];
}

template <class Real>
inline void ChQuaternion<Real>::Cross(const ChQuaternion<Real>& qa, const ChQuaternion<Real>& qb) {
    Real w = qa.m_data[0] * qb.m_data[0] - qa.m_data[1] * qb.m_data[1] - qa.m_data[2] * qb.m_data[2] -
             qa.m_data[3] * qb.m_data[3];
    Real x = qa.m_data[0] * qb.m_data[1] + qa.m_data[1] * qb.m_data[0] - qa.m_data[3] * qb.m_data[2] +
             qa.m_data[2] * qb.m_data[3];
    Real y = qa.m_data[0] * qb.m_data[2] + qa.m_data[2] * qb.m_data[0] + qa.m_data[3] * qb.m_data[1] -
             qa.m_data[1] * qb.m_data[3];
    Real z = qa.m_data[0] * qb.m_data[3] + qa.m_data[3] * qb.m_data[0] - qa.m_data[2] * qb.m_data[1] +
             qa.m_data[1] * qb.m_data[2];
    m_data[0] = w;
    m_data[1] = x;
    m_data[2] = y;
    m_data[3] = z;
}

template <class Real>
inline Real ChQuaternion<Real>::Dot(const ChQuaternion<Real>& B) const {
    return (m_data[0] * B.m_data[0]) + (m_data[1] * B.m_data[1]) + (m_data[2] * B.m_data[2]) +
           (m_data[3] * B.m_data[3]);
}

template <class Real>
inline void ChQuaternion<Real>::Mul(const ChQuaternion<Real>& A, Real s) {
    m_data[0] = A.m_data[0] * s;
    m_data[1] = A.m_data[1] * s;
    m_data[2] = A.m_data[2] * s;
    m_data[3] = A.m_data[3] * s;
}

template <class Real>
inline void ChQuaternion<Real>::Scale(Real s) {
    m_data[0] *= s;
    m_data[1] *= s;
    m_data[2] *= s;
    m_data[3] *= s;
}

template <class Real>
inline bool ChQuaternion<Real>::Normalize() {
    Real length = this->Length();
    if (length < std::numeric_limits<Real>::min()) {
        m_data[0] = 1;
        m_data[1] = 0;
        m_data[2] = 0;
        m_data[3] = 0;
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
    m_data[0] = +A.m_data[0];
    m_data[1] = -A.m_data[1];
    m_data[2] = -A.m_data[2];
    m_data[3] = -A.m_data[3];
}

template <class Real>
inline void ChQuaternion<Real>::Conjugate() {
    Conjugate(*this);
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::GetConjugate() const {
    return ChQuaternion<Real>(m_data[0], -m_data[1], -m_data[2], -m_data[3]);
}

template <class Real>
inline ChQuaternion<Real> ChQuaternion<Real>::GetInverse() const {
    ChQuaternion<Real> invq = this->GetConjugate();
    invq.Scale(1 / this->Length2());
    return invq;
}

template <class Real>
inline ChVector3<Real> ChQuaternion<Real>::Rotate(const ChVector3<Real>& A) const {
    Real e0e0 = m_data[0] * m_data[0];
    Real e1e1 = m_data[1] * m_data[1];
    Real e2e2 = m_data[2] * m_data[2];
    Real e3e3 = m_data[3] * m_data[3];
    Real e0e1 = m_data[0] * m_data[1];
    Real e0e2 = m_data[0] * m_data[2];
    Real e0e3 = m_data[0] * m_data[3];
    Real e1e2 = m_data[1] * m_data[2];
    Real e1e3 = m_data[1] * m_data[3];
    Real e2e3 = m_data[2] * m_data[3];
    return ChVector3<Real>(((e0e0 + e1e1) * 2 - 1) * A.x() + ((e1e2 - e0e3) * 2) * A.y() + ((e1e3 + e0e2) * 2) * A.z(),
                           ((e1e2 + e0e3) * 2) * A.x() + ((e0e0 + e2e2) * 2 - 1) * A.y() + ((e2e3 - e0e1) * 2) * A.z(),
                           ((e1e3 - e0e2) * 2) * A.x() + ((e2e3 + e0e1) * 2) * A.y() + ((e0e0 + e3e3) * 2 - 1) * A.z());
}

template <class Real>
inline ChVector3<Real> ChQuaternion<Real>::RotateBack(const ChVector3<Real>& A) const {
    Real e0e0 = +m_data[0] * m_data[0];
    Real e1e1 = +m_data[1] * m_data[1];
    Real e2e2 = +m_data[2] * m_data[2];
    Real e3e3 = +m_data[3] * m_data[3];
    Real e0e1 = -m_data[0] * m_data[1];
    Real e0e2 = -m_data[0] * m_data[2];
    Real e0e3 = -m_data[0] * m_data[3];
    Real e1e2 = +m_data[1] * m_data[2];
    Real e1e3 = +m_data[1] * m_data[3];
    Real e2e3 = +m_data[2] * m_data[3];
    return ChVector3<Real>(((e0e0 + e1e1) * 2 - 1) * A.x() + ((e1e2 - e0e3) * 2) * A.y() + ((e1e3 + e0e2) * 2) * A.z(),
                           ((e1e2 + e0e3) * 2) * A.x() + ((e0e0 + e2e2) * 2 - 1) * A.y() + ((e2e3 - e0e1) * 2) * A.z(),
                           ((e1e3 - e0e2) * 2) * A.x() + ((e2e3 + e0e1) * 2) * A.y() + ((e0e0 + e3e3) * 2 - 1) * A.z());
}

template <class Real>
inline void ChQuaternion<Real>::SetFromRotVec(const ChVector3<Real>& angle_axis) {
    Real theta_squared = angle_axis.Length2();
    // For non-zero rotation:
    if (theta_squared > 1e-30) {
        Real theta = std::sqrt(theta_squared);
        Real half_theta = theta / 2;
        Real k = std::sin(half_theta) / theta;
        m_data[0] = std::cos(half_theta);
        m_data[1] = angle_axis.x() * k;
        m_data[2] = angle_axis.y() * k;
        m_data[3] = angle_axis.z() * k;
    } else {
        // For almost zero rotation:
        Real k(0.5);
        m_data[0] = Real(1.0);
        m_data[1] = angle_axis.x() * k;
        m_data[2] = angle_axis.y() * k;
        m_data[3] = angle_axis.z() * k;
    }
}

template <class Real>
inline ChVector3<Real> ChQuaternion<Real>::GetRotVec() const {
    ChVector3<Real> angle_axis;
    Real sin_squared = m_data[1] * m_data[1] + m_data[2] * m_data[2] + m_data[3] * m_data[3];
    // For non-zero rotation
    if (sin_squared > 1e-30) {
        Real sin_theta = std::sqrt(sin_squared);
        Real k = 2 * std::atan2(sin_theta, m_data[0]) / sin_theta;
        angle_axis.x() = m_data[1] * k;
        angle_axis.y() = m_data[2] * k;
        angle_axis.z() = m_data[3] * k;
    } else {
        // For almost zero rotation
        Real k(2.0);
        angle_axis.x() = m_data[1] * k;
        angle_axis.y() = m_data[2] * k;
        angle_axis.z() = m_data[3] * k;
    }
    return angle_axis;
}

template <class Real>
inline void ChQuaternion<Real>::SetFromAngleAxis(Real angle, const ChVector3<Real>& axis) {
    Real halfang = (angle / 2);
    Real sinhalf = std::sin(halfang);
    m_data[0] = std::cos(halfang);
    m_data[1] = axis.x() * sinhalf;
    m_data[2] = axis.y() * sinhalf;
    m_data[3] = axis.z() * sinhalf;
}

template <class Real>
inline void ChQuaternion<Real>::GetAngleAxis(Real& angle, ChVector3<Real>& axis) const {
    Real sin_squared = m_data[1] * m_data[1] + m_data[2] * m_data[2] + m_data[3] * m_data[3];
    // For non-zero rotation
    if (sin_squared > 0) {
        Real sin_theta = std::sqrt(sin_squared);
        angle = 2 * std::atan2(sin_theta, m_data[0]);
        Real k = 1 / sin_theta;
        axis.x() = m_data[1] * k;
        axis.y() = m_data[2] * k;
        axis.z() = m_data[3] * k;
        axis.Normalize();
    } else {
        // For almost zero rotation
        angle = 0.0;
        axis.x() = 1;  // m_data[1] * 2.0;
        axis.y() = 0;  // m_data[2] * 2.0;
        axis.z() = 0;  // m_data[3] * 2.0;
    }
    // Ensure that angle is always in  [-PI...PI] range
    auto PI = static_cast<Real>(CH_PI);
    if (angle > PI) {
        angle -= 2 * PI;
    } else if (angle < -PI) {
        angle += 2 * PI;
    }
}

template <class Real>
inline void ChQuaternion<Real>::SetFromCardanAnglesZYX(const ChVector3<Real>& ang) {
    Real c1 = std::cos(ang.z() / 2);
    Real s1 = std::sin(ang.z() / 2);
    Real c2 = std::cos(ang.x() / 2);
    Real s2 = std::sin(ang.x() / 2);
    Real c3 = std::cos(ang.y() / 2);
    Real s3 = std::sin(ang.y() / 2);

    Real c1c2 = c1 * c2;
    Real s1s2 = s1 * s2;

    m_data[0] = c1c2 * c3 + s1s2 * s3;
    m_data[1] = c1c2 * s3 - s1s2 * c3;
    m_data[2] = c1 * s2 * c3 + s1 * c2 * s3;
    m_data[3] = s1 * c2 * c3 - c1 * s2 * s3;
}

template <class Real>
inline ChVector3<Real> ChQuaternion<Real>::GetCardanAnglesZYX() const {
    ChVector3<Real> nasa;
    Real sqw = m_data[0] * m_data[0];
    Real sqx = m_data[1] * m_data[1];
    Real sqy = m_data[2] * m_data[2];
    Real sqz = m_data[3] * m_data[3];
    // heading
    nasa.z() = std::atan2(2 * (m_data[1] * m_data[2] + m_data[3] * m_data[0]), (sqx - sqy - sqz + sqw));
    // bank
    nasa.y() = std::atan2(2 * (m_data[2] * m_data[3] + m_data[1] * m_data[0]), (-sqx - sqy + sqz + sqw));
    // attitude
    nasa.x() = std::asin(-2 * (m_data[1] * m_data[3] - m_data[2] * m_data[0]));
    return nasa;
}

template <class Real>
inline void ChQuaternion<Real>::SetFromCardanAnglesXYZ(const ChVector3<Real>& ang) {
    // Angles {phi;theta;psi} aka {roll;pitch;yaw}
    Real t0 = std::cos(ang.z() * Real(0.5));
    Real t1 = std::sin(ang.z() * Real(0.5));
    Real t2 = std::cos(ang.x() * Real(0.5));
    Real t3 = std::sin(ang.x() * Real(0.5));
    Real t4 = std::cos(ang.y() * Real(0.5));
    Real t5 = std::sin(ang.y() * Real(0.5));

    m_data[0] = t0 * t2 * t4 + t1 * t3 * t5;
    m_data[1] = t0 * t3 * t4 - t1 * t2 * t5;
    m_data[2] = t0 * t2 * t5 + t1 * t3 * t4;
    m_data[3] = t1 * t2 * t4 - t0 * t3 * t5;
}

template <class Real>
inline ChVector3<Real> ChQuaternion<Real>::GetCardanAnglesXYZ() const {
    // Angles {phi;theta;psi} aka {roll;pitch;yaw} rotation XYZ
    ChVector3<Real> euler;
    Real sq0 = m_data[0] * m_data[0];
    Real sq1 = m_data[1] * m_data[1];
    Real sq2 = m_data[2] * m_data[2];
    Real sq3 = m_data[3] * m_data[3];
    // roll
    euler.x() = std::atan2(2 * (m_data[2] * m_data[3] + m_data[0] * m_data[1]), sq3 - sq2 - sq1 + sq0);
    // pitch
    euler.y() = -std::asin(2 * (m_data[1] * m_data[3] - m_data[0] * m_data[2]));
    // yaw
    euler.z() = std::atan2(2 * (m_data[1] * m_data[2] + m_data[3] * m_data[0]), sq1 + sq0 - sq3 - sq2);
    return euler;
}

template <class Real>
inline void ChQuaternion<Real>::SetDtFromAngVelAbs(const ChVector3<Real>& w, const ChQuaternion<Real>& q) {
    ChQuaternion<Real> qwo(0, w);
    this->Cross(qwo, q);
    this->Scale((Real)0.5);  // {q_dt} = 1/2 {0,w}*{q}
}

template <class Real>
inline void ChQuaternion<Real>::SetDtFromAngVelRel(const ChVector3<Real>& w, const ChQuaternion<Real>& q) {
    ChQuaternion<Real> qwl(0, w);
    this->Cross(q, qwl);
    this->Scale((Real)0.5);  // {q_dt} = 1/2 {q}*{0,w_rel}
}

template <class Real>
inline void ChQuaternion<Real>::GetAngVelAbs(ChVector3<Real>& w, const ChQuaternion<Real>& q) {
    ChQuaternion<Real> qwo;
    qwo.Cross(*this, q.GetConjugate());
    w = qwo.GetVector();
    w.Scale(2);
}

template <class Real>
inline void ChQuaternion<Real>::GetAngVelRel(ChVector3<Real>& w, const ChQuaternion<Real>& q) {
    ChQuaternion<Real> qwl;
    qwl.Cross(q.GetConjugate(), *this);
    w = qwl.GetVector();
    w.Scale(2);
}

template <class Real>
inline void ChQuaternion<Real>::SetDt2FromAngAccAbs(const ChVector3<Real>& a,
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
inline void ChQuaternion<Real>::SetDt2FromAngAccRel(const ChVector3<Real>& a,
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
inline void ChQuaternion<Real>::SetDtFromAngleAxis(const ChQuaternion<Real>& q,
                                                   Real angle_dt,
                                                   const ChVector3<Real>& axis) {
    this->SetDtFromAngVelAbs(angle_dt * axis, q);
}

template <class Real>
inline void ChQuaternion<Real>::SetDt2FromAngleAxis(const ChQuaternion<Real>& q,
                                                    const ChQuaternion<Real>& q_dt,
                                                    Real angle_dtdt,
                                                    const ChVector3<Real>& axis) {
    this->SetDt2FromAngAccAbs(angle_dtdt * axis, q, q_dt);
}

// -----------------------------------------------------------------------------
// Streaming operations

template <class Real>
inline void ChQuaternion<Real>::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChQuaterniond>();  // must use specialized template (any)
    // stream out all member m_data
    archive_out << CHNVP(m_data[0], "e0");
    archive_out << CHNVP(m_data[1], "e1");
    archive_out << CHNVP(m_data[2], "e2");
    archive_out << CHNVP(m_data[3], "e3");
}

template <class Real>
inline void ChQuaternion<Real>::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChQuaterniond>();  // must use specialized template (any)
    // stream in all member m_data
    archive_in >> CHNVP(m_data[0], "e0");
    archive_in >> CHNVP(m_data[1], "e1");
    archive_in >> CHNVP(m_data[2], "e2");
    archive_in >> CHNVP(m_data[3], "e3");
}

}  // end namespace chrono

#endif
