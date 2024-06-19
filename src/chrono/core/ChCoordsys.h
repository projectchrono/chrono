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

#ifndef CH_COORDSYS_H
#define CH_COORDSYS_H

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector3.h"

namespace chrono {

/// Representation of a transform with translation and rotation.
/// A 'coordinate system' contains both translational variable (the origin of the axis) and rotational variable (the
/// unit quaternion which represent the special-orthogonal transformation matrix). Basic features for point-coordinate
/// transformations are provided.
///
/// See @ref coordinate_systems manual page.
template <class Real = double>
class ChCoordsys {
  public:
    ChVector3<Real> pos;
    ChQuaternion<Real> rot;

  public:
    /// Default constructor (identity frame).
    ChCoordsys() : pos(ChVector3<Real>(0, 0, 0)), rot(ChQuaternion<Real>(1, 0, 0, 0)) {}

    /// Construct from position and rotation (as quaternion).
    ChCoordsys(const ChVector3<Real>& v, const ChQuaternion<Real>& q = ChQuaternion<Real>(1, 0, 0, 0))
        : pos(v), rot(q) {}

    /// Construct from position v and rotation of angle alpha around unit vector u.
    explicit ChCoordsys(const ChVector3<Real>& v, const Real alpha, const ChVector3<Real>& u) : pos(v) {
        rot.SetFromAngleAxis(alpha, u);
    };

    /// Copy constructor.
    ChCoordsys(const ChCoordsys<Real>& other) : pos(other.pos), rot(other.rot){};

    // EIGEN INTER-OPERABILITY

    /// Construct a coordinate system from an Eigen vector expression.
    template <typename Derived>
    ChCoordsys(const Eigen::MatrixBase<Derived>& vec,
               typename std::enable_if<(Derived::MaxRowsAtCompileTime == 1 || Derived::MaxColsAtCompileTime == 1),
                                       Derived>::type* = 0) {
        pos.x() = vec(0);
        pos.y() = vec(1);
        pos.z() = vec(2);
        rot.e0() = vec(3);
        rot.e1() = vec(4);
        rot.e2() = vec(5);
        rot.e3() = vec(6);
    }

    /// Assign an Eigen vector expression to this coordinate system.
    template <typename Derived>
    ChCoordsys& operator=(const Eigen::MatrixBase<Derived>& vec) {
        pos.x() = vec(0);
        pos.y() = vec(1);
        pos.z() = vec(2);
        rot.e0() = vec(3);
        rot.e1() = vec(4);
        rot.e2() = vec(5);
        rot.e3() = vec(6);
        return *this;
    }

    // OPERATORS OVERLOADING

    /// Assignment operator: copy from another coordinate system.
    ChCoordsys<Real>& operator=(const ChCoordsys<Real>& other) {
        if (&other == this)
            return *this;
        pos = other.pos;
        rot = other.rot;
        return *this;
    }

    bool operator<=(const ChCoordsys<Real>& other) const { return rot <= other.rot && pos <= other.pos; };
    bool operator>=(const ChCoordsys<Real>& other) const { return rot >= other.rot && pos >= other.pos; };

    bool operator==(const ChCoordsys<Real>& other) const { return rot == other.rot && pos == other.pos; }
    bool operator!=(const ChCoordsys<Real>& other) const { return rot != other.rot || pos != other.pos; }

    /// Transform  another coordinate system through this coordinate system.
    /// If A is this coordinate system and F another coordinate system expressed in A, then G = F >> A is the coordinate
    /// system F expresssed in the parent coordinate system of A. For a sequence of transformations, i.e. a chain of
    /// coordinate systems, one can also write:
    ///   G = F >> F_3to2 >> F_2to1 >> F_1to0;
    /// i.e., just like done with a sequence of Denavitt-Hartemberg matrix multiplications (but reverting order).
    /// This operation is not commutative.
    ChCoordsys<Real> operator>>(const ChCoordsys<Real>& F) const { return F.TransformLocalToParent(*this); }

    /// Transform another coordinate system through this coordinate system.
    /// If A is this coordinate system and F another coordinate system expressed in A, then G = A * F is the coordinate
    /// system F expresssed in the parent coordinate system of A. For a sequence of transformations, i.e. a chain of
    /// coordinate systems, one can also write:
    ///   G = F_1to0 * F_2to1 * F_3to2 * F;
    /// i.e., just like done with a sequence of Denavitt-Hartemberg matrix multiplications.
    /// This operation is not commutative.
    ChCoordsys<Real> operator*(const ChCoordsys<Real>& F) const { return this->TransformLocalToParent(F); }

    /// Transform a vector through this coordinate system (express from parent coordinate system).
    /// If A is this coordinate system and v a vector expressed in the parent coordinate system of A, then w = A / v is
    /// the vector expressed in A. In other words, w = A * v  implies v = A/w.
    ChVector3<Real> operator/(const ChVector3<Real>& v) const { return TransformPointParentToLocal(v); }

    /// Transform this coordinate system by pre-multiplication with another coordinate system.
    /// If A is this frame, then A >>= F means A' = F * A or A' = A >> F.
    ChCoordsys<Real>& operator>>=(const ChCoordsys<Real>& F) {
        ConcatenatePreTransformation(F);
        return *this;
    }

    /// Transform this coordinate system by post-multiplication with another coordinate system.
    /// If A is this csys, then A *= F means A' = A * F or A' = F >> A.
    ChCoordsys<Real>& operator*=(const ChCoordsys<Real>& F) {
        ConcatenatePostTransformation(F);
        return *this;
    }

    // Mixed type operators

    /// Performs pre-multiplication of this frame by a vector D, to 'move' by a displacement v.
    ChCoordsys<Real>& operator>>=(const ChVector3<Real>& v) {
        this->pos += v;
        return *this;
    }

    /// Performs pre-multiplication of this frame by a quaternion R, to 'rotate' it by q.
    ChCoordsys<Real>& operator>>=(const ChQuaternion<Real>& q) {
        this->pos = q.Rotate(this->pos);
        this->rot = q * this->rot;
        return *this;
    }

    // FUNCTIONS

    /// Force to z=0, and z rotation only. No normalization to quaternion, however.
    void Force2D() {
        pos.z() = 0;
        rot.e1() = 0;
        rot.e2() = 0;
    }

    /// Return true if this coordinate system is identical to other coordsys.
    bool Equals(const ChCoordsys<Real>& other) const { return rot.Equals(other.rot) && pos.Equals(other.pos); }

    /// Return true if thsi coordinate system is equal to other coordsys, within a tolerance 'tol'.
    bool Equals(const ChCoordsys<Real>& other, Real tol) const {
        return rot.Equals(other.rot, tol) && pos.Equals(other.pos, tol);
    }

    /// Set to no translation and no rotation.
    void SetIdentity() {
        pos = ChVector3<Real>(0, 0, 0);
        rot = ChQuaternion<Real>(1, 0, 0, 0);
    }

    // FUNCTIONS TO TRANSFORM THE FRAME ITSELF

    /// Apply a transformation (rotation and translation) represented by another coordinate system.
    /// This is equivalent to pre-multiply this coordinate system by the other coordinate system F:
    ///     this'= F * this
    ///  or
    ///     this' = this >> F
    void ConcatenatePreTransformation(const ChCoordsys<Real>& F) {
        this->pos = F.TransformPointLocalToParent(this->pos);
        this->rot = F.rot * this->rot;
    }

    /// Apply a transformation (rotation and translation) represented by another coordinate system F in local
    /// coordinate. This is equivalent to post-multiply this coordinate system by the other coordinate system F:
    ///    this'= this * F
    ///  or
    ///    this'= F >> this
    void ConcatenatePostTransformation(const ChCoordsys<Real>& F) {
        this->pos = this->TransformPointLocalToParent(F.pos);
        this->rot = this->rot * F.rot;
    }

    // FUNCTIONS FOR COORDINATE TRANSFORMATIONS

    /// Transform a point from the local coordinate system to the parent coordinate system.
    ChVector3<Real> TransformPointLocalToParent(const ChVector3<Real>& v) const { return pos + rot.Rotate(v); }

    /// Transforms a point from the parent coordinate system to local coordinate system.
    ChVector3<Real> TransformPointParentToLocal(const ChVector3<Real>& v) const { return rot.RotateBack(v - pos); }

    /// Transform a direction from the parent coordinate system to 'this' local coordinate system.
    ChVector3<Real> TransformDirectionLocalToParent(const ChVector3<Real>& d) const { return rot.Rotate(d); }

    /// Transforms a direction from 'this' local coordinate system to parent coordinate system.
    ChVector3<Real> TransformDirectionParentToLocal(const ChVector3<Real>& d) const { return rot.RotateBack(d); }

    /// Transform a wrench from the local coordinate system to the parent coordinate system.
    ChWrench<Real> TransformWrenchLocalToParent(const ChWrench<Real>& w) const {
        auto force_parent = TransformDirectionLocalToParent(w.force);
        return {force_parent,                                                            //
                Vcross(pos, force_parent) + TransformDirectionLocalToParent(w.torque)};  //
    }

    /// Transform a wrench from the parent coordinate system to the local coordinate system.
    ChWrench<Real> TransformWrenchParentToLocal(const ChWrench<Real>& w) const {
        auto force_local = TransformDirectionParentToLocal(w.force);
        auto pos_local = TransformDirectionParentToLocal(-pos);
        return {force_local,                                                                  //
                Vcross(pos_local, force_local) + TransformDirectionParentToLocal(w.torque)};  //
    }

    /// Transform a coordinate system from 'this' local coordinate system to parent coordinate system.
    ChCoordsys<Real> TransformLocalToParent(const ChCoordsys<Real>& F) const {
        return ChCoordsys<Real>(TransformPointLocalToParent(F.pos), rot * F.rot);
    }

    /// Transform a coordinate system from the parent coordinate system to 'this' local coordinate system.
    ChCoordsys<Real> TransformParentToLocal(const ChCoordsys<Real>& F) const {
        return ChCoordsys<Real>(TransformPointParentToLocal(F.pos), rot.GetConjugate() * F.rot);
    }

    // STREAMING

    /// Method to allow serialization of transient data in archives.
    void ArchiveOut(ChArchiveOut& archive_out) {
        // suggested: use versioning
        archive_out.VersionWrite<ChCoordsys<double>>();
        // stream out all member data
        archive_out << CHNVP(pos);
        archive_out << CHNVP(rot);
    }

    /// Method to allow de-serialization of transient data from archives.
    void ArchiveIn(ChArchiveIn& archive_in) {
        // suggested: use versioning
        /*int version =*/archive_in.VersionRead<ChCoordsys<double>>();
        // stream in all member data
        archive_in >> CHNVP(pos);
        archive_in >> CHNVP(rot);
    }
};

CH_CLASS_VERSION(ChCoordsys<double>, 0)

// -----------------------------------------------------------------------------

/// Insertion of coordsys to output stream.
template <typename Real>
inline std::ostream& operator<<(std::ostream& out, const ChCoordsys<Real>& cs) {
    out << cs.pos.x() << "  " << cs.pos.y() << "  " << cs.pos.z() << "  ";
    out << cs.rot.e0() << "  " << cs.rot.e1() << "  " << cs.rot.e2() << "  " << cs.rot.e3();
    return out;
}

// -----------------------------------------------------------------------------

// MIXED ARGUMENT OPERATORS

// Mixing with ChVector

/// The '*' operator that transforms 'mixed' types:
///  vector_C = frame_A * vector_B;
/// where frame_A  is  a ChCoordsys
///       vector_B is  a ChVector
/// Returns a ChVector.
/// The effect is like applying the transformation frame_A to vector_B and get vector_C.
template <class Real>
ChVector3<Real> operator*(const ChCoordsys<Real>& Fa, const ChVector3<Real>& Fb) {
    return Fa.TransformPointLocalToParent(Fb);
}

/// The '*' operator that transforms 'mixed' types:
///  frame_C = vector_A * frame_B;
/// where vector_A is  a ChVector
///       frame_B  is  a ChCoordsys
/// Returns a ChCoordsys.
/// The effect is like applying the translation vector_A to frame_B and get frame_C.
template <class Real>
ChCoordsys<Real> operator*(const ChVector3<Real>& Fa, const ChCoordsys<Real>& Fb) {
    ChCoordsys<Real> res(Fb);
    res.pos += Fa;
    return res;
}

/// The '>>' operator that transforms 'mixed' types:
///  vector_C = vector_A >> frame_B;
/// where vector_A is  a ChVector
///       frame_B  is  a ChCoordsys
/// Returns a ChVector.
/// The effect is like applying the transformation frame_B to frame_A and get frame_C.
template <class Real>
ChVector3<Real> operator>>(const ChVector3<Real>& Fa, const ChCoordsys<Real>& Fb) {
    return Fb.TransformPointLocalToParent(Fa);
}

/// The '>>' operator that transforms 'mixed' types:
///  frame_C = frame_A >> vector_B;
/// where frame_A  is  a ChCoordsys
///       vector_B is  a ChVector
/// Returns a ChCoordsys.
/// The effect is like applying the translation vector_B to frame_A and get frame_C.
template <class Real>
ChCoordsys<Real> operator>>(const ChCoordsys<Real>& Fa, const ChVector3<Real>& Fb) {
    ChCoordsys<Real> res(Fa);
    res.pos += Fb;
    return res;
}

// Mixing with ChQuaternion

/// The '*' operator that transforms 'mixed' types:
///  quat_C = frame_A * quat_B;
/// where frame_A  is  a ChCoordsys
///       quat_B   is  a ChQuaternion
/// Returns a ChQuaternion.
/// The effect is like applying the transformation frame_A to quat_B and get quat_C.
template <class Real>
ChQuaternion<Real> operator*(const ChCoordsys<Real>& Fa, const ChQuaternion<Real>& Fb) {
    return Fa.rot * Fb;
}

/// The '*' operator that transforms 'mixed' types:
///  frame_C = quat_A * frame_B;
/// where quat_A   is  a ChQuaternion
///       frame_B  is  a ChCoordsys
/// Returns a ChCoordsys.
/// The effect is like applying the rotation quat_A to frame_B and get frame_C.
template <class Real>
ChCoordsys<Real> operator*(const ChQuaternion<Real>& Fa, const ChCoordsys<Real>& Fb) {
    ChCoordsys<Real> res(Fa.Rotate(Fb.pos), Fa * Fb.rot);
    return res;
}

/// The '>>' operator that transforms 'mixed' types:
///  quat_C = quat_A >> frame_B;
/// where quat_A   is  a ChQuaternion
///       frame_B  is  a ChCoordsys
/// Returns a ChQuaternion.
/// The effect is like applying the transformation frame_B to quat_A and get quat_C.
template <class Real>
ChQuaternion<Real> operator>>(const ChQuaternion<Real>& Fa, const ChCoordsys<Real>& Fb) {
    return Fa >> Fb.rot;
}

/// The '>>' operator that transforms 'mixed' types:
///  frame_C = frame_A >> quat_B;
/// where frame_A is  a ChCoordsys
///       frame_B is  a ChQuaternion
/// Returns a ChCoordsys.
/// The effect is like applying the rotation quat_B to frame_A and get frame_C.
template <class Real>
ChCoordsys<Real> operator>>(const ChCoordsys<Real>& Fa, const ChQuaternion<Real>& Fb) {
    ChCoordsys<Real> res(Fb.Rotate(Fa.pos), Fa.rot >> Fb);
    return res;
}

// -----------------------------------------------------------------------------

/// Alias for double-precision coordinate systems.
/// <pre>
/// Instead of writing
///    ChCoordsys<double> csys;
/// or
///    ChCoordsys<> csys;
/// you can use:
///    ChCoordsysd csys;
/// </pre>
typedef ChCoordsys<double> ChCoordsysd;

/// Alias for single-precision coordinate systems.
/// <pre>
/// Instead of writing
///    ChCoordsys<float> csys;
/// you can use:
///    ChCoordsysf csys;
/// </pre>
typedef ChCoordsys<float> ChCoordsysf;

// -----------------------------------------------------------------------------
// CONSTANTS

ChApi extern const ChCoordsysd CSYSNULL;
ChApi extern const ChCoordsysd CSYSNORM;

}  // end namespace chrono

#endif
