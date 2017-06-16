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

#ifndef CHFRAME_H
#define CHFRAME_H

#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChMatrixNM.h"
#include "chrono/core/ChTransform.h"

namespace chrono {

/// ChFrame: a class for coordinate systems in 3D space.
///
///  A 'frame' coordinate system has a translation and
/// a rotation respect to a 'parent' coordinate system,
/// usually the absolute (world) coordinates.
///
///  Differently from a simple ChCoordsys object, however,
/// the ChFrame implements some optimizations because
/// each ChFrame stores also a 3x3 rotation matrix, which
/// can speed up coordinate transformations when a large
/// amount of vectors must be transformed by the same
/// coordinate frame.
///
/// Further info at the @ref coordinate_transformations manual page.

template <class Real = double>
class ChFrame {
  public:
    ChCoordsys<Real> coord;  ///< Rotation and position, as vector+quaternion

    // Auxiliary, for faster transformation of many coordinates
    ChMatrix33<Real> Amatrix;  ///< 3x3 orthogonal rotation matrix

    /// Default constructor, or construct from pos and rot (as a quaternion)
    explicit ChFrame(const ChVector<Real>& mv = ChVector<Real>(0, 0, 0),
                     const ChQuaternion<Real>& mq = ChQuaternion<Real>(1, 0, 0, 0))
        : coord(mv, mq), Amatrix(mq) {}

    /// Construct from pos and rotation (as a 3x3 matrix)
    ChFrame(const ChVector<Real>& mv, const ChMatrix33<Real>& ma) : coord(mv, ma.Get_A_quaternion()), Amatrix(ma) {}

    /// Construct from a coordsys
    explicit ChFrame(const ChCoordsys<Real>& mc) : coord(mc), Amatrix(mc.rot) {}

    /// Construct from position mv and rotation of angle alpha around unit vector mu
    ChFrame(const ChVector<Real>& mv, const Real alpha, const ChVector<Real>& mu) : coord(mv, alpha, mu) {
        Amatrix.Set_A_quaternion(coord.rot);
    }

    /// Copy constructor, build from another frame
    ChFrame(const ChFrame<Real>& other) : coord(other.coord), Amatrix(other.Amatrix) {}

    virtual ~ChFrame() {}

    //
    // OPERATORS OVERLOADING
    //

    /// Assignment operator: copy from another frame
    ChFrame<Real>& operator=(const ChFrame<Real>& other) {
        if (&other == this)
            return *this;
        coord = other.coord;
        Amatrix = other.Amatrix;
        return *this;
    }

    /// Returns true for identical frames.
    virtual bool operator==(const ChFrame<Real>& other) const { return Equals(other); }

    /// Returns true for different frames.
    virtual bool operator!=(const ChFrame<Real>& other) const { return !Equals(other); }

    /// The '>>' operator transforms a coordinate system, so
    /// transformations can be represented with this syntax:
    ///  new_frame = old_frame >> tr_frame;
    /// For a sequence of transformations, i.e. a chain of coordinate
    /// systems, you can also write this (like you would do with
    /// a sequence of Denavitt-Hartemberg matrix multiplications,
    /// but in the _opposite_ order...)
    ///  new_frame = old_frame >> frame3to2 >> frame2to1 >> frame1to0;
    /// This operation is not commutative.
    ChFrame<Real> operator>>(const ChFrame<Real>& Fb) const {
        ChFrame<Real> res;
        Fb.TransformLocalToParent(*this, res);
        return res;
    }

    /// The '>>' operator transforms a vector, so transformations
    /// can be represented with this syntax:
    ///  new_v = old_v >> tr_frame;
    /// For a sequence of transformations, i.e. a chain of coordinate
    /// systems, you can also write this (like you would do with
    /// a sequence of Denavitt-Hartemberg matrix multiplications,
    /// but in the opposite order...)
    ///  new_v = old_v >> frame3to2 >> frame2to1 >> frame1to0;
    /// This operation is not commutative.
    //	friend ChVector<Real> operator >> (const ChVector<Real>& V, const ChFrame<Real>& mframe)
    //		{
    //			return mframe.TransformLocalToParent(V);
    //		}

    /// The '*' operator transforms a coordinate system, so
    /// transformations can be represented with this syntax:
    ///  new_frame = tr_frame * old_frame;
    /// For a sequence of transformations, i.e. a chain of coordinate
    /// systems, you can also write this (just like you would do with
    /// a sequence of Denavitt-Hartemberg matrix multiplications!)
    ///  new_frame = frame1to0 * frame2to1 * frame3to2 * old_frame;
    /// This operation is not commutative.
    ///  NOTE: since c++ operator execution is from left to right, in
    /// case of multiple transformations like w=A*B*C*v, the >> operator
    /// performs faster, like  w=v>>C>>B>>A;
    ChFrame<Real> operator*(const ChFrame<Real>& Fb) const {
        ChFrame<Real> res;
        TransformLocalToParent(Fb, res);
        return res;
    }

    /// The '*' operator transforms a vector, so
    /// transformations can be represented with this syntax:
    ///  new_v = tr_frame * old_v;
    /// For a sequence of transformations, i.e. a chain of coordinate
    /// systems, you can also write this (just like you would do with
    /// a sequence of Denavitt-Hartemberg matrix multiplications!)
    ///  new_v = frame1to0 * frame2to1 * frame3to2 * old_v;
    /// This operation is not commutative.
    ///  NOTE: since c++ operator execution is from left to right, in
    /// case of multiple transformations like w=A*B*C*v, the >> operator
    /// performs faster, like  w=v>>C>>B>>A;
    ChVector<Real> operator*(const ChVector<Real>& V) const { return TransformLocalToParent(V); }

    /// The '/' is like the '*' operator (see), but uses the inverse
    /// transformation for A, in A/b. (with A ChFrame, b ChVector)
    /// That is: c=A*b ; b=A/c;
    ChVector<Real> operator/(const ChVector<Real>& V) const { return TransformParentToLocal(V); }

    /// Performs pre-multiplication of this frame by another
    /// frame, for example: A>>=T means  A'=T*A ; or A'=A >> T
    ChFrame<Real>& operator>>=(const ChFrame<Real>& T) {
        ConcatenatePreTransformation(T);
        return *this;
    }

    /// Performs pre-multiplication of this frame by another
    /// frame, for example: A%=T means  A'=T*A ; or A'=A >> T
    /// Note: DEPRECATED, use >>= instead.
    ChFrame<Real>& operator%=(const ChFrame<Real>& T) {
        ConcatenatePreTransformation(T);
        return *this;
    }

    /// Performs post-multiplication of this frame by another
    /// frame, for example: A*=T means  A'=A*T ; or A'=T >> A
    ChFrame<Real>& operator*=(const ChFrame<Real>& T) {
        ConcatenatePostTransformation(T);
        return *this;
    }

    // Mixed type operators:

    /// Performs pre-multiplication of this frame by a vector D, to 'move' by a displacement D:
    ChFrame<Real>& operator>>=(const ChVector<Real>& D) {
        this->coord.pos += D;
        return *this;
    }
    /// Performs pre-multiplication of this frame by a quaternion R, to 'rotate' it by R:
    ChFrame<Real>& operator>>=(const ChQuaternion<Real>& R) {
        this->SetCoord(R.Rotate(this->coord.pos), this->coord.rot >> R);
        return *this;
    }
    /// Performs pre-multiplication of this frame by a ChCoordsys F, to transform it:
    ChFrame<Real>& operator>>=(const ChCoordsys<Real>& F) {
        this->SetCoord(this->coord >> F);
        return *this;
    }

    //
    // FUNCTIONS
    //

    // GET-FUNCTIONS

    /// Return both current rotation and translation as
    /// a coordsystem object, with vector and quaternion
    ChCoordsys<Real>& GetCoord() { return coord; }
    const ChCoordsys<Real>& GetCoord() const { return coord; }

    /// Return the current translation as a 3d vector
    ChVector<Real>& GetPos() { return coord.pos; }
    const ChVector<Real>& GetPos() const { return coord.pos; }

    /// Return the current rotation as a quaternion
    ChQuaternion<Real>& GetRot() { return coord.rot; }
    const ChQuaternion<Real>& GetRot() const { return coord.rot; }

    /// Return the current rotation as a 3x3 matrix
    ChMatrix33<Real>& GetA() { return Amatrix; }
    const ChMatrix33<Real>& GetA() const { return Amatrix; }

    /// Get axis of finite rotation, in parent space
    ChVector<Real> GetRotAxis() {
        ChVector<Real> vtmp;
        Real angle;
        coord.rot.Q_to_AngAxis(angle, vtmp);
        return vtmp;
    }

    /// Get angle of rotation about axis of finite rotation
    Real GetRotAngle() {
        ChVector<Real> vtmp;
        Real angle;
        coord.rot.Q_to_AngAxis(angle, vtmp);
        return angle;
    }

    // SET-FUNCTIONS

    /// Impose both translation and rotation as a
    /// single ChCoordsys. Note: the quaternion part must be
    /// already normalized!
    virtual void SetCoord(const ChCoordsys<Real>& mcoord) {
        coord = mcoord;
        Amatrix.Set_A_quaternion(mcoord.rot);
    }

    /// Impose both translation and rotation.
    /// Note: the quaternion part must be already normalized!
    virtual void SetCoord(const ChVector<Real>& mv, const ChQuaternion<Real>& mq) {
        coord.pos = mv;
        coord.rot = mq;
        Amatrix.Set_A_quaternion(mq);
    }

    /// Impose the rotation as a quaternion.
    /// Note: the quaternion must be already normalized!
    virtual void SetRot(const ChQuaternion<Real>& mrot) {
        coord.rot = mrot;
        Amatrix.Set_A_quaternion(mrot);
    }

    /// Impose the rotation as a 3x3 matrix.
    /// Note: the rotation matrix must be already orthogonal!
    virtual void SetRot(const ChMatrix33<Real>& mA) {
        coord.rot = mA.Get_A_quaternion();
        Amatrix.CopyFromMatrix(mA);
    }

    /// Impose the translation
    virtual void SetPos(const ChVector<Real>& mpos) { coord.pos = mpos; }

    // FUNCTIONS TO TRANSFORM THE FRAME ITSELF

    /// Apply a transformation (rotation and translation) represented by
    /// another ChFrame T. This is equivalent to pre-multiply this frame
    /// by the other frame T:   this'= T * this; or this' = this >> T
    void ConcatenatePreTransformation(const ChFrame<Real>& T) {
        this->SetCoord(T.TransformLocalToParent(coord.pos), T.coord.rot % coord.rot);
    }

    /// Apply a transformation (rotation and translation) represented by
    /// another ChFrame T in local coordinate. This is equivalent to
    /// post-multiply this frame by the other frame T:   this'= this * T; or this'= T >> this
    void ConcatenatePostTransformation(const ChFrame<Real>& T) {
        this->SetCoord(TransformLocalToParent(T.coord.pos), coord.rot % T.coord.rot);
    }

    /// An easy way to move the frame by the amount specified by vector V,
    /// (assuming V expressed in parent coordinates)
    void Move(const ChVector<Real>& V) { this->coord.pos += V; }

    /// Apply both translation and rotation, assuming both expressed in parent
    /// coordinates, as a vector for translation and quaternion for rotation,
    void Move(const ChCoordsys<Real>& VR) { this->SetCoord(VR.TransformLocalToParent(coord.pos), VR.rot % coord.rot); }

    // FUNCTIONS FOR COORDINATE TRANSFORMATIONS

    /// This function transforms a point from the local frame coordinate
    /// system to the parent coordinate system.
    /// OPTIMIZED FOR SPEED.
    /// Since it will use the auxiliary rotation matrix of the ChFrame
    /// object, this function is about 50% faster than TransformParentToLocal
    /// of a ChCoordsys.
    /// \return The point in parent coordinate
    virtual ChVector<Real> TransformLocalToParent(const ChVector<Real>& local) const {
        return ChTransform<Real>::TransformLocalToParent(local, coord.pos, Amatrix);
    }

    virtual ChVector<Real> TransformPointLocalToParent(const ChVector<Real>& local) const {
        return ChTransform<Real>::TransformLocalToParent(local, coord.pos, Amatrix);
    }

    /// This function transforms a point from the parent coordinate
    /// system to local frame coordinate system.
    /// OPTIMIZED FOR SPEED.
    /// Since it will use the auxiliary rotation matrix of the ChFrame
    /// object, this function is about 50% faster than TransformParentToLocal
    /// method of a ChCoordsys.
    /// \return The point in local frame coordinate
    virtual ChVector<Real> TransformParentToLocal(const ChVector<Real>& parent) const {
        return ChTransform<Real>::TransformParentToLocal(parent, coord.pos, Amatrix);
    }

    virtual ChVector<Real> TransformPointParentToLocal(const ChVector<Real>& parent) const {
        return ChTransform<Real>::TransformParentToLocal(parent, coord.pos, Amatrix);
    }

    /// This function transforms a frame from 'this' local coordinate
    /// system to parent frame coordinate system.
    /// \return The frame in parent frame coordinate
    virtual void TransformLocalToParent(
        const ChFrame<Real>& local,  ///< frame to transform, given in local frame coordinates
        ChFrame<Real>& parent        ///< transformed frame, in parent coordinates, will be stored here
        ) const {
        parent.SetCoord(TransformLocalToParent(local.coord.pos), coord.rot % local.coord.rot);
    }

    /// This function transforms a frame from the parent coordinate
    /// system to 'this' local frame coordinate system.
    /// \return The frame in local frame coordinate
    virtual void TransformParentToLocal(
        const ChFrame<Real>& parent,  ///< frame to transform, given in parent coordinates
        ChFrame<Real>& local          ///< transformed frame, in local coordinates, will be stored here
        ) const {
        local.SetCoord(TransformParentToLocal(parent.coord.pos), coord.rot.GetConjugate() % parent.coord.rot);
    }

    /// This function transforms a direction from 'this' local coordinate
    /// system to parent frame coordinate system.
    /// \return The direction in local frame coordinate
    virtual ChVector<Real> TransformDirectionParentToLocal(
        const ChVector<>& mdirection  ///< direction to transform, given in parent coordinates
        ) const {
        return Amatrix.MatrT_x_Vect(mdirection);
    }

    /// This function transforms a direction from the parent frame coordinate system
    /// to 'this' local coordinate system.
    /// \return The direction in parent frame coordinate
    virtual ChVector<Real> TransformDirectionLocalToParent(const ChVector<>& mdirection) const {
        return Amatrix.Matr_x_Vect(mdirection);
    }

    // OTHER FUNCTIONS

    /// Returns true if coordsys is identical to other coordsys
    bool Equals(const ChFrame<Real>& other) const { return coord.Equals(other.coord); }

    /// Returns true if coordsys is equal to other coordsys, within a tolerance 'tol'
    bool Equals(const ChFrame<Real>& other, Real tol) const { return coord.Equals(other.coord, tol); }

    /// Normalize the rotation, so that quaternion has unit length
    void Normalize() {
        coord.rot.Normalize();
        Amatrix.Set_A_quaternion(coord.rot);
    }

    /// Sets to no translation and no rotation
    virtual void SetIdentity() {
        coord.SetIdentity();
        Amatrix.SetIdentity();
    }

    /// The transformation is inverted in place.
    /// That is if w=A*v, then A.Invert();v=A*w;
    virtual void Invert() {
        coord.rot.Conjugate();
        Amatrix.MatrTranspose();
        coord.pos = -Amatrix.Matr_x_Vect(coord.pos);
    }

    ChFrame<Real> GetInverse() const {
        ChFrame<Real> tmp(*this);
        tmp.Invert();
        return tmp;
    }

    /// Fills a 3x4 matrix [Fp(q)], as in  [Fp(q)]*[Fm(q)]' = [A(q)]
    static void SetMatrix_Fp(ChMatrixNM<Real, 3, 4>& Fp, const ChQuaternion<Real>& mq) {
        assert((Fp.GetRows() == 3) && (Fp.GetColumns() == 4));
        Fp(0) = mq.e1();
        Fp(1) = mq.e0();
        Fp(2) = -mq.e3();
        Fp(3) = mq.e2();
        Fp(4) = mq.e2();
        Fp(5) = mq.e3();
        Fp(6) = mq.e0();
        Fp(7) = -mq.e1();
        Fp(8) = mq.e3();
        Fp(9) = -mq.e2();
        Fp(10) = mq.e1();
        Fp(11) = mq.e0();
    }

    /// Fills a 3x4 matrix [Fm(q)], as in  [Fp(q)]*[Fm(q)]' = [A(q)]
    static void SetMatrix_Fm(ChMatrixNM<Real, 3, 4>& Fm, const ChQuaternion<Real>& mq) {
        assert((Fm.GetRows() == 3) && (Fm.GetColumns() == 4));
        Fm(0) = mq.e1();
        Fm(1) = mq.e0();
        Fm(2) = mq.e3();
        Fm(3) = -mq.e2();
        Fm(4) = mq.e2();
        Fm(5) = -mq.e3();
        Fm(6) = mq.e0();
        Fm(7) = mq.e1();
        Fm(8) = mq.e3();
        Fm(9) = mq.e2();
        Fm(10) = -mq.e1();
        Fm(11) = mq.e0();
    }

    /// Fast fill a 3x4 matrix [Gl(q)], as in local angular speed conversion
    /// Wl=[Gl]*q_dt   (btw: [Gl(q)] = 2*[Fp(q')] = 2*[G] with G matrix as in Shabana)
    static void SetMatrix_Gl(ChMatrixNM<Real, 3, 4>& Gl, const ChQuaternion<Real>& mq) {
        assert((Gl.GetRows() == 3) && (Gl.GetColumns() == 4));
        Real de0 = 2 * mq.e0();
        Real de1 = 2 * mq.e1();
        Real de2 = 2 * mq.e2();
        Real de3 = 2 * mq.e3();
        Gl(0) = -de1;
        Gl(1) = de0;
        Gl(2) = de3;
        Gl(3) = -de2;
        Gl(4) = -de2;
        Gl(5) = -de3;
        Gl(6) = de0;
        Gl(7) = de1;
        Gl(8) = -de3;
        Gl(9) = de2;
        Gl(10) = -de1;
        Gl(11) = de0;
    }

    /// Fast fill a 3x4 matrix [Gw(q)], as in absolute angular speed conversion
    /// Ww=[Gw]*q_dt   (btw: [Gw(q)] = 2*[Fm(q')] = 2*[E] with E matrix as in Shabana)
    static void SetMatrix_Gw(ChMatrixNM<Real, 3, 4>& Gw, const ChQuaternion<Real>& mq) {
        assert((Gw.GetRows() == 3) && (Gw.GetColumns() == 4));
        Real de0 = 2 * mq.e0();
        Real de1 = 2 * mq.e1();
        Real de2 = 2 * mq.e2();
        Real de3 = 2 * mq.e3();
        Gw(0) = -de1;
        Gw(1) = de0;
        Gw(2) = -de3;
        Gw(3) = de2;
        Gw(4) = -de2;
        Gw(5) = de3;
        Gw(6) = de0;
        Gw(7) = -de1;
        Gw(8) = -de3;
        Gw(9) = -de2;
        Gw(10) = de1;
        Gw(11) = de0;
    }

    /// Computes the product v=[Gl(mq)]*qb  without the need of having
    /// the [Gl] matrix (just pass the mq quaternion, since Gl is function of mq)
    static ChVector<Real> Gl_x_Quat(const ChQuaternion<Real>& mq, const ChQuaternion<Real>& qb) {
        Real de0 = 2 * mq.e0();
        Real de1 = 2 * mq.e1();
        Real de2 = 2 * mq.e2();
        Real de3 = 2 * mq.e3();
        return ChVector<Real>(-de1 * qb.e0() + de0 * qb.e1() + de3 * qb.e2() - de2 * qb.e3(),
                              -de2 * qb.e0() - de3 * qb.e1() + de0 * qb.e2() + de1 * qb.e3(),
                              -de3 * qb.e0() + de2 * qb.e1() - de1 * qb.e2() + de0 * qb.e3());
    }

    /// Computes the product q=[Gl(mq)]*v  without the need of having
    /// the [Gl] matrix (just pass the mq quaternion, since Gl is function of mq)
    static ChQuaternion<Real> GlT_x_Vect(const ChQuaternion<Real>& mq, const ChVector<Real>& v) {
        Real de0 = 2 * mq.e0();
        Real de1 = 2 * mq.e1();
        Real de2 = 2 * mq.e2();
        Real de3 = 2 * mq.e3();
        return ChQuaternion<Real>(-de1 * v.x() - de2 * v.y() - de3 * v.z(), +de0 * v.x() - de3 * v.y() + de2 * v.z(),
                                  +de3 * v.x() + de0 * v.y() - de1 * v.z(), -de2 * v.x() + de1 * v.y() + de0 * v.z());
    }

    //
    // STREAMING
    //

    /// Method to allow serialization of transient data in archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) {
        // suggested: use versioning
        marchive.VersionWrite<ChFrame<double>>();
        // stream out all member data
        marchive << CHNVP(coord);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) {
        // suggested: use versioning
        int version = marchive.VersionRead<ChFrame<double>>();
        // stream in all member data
        marchive >> CHNVP(coord);
        Amatrix.Set_A_quaternion(coord.rot);
    }
};

CH_CLASS_VERSION(ChFrame<double>,0)


//
// MIXED ARGUMENT OPERATORS
//

// Mixing with ChCoordsys :

/// The '*' operator that transforms a coordinate system of 'mixed' type:
///  frame_C = frame_A * frame_B;
/// where frame_A is  a ChFrame
///       frame_B is  a ChCoordsys
/// Returns a ChCoordsys.
/// The effect is like applying the transformation frame_A to frame_B and get frame_C.
template <class Real>
ChCoordsys<Real> operator*(const ChFrame<Real>& Fa, const ChCoordsys<Real>& Fb) {
    return Fa.coord.TransformLocalToParent(Fb);
}

/// The '*' operator that transforms a coordinate system of 'mixed' type:
///  frame_C = frame_A * frame_B;
/// where frame_A is  a ChCoordsys
///       frame_B is  a ChFrame
/// Returns a ChFrame.
/// The effect is like applying the transformation frame_A to frame_B and get frame_C.
/// Performance warning: this operator promotes frame_A to a temporary ChFrame.
template <class Real>
ChFrame<Real> operator*(const ChCoordsys<Real>& Fa, const ChFrame<Real>& Fb) {
    ChFrame<Real> res;
    ChFrame<Real> Fam(Fa);
    Fam.TransformLocalToParent(Fb, res);
    return res;
}

/// The '>>' operator that transforms a coordinate system of 'mixed' type:
///  frame_C = frame_A >> frame_B;
/// where frame_A is  a ChCoordsys
///       frame_B is  a ChFrame
/// Returns a ChCoordsys.
/// The effect is like applying the transformation frame_B to frame_A and get frame_C.
template <class Real>
ChCoordsys<Real> operator>>(const ChCoordsys<Real>& Fa, const ChFrame<Real>& Fb) {
    return Fb.coord.TransformLocalToParent(Fa);
}

/// The '>>' operator that transforms a coordinate system of 'mixed' type:
///  frame_C = frame_A >> frame_B;
/// where frame_A is  a ChFrame
///       frame_B is  a ChCoordsys
/// Returns a ChFrame.
/// The effect is like applying the transformation frame_B to frame_A and get frame_C.
/// Performance warning: this operator promotes frame_B to a temporary ChFrame.
template <class Real>
ChFrame<Real> operator>>(const ChFrame<Real>& Fa, const ChCoordsys<Real>& Fb) {
    ChFrame<Real> res;
    ChFrame<Real> Fbm(Fb);
    Fbm.TransformLocalToParent(Fa, res);
    return res;
}

// Mixing with ChVector :

/// The '*' operator that transforms 'mixed' types:
///  vector_C = frame_A * vector_B;
/// where frame_A   is  a ChFrame
///       vector_B is  a ChVector
/// Returns a ChVector.
/// The effect is like applying the transformation frame_A to vector_B and get vector_C.
template <class Real>
ChVector<Real> operator*(const ChFrame<Real>& Fa, const ChVector<Real>& Fb) {
    return Fa.TransformLocalToParent(Fb);
}

/// The '*' operator that transforms 'mixed' types:
///  frame_C = vector_A * frame_B;
/// where vector_A is  a ChVector
///       frame_B  is  a ChFrame
/// Returns a ChFrame.
/// The effect is like applying the translation vector_A to frame_B and get frame_C.
template <class Real>
ChFrame<Real> operator*(const ChVector<Real>& Fa, const ChFrame<Real>& Fb) {
    ChFrame<Real> res(Fb);
    res.coord.pos += Fa;
    return res;
}

/// The '>>' operator that transforms 'mixed' types:
///  vector_C = vector_A >> frame_B;
/// where vector_A is  a ChVector
///       frame_B  is  a ChFrame
/// Returns a ChVector.
/// The effect is like applying the transformation frame_B to vector_A and get vector_C.
/// For a sequence of transformations, i.e. a chain of coordinate
/// systems, you can also write this (like you would do with
/// a sequence of Denavitt-Hartemberg matrix multiplications,
/// but in the opposite order...)
///  new_v = old_v >> frame3to2 >> frame2to1 >> frame1to0;
/// This operation is not commutative.
template <class Real>
ChVector<Real> operator>>(const ChVector<Real>& Fa, const ChFrame<Real>& Fb) {
    return Fb.TransformLocalToParent(Fa);
}

/// The '>>' operator that transforms 'mixed' types:
///  frame_C = frame_A >> vector_B;
/// where frame_A is  a ChFrame
///       frame_B is  a ChVector
/// Returns a ChFrame.
/// The effect is like applying the translation vector_B to frame_A and get frame_C.
template <class Real>
ChFrame<Real> operator>>(const ChFrame<Real>& Fa, const ChVector<Real>& Fb) {
    ChFrame<Real> res(Fa);
    res.coord.pos += Fb;
    return res;
}

// Mixing with ChQuaternion :

/// The '*' operator that transforms 'mixed' types:
///  quat_C = frame_A * quat_B;
/// where frame_A  is  a ChFrame
///       quat_B   is  a ChQuaternion
/// Returns a ChQuaternion.
/// The effect is like applying the transformation frame_A to quat_B and get quat_C.
template <class Real>
ChQuaternion<Real> operator*(const ChFrame<Real>& Fa, const ChQuaternion<Real>& Fb) {
    return Fa.coord.rot * Fb;
}

/// The '*' operator that transforms 'mixed' types:
///  frame_C = quat_A * frame_B;
/// where quat_A   is  a ChQuaternion
///       frame_B  is  a ChFrame
/// Returns a ChFrame.
/// The effect is like applying the rotation quat_A to frame_B and get frame_C.
template <class Real>
ChFrame<Real> operator*(const ChQuaternion<Real>& Fa, const ChFrame<Real>& Fb) {
    ChFrame<Real> res(Fa.Rotate(Fb.coord.pos), Fa * Fb.coord.rot);
    return res;
}

/// The '>>' operator that transforms 'mixed' types:
///  quat_C = quat_A >> frame_B;
/// where quat_A   is  a ChQuaternion
///       frame_B  is  a ChFrame
/// Returns a ChQuaternion.
/// The effect is like applying the transformation frame_B to quat_A and get quat_C.
template <class Real>
ChQuaternion<Real> operator>>(const ChQuaternion<Real>& Fa, const ChFrame<Real>& Fb) {
    return Fa >> Fb.coord.rot;
}

/// The '>>' operator that transforms 'mixed' types:
///  frame_C = frame_A >> quat_B;
/// where frame_A is  a ChFrame
///       frame_B is  a ChQuaternion
/// Returns a ChFrame.
/// The effect is like applying the rotation quat_B to frame_A and get frame_C.
template <class Real>
ChFrame<Real> operator>>(const ChFrame<Real>& Fa, const ChQuaternion<Real>& Fb) {
    ChFrame<Real> res(Fb.Rotate(Fa.coord.pos), Fa.coord.rot >> Fb);
    return res;
}

}  // end namespace chrono

#endif
