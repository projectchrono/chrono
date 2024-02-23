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

#ifndef CH_FRAMEMOVING_H
#define CH_FRAMEMOVING_H

#include "chrono/core/ChFrame.h"

namespace chrono {

/// Representation of a moving 3D.
/// A ChFrameMoving is a ChFrame that also keeps track of the frame velocity and acceleration.
///
/// See @ref coordinate_transformations manual page.
template <class Real = double>
class ChFrameMoving : public ChFrame<Real> {
  public:
    /// Construct from pos and rot (as a quaternion)
    explicit ChFrameMoving(const ChVector3<Real>& mv = ChVector3<Real>(0, 0, 0),
                           const ChQuaternion<Real>& mq = ChQuaternion<Real>(1, 0, 0, 0))
        : ChFrame<Real>(mv, mq) {
        Csys_dt.rot = Csys_dtdt.rot = ChQuaternion<Real>(0, 0, 0, 0);
    }

    /// Construct from pos and rotation (as a 3x3 matrix)
    ChFrameMoving(const ChVector3<Real>& mv, const ChMatrix33<Real>& ma) : ChFrame<Real>(mv, ma) {
        Csys_dt.rot = Csys_dtdt.rot = ChQuaternion<Real>(0, 0, 0, 0);
    }

    /// Construct from a coordsys
    explicit ChFrameMoving(const ChCoordsys<Real>& mc) : ChFrame<Real>(mc) {
        Csys_dt.rot = Csys_dtdt.rot = ChQuaternion<Real>(0, 0, 0, 0);
    }

    /// Construct from a frame
    explicit ChFrameMoving(const ChFrame<Real>& mc) : ChFrame<Real>(mc) {
        Csys_dt.rot = Csys_dtdt.rot = ChQuaternion<Real>(0, 0, 0, 0);
    }

    /// Copy constructor, build from another moving frame
    ChFrameMoving(const ChFrameMoving<Real>& other)
        : ChFrame<Real>(other), Csys_dt(other.Csys_dt), Csys_dtdt(other.Csys_dtdt) {}

    /// Destructor
    virtual ~ChFrameMoving() {}

    // OPERATORS OVERLOADING

    /// Assignment operator: copy from another moving frame
    ChFrameMoving<Real>& operator=(const ChFrameMoving<Real>& other) {
        if (&other == this)
            return *this;
        ChFrame<Real>::operator=(other);
        Csys_dt = other.Csys_dt;
        Csys_dtdt = other.Csys_dtdt;
        return *this;
    }

    /// Assignment operator: copy from another frame
    ChFrameMoving<Real>& operator=(const ChFrame<Real>& other) {
        if (&other == this)
            return *this;
        ChFrame<Real>::operator=(other);
        Csys_dt.rot = Csys_dtdt.rot = QNULL;
        return *this;
    }

    /// Returns true for identical frames.
    bool operator==(const ChFrameMoving<Real>& other) const { return Equals(other); }

    /// Returns true for different frames.
    bool operator!=(const ChFrameMoving<Real>& other) const { return !Equals(other); }

    /// The '>>' operator transforms a coordinate system, so
    /// transformations can be represented with this syntax:
    ///  new_frame = old_frame >> tr_frame;
    /// For a sequence of transformations, i.e. a chain of coordinate
    /// systems, you can also write this (like you would do with
    /// a sequence of Denavitt-Hartemberg matrix multiplications,
    /// but in the _opposite_ order...)
    ///  new_frame = old_frame >> frame3to2 >> frame2to1 >> frame1to0;
    /// This operation is not commutative.
    /// Also speeds and accelerations are transformed.
    ChFrameMoving<Real> operator>>(const ChFrameMoving<Real>& Fb) const {
        ChFrameMoving<Real> res;
        Fb.TransformLocalToParent(*this, res);
        return res;
    }

    /// The '*' operator transforms a coordinate system, so
    /// transformations can be represented with this syntax:
    ///  new_frame = tr_frame * old_frame;
    /// For a sequence of transformations, i.e. a chain of coordinate
    /// systems, you can also write this (just like you would do with
    /// a sequence of Denavitt-Hartemberg matrix multiplications!)
    ///  new_frame = frame1to0 * frame2to1 * frame3to2 * old_frame;
    /// This operation is not commutative.
    /// Also speeds and accelerations are transformed.
    ChFrameMoving<Real> operator*(const ChFrameMoving<Real>& Fb) const {
        ChFrameMoving<Real> res;
        TransformLocalToParent(Fb, res);
        return res;
    }

    /// Performs pre-multiplication of this frame by another
    /// frame, for example: A>>=T means  A'=T*A ; or A'=A >> T
    ChFrameMoving<Real>& operator>>=(const ChFrameMoving<Real>& T) {
        ConcatenatePreTransformation(T);
        return *this;
    }

    /// Performs pre-multiplication of this frame by another
    /// frame, for example: A%=T means  A'=T*A ; or A'=A >> T
    /// Note: DEPRECATED, use >>= instead.
    ChFrameMoving<Real>& operator%=(const ChFrameMoving<Real>& T) {
        ConcatenatePreTransformation(T);
        return *this;
    }

    /// Performs post-multiplication of this frame by another
    /// frame, for example: A*=T means  A'=A*T ; or A'=T >> A
    ChFrameMoving<Real>& operator*=(const ChFrameMoving<Real>& T) {
        ConcatenatePostTransformation(T);
        return *this;
    }

    // Mixed type operators:

    /// Performs pre-multiplication of this frame by a vector D, to 'move' by a displacement D:
    ChFrameMoving<Real>& operator>>=(const ChVector3<Real>& D) {
        this->Csys.pos += D;
        return *this;
    }
    /// Performs pre-multiplication of this frame by a quaternion R, to 'rotate' it by R:
    ChFrameMoving<Real>& operator>>=(const ChQuaternion<Real>& R) {
        ChFrameMoving<Real> Fm(VNULL, R);
        ConcatenatePreTransformation(Fm);
        return *this;
    }
    /// Performs pre-multiplication of this frame by a ChCoordsys F:
    ChFrameMoving<Real>& operator>>=(const ChCoordsys<Real>& F) {
        ChFrameMoving<Real> Fm(F);
        ConcatenatePreTransformation(Fm);
        return *this;
    }
    /// Performs pre-multiplication of this frame by a ChFrame F:
    ChFrameMoving<Real>& operator>>=(const ChFrame<Real>& F) {
        ChFrameMoving<Real> Fm(F);
        ConcatenatePreTransformation(Fm);
        return *this;
    }

    // GET-FUNCTIONS

    /// Return both current rotation and translation velocities as a ChCoordsys object.
    ChCoordsys<Real>& GetCsysDer() { return Csys_dt; }
    const ChCoordsys<Real>& GetCsysDer() const { return Csys_dt; }

    /// Return both current rotation and translation accelerations as a ChCoordsys object.
    ChCoordsys<Real>& GetCsysDer2() { return Csys_dtdt; }
    const ChCoordsys<Real>& GetCsysDer2() const { return Csys_dtdt; }

    /// Return the current linear velocity.
    ChVector3<Real>& GetPos_dt() { return Csys_dt.pos; }
    const ChVector3<Real>& GetPos_dt() const { return Csys_dt.pos; }

    /// Return the current linear acceleration.
    ChVector3<Real>& GetPos_dtdt() { return Csys_dtdt.pos; }
    const ChVector3<Real>& GetPos_dtdt() const { return Csys_dtdt.pos; }

    /// Return the current rotation velocity as a quaternion.
    ChQuaternion<Real>& GetRot_dt() { return Csys_dt.rot; }
    const ChQuaternion<Real>& GetRot_dt() const { return Csys_dt.rot; }

    /// Return the current rotation acceleration as a quaternion.
    ChQuaternion<Real>& GetRot_dtdt() { return Csys_dtdt.rot; }
    const ChQuaternion<Real>& GetRot_dtdt() const { return Csys_dtdt.rot; }

    /// Compute the angular velocity (expressed in local coords).
    ChVector3<Real> GetWvel_loc() const {
        ChGlMatrix34<> Gl(this->Csys.rot);
        return Gl * Csys_dt.rot;
    }

    /// Compute the actual angular velocity (expressed in parent coords).
    ChVector3<Real> GetWvel_par() const {
        ChGwMatrix34<> Gw(this->Csys.rot);
        return Gw * Csys_dt.rot;
    }

    /// Compute the actual angular acceleration (expressed in local coords).
    ChVector3<Real> GetWacc_loc() const {
        ChGlMatrix34<> Gl(this->Csys.rot);
        return Gl * Csys_dtdt.rot;
    }

    /// Compute the actual angular acceleration (expressed in parent coords).
    ChVector3<Real> GetWacc_par() const {
        ChGwMatrix34<> Gw(this->Csys.rot);
        return Gw * Csys_dtdt.rot;
    }

    // SET-FUNCTIONS

    /// Set both linear andd rotation velocities as a single ChCoordsys derivative.
    virtual void SetCsysDer(const ChCoordsys<Real>& csys_dt) { Csys_dt = csys_dt; }

    /// Set the linear velocity.
    virtual void SetPos_dt(const ChVector3<Real>& vel) { Csys_dt.pos = vel; }

    /// Set the rotation velocity as a quaternion derivative.
    /// Note: the quaternion must satisfy  dot(q,q_dt)=0
    virtual void SetRot_dt(const ChQuaternion<Real>& q_dt) { Csys_dt.rot = q_dt; }

    /// Set the rotation velocity from the given angular velocity (expressed in local coordinates).
    virtual void SetWvel_loc(const ChVector3<Real>& w) {
        Csys_dt.rot.Cross(this->Csys.rot, ChQuaternion<Real>(0, w));
        Csys_dt.rot *= (Real)0.5;  // q_dt = 1/2 * q * (0,w)
    }

    /// Set the rotation velocity from given angular velocity (expressed in parent coordinates).
    virtual void SetWvel_par(const ChVector3<Real>& w) {
        Csys_dt.rot.Cross(ChQuaternion<Real>(0, w), this->Csys.rot);
        Csys_dt.rot *= (Real)0.5;  // q_dt = 1/2 * (0,w) * q
    }

    /// Set the linear and rotation accelerations as a single ChCoordsys derivative.
    virtual void SetCsysDer2(const ChCoordsys<Real>& csys_dtdt) { Csys_dtdt = csys_dtdt; }

    /// Set the linear acceleration.
    virtual void SetPos_dtdt(const ChVector3<Real>& acc) { Csys_dtdt.pos = acc; }

    /// Set the rotation acceleration as a quaternion derivative.
    /// Note: the quaternion must already satisfy  dot(q,q_dt)=0
    virtual void SetRot_dtdt(const ChQuaternion<Real>& q_dtdt) { Csys_dtdt.rot = q_dtdt; }

    /// Set the rotation acceleration from given angular acceleration (expressed in local coordinates).
    /// Note: even when the local angular acceleration is zero, this function should still be called because q_dtdt
    /// might be nonzero due to nonzero q_dt (in case of rotational motion).
    virtual void SetWacc_loc(const ChVector3<Real>& a) {
        // q_dtdt = q_dt * q' * q_dt + 1/2 * q * (0,a)
        Csys_dtdt.rot = (Csys_dt.rot * this->Csys.rot.GetConjugate() * Csys_dt.rot) +
                        (this->Csys.rot * ChQuaternion<Real>(0, a) * (Real)0.5);
    }

    /// Set the rotation acceleration from given angular acceleration (expressed in parent coordinates).
    virtual void SetWacc_par(const ChVector3<Real>& a) {
        // q_dtdt = q_dt * q' * q_dt + 1/2 * (0,a) * q
        Csys_dtdt.rot = (Csys_dt.rot * this->Csys.rot.GetConjugate() * Csys_dt.rot) +
                        (ChQuaternion<Real>(0, a) * this->Csys.rot * (Real)0.5);
    }

    /// Compute the time derivative of the rotation matrix.
    void Compute_Adt(ChMatrix33<Real>& R_dt) const {
        //  [A_dt]=2[dFp/dt][Fm]'=2[Fp(q_dt)][Fm(q)]'
        ChFpMatrix34<Real> Fpdt(Csys_dt.rot);
        ChFmMatrix34<Real> Fm(this->Csys.rot);
        R_dt = 2 * Fpdt * Fm.transpose();
    }

    /// Compute the second time derivative of the rotation matrix.
    void Compute_Adtdt(ChMatrix33<Real>& R_dtdt) {
        //  [A_dtdt]=2[Fp(q_dtdt)][Fm(q)]'+2[Fp(q_dt)][Fm(q_dt)]'
        ChFpMatrix34<> Fpdtdt(Csys_dtdt.rot);
        ChFmMatrix34<> Fm(this->Csys.rot);
        ChFpMatrix34<> Fpdt(Csys_dt.rot);
        ChFmMatrix34<> Fmdt(Csys_dt.rot);
        R_dtdt = 2 * (Fpdtdt * Fm.transpose() + Fpdt * Fmdt.transpose());
    }

    /// Return the time derivative of the rotation matrix.
    ChMatrix33<Real> GetA_dt() {
        ChMatrix33<Real> res;
        Compute_Adt(res);
        return res;
    }

    /// Return the second time derivative of the rotation matrix.
    ChMatrix33<Real> GetA_dtdt() {
        ChMatrix33<Real> res;
        Compute_Adtdt(res);
        return res;
    }

    // FUNCTIONS TO TRANSFORM THE FRAME ITSELF

    /// Apply a transformation (rotation and translation) represented by another frame.
    /// This is equivalent to pre-multiply this frame by the other frame F:
    ///     this'= F * this
    ///  or
    ///     this' = this >> F
    void ConcatenatePreTransformation(const ChFrameMoving<Real>& F) {
        ChFrameMoving<Real> res;
        F.TransformLocalToParent(*this, res);
        *this = res;
    }

    /// Apply a transformation (rotation and translation) represented by another frame F in local coordinate.
    /// This is equivalent to post-multiply this frame by the other frame F:
    ///    this'= this * F
    ///  or
    ///    this'= F >> this
    void ConcatenatePostTransformation(const ChFrameMoving<Real>& F) {
        ChFrameMoving<Real> res;
        this->TransformLocalToParent(F, res);
        *this = res;
    }

    // FUNCTIONS FOR COORDINATE TRANSFORMATIONS

    /// Given the position of a point in local frame coords, and
    /// assuming it is sticky to frame, return the speed in parent coords.
    ChVector3<Real> PointSpeedLocalToParent(const ChVector3<Real>& localpos) const {
        return Csys_dt.pos +
               ((Csys_dt.rot * ChQuaternion<Real>(0, localpos) * this->Csys.rot.GetConjugate()).GetVector() * 2);
    }

    /// Given the position localpos of a point in the local reference frame, assuming
    /// that the point moves in the local reference frame with localspeed,
    /// return the speed in the parent reference frame.
    ChVector3<Real> PointSpeedLocalToParent(const ChVector3<Real>& localpos, const ChVector3<Real>& localspeed) const {
        return Csys_dt.pos + this->Rmat * localspeed +
               ((Csys_dt.rot * ChQuaternion<Real>(0, localpos) * this->Csys.rot.GetConjugate()).GetVector() * 2);
    }

    /// Given the position of a point in local frame coords, and
    /// assuming it is sticky to frame, return the acceleration in parent coords.
    /// Note: please ensure all the first and second derivatives of pos and rot are assigned as the precondition.
    /// A special occasion: when the local angular acceleration is zero, it's still necessary to call SetWacc_loc(VNULL)
    /// bacause the q_dtdt may be nonzero due to nonzero q_dt in case of rotational motion.
    ChVector3<Real> PointAccelerationLocalToParent(const ChVector3<Real>& localpos) const {
        return Csys_dtdt.pos +
               ((Csys_dtdt.rot * ChQuaternion<Real>(0, localpos) * this->Csys.rot.GetConjugate()).GetVector() * 2) +
               ((Csys_dt.rot * ChQuaternion<Real>(0, localpos) * Csys_dt.rot.GetConjugate()).GetVector() * 2);
    }

    /// Given the position of a point in local frame coords, and
    /// assuming it has a frame-relative speed localspeed and frame-relative
    /// acceleration localacc, return the acceleration in parent coords.
    ChVector3<Real> PointAccelerationLocalToParent(const ChVector3<Real>& localpos,
                                                   const ChVector3<Real>& localspeed,
                                                   const ChVector3<Real>& localacc) const {
        return Csys_dtdt.pos + this->Rmat * localacc +
               ((Csys_dtdt.rot * ChQuaternion<Real>(0, localpos) * this->Csys.rot.GetConjugate()).GetVector() * 2) +
               ((Csys_dt.rot * ChQuaternion<Real>(0, localpos) * Csys_dt.rot.GetConjugate()).GetVector() * 2) +
               ((Csys_dt.rot * ChQuaternion<Real>(0, localspeed) * this->Csys.rot.GetConjugate()).GetVector() * 4);
    }

    /// Given the position of a point in parent frame coords, and
    /// assuming it has an absolute speed parentspeed,
    /// return the speed in local coords.
    ChVector3<Real> PointSpeedParentToLocal(const ChVector3<Real>& parentpos,
                                            const ChVector3<Real>& parentspeed) const {
        ChVector3<Real> localpos = ChFrame<Real>::TransformPointParentToLocal(parentpos);
        return this->Rmat.transpose() *
               (parentspeed - Csys_dt.pos -
                ((Csys_dt.rot * ChQuaternion<Real>(0, localpos) * this->Csys.rot.GetConjugate()).GetVector() * 2));
    }

    /// Given the position of a point in parent frame coords, and
    /// assuming it has an absolute speed parentspeed and absolute
    /// acceleration parentacc, return the acceleration in local coords.
    ChVector3<Real> PointAccelerationParentToLocal(const ChVector3<Real>& parentpos,
                                                   const ChVector3<Real>& parentspeed,
                                                   const ChVector3<Real>& parentacc) const {
        ChVector3<Real> localpos = ChFrame<Real>::TransformPointParentToLocal(parentpos);
        ChVector3<Real> localspeed = PointSpeedParentToLocal(parentpos, parentspeed);
        return this->Rmat.transpose() *
               (parentacc - Csys_dtdt.pos -
                (Csys_dtdt.rot * ChQuaternion<Real>(0, localpos) * this->Csys.rot.GetConjugate()).GetVector() * 2 -
                (Csys_dt.rot * ChQuaternion<Real>(0, localpos) * Csys_dt.rot.GetConjugate()).GetVector() * 2 -
                (Csys_dt.rot * ChQuaternion<Real>(0, localspeed) * this->Csys.rot.GetConjugate()).GetVector() * 4);
    }

    /// Transform a frame from 'this' local coordinate system to parent frame coordinate system.
    /// Also transform the speed and acceleration of the frame.
    void TransformLocalToParent(const ChFrameMoving<Real>& local,  ///< frame to transform, given in local coordinates
                                ChFrameMoving<Real>& parent        ///< transformed frame, in parent coordinates
    ) const {
        // pos & rot
        ChFrame<Real>::TransformLocalToParent(local, parent);

        // pos_dt
        parent.Csys_dt.pos = PointSpeedLocalToParent(local.Csys.pos, local.Csys_dt.pos);

        // pos_dtdt
        parent.Csys_dtdt.pos = PointAccelerationLocalToParent(local.Csys.pos, local.Csys_dt.pos, local.Csys_dtdt.pos);

        // rot_dt
        parent.Csys_dt.rot = Csys_dt.rot * local.Csys.rot + this->Csys.rot * local.Csys_dt.rot;

        // rot_dtdt
        parent.Csys_dtdt.rot = Csys_dtdt.rot * local.Csys.rot + (Csys_dt.rot * local.Csys_dt.rot) * 2 +
                               this->Csys.rot * local.Csys_dtdt.rot;
    }

    /// Transform a frame from the parent coordinate system to 'this' local frame coordinate system.
    /// Also transform the speed and acceleration of the frame.
    void TransformParentToLocal(const ChFrameMoving<Real>& parent,  ///< frame to transform, given in parent coordinates
                                ChFrameMoving<Real>& local          ///< transformed frame, in local coordinates
    ) const {
        // pos & rot
        ChFrame<Real>::TransformParentToLocal(parent, local);

        // pos_dt
        local.Csys_dt.pos = PointSpeedParentToLocal(parent.Csys.pos, parent.Csys_dt.pos);

        // pos_dtdt
        local.Csys_dtdt.pos = PointAccelerationParentToLocal(parent.Csys.pos, parent.Csys_dt.pos, parent.Csys_dtdt.pos);

        // rot_dt
        local.Csys_dt.rot = this->Csys.rot.GetConjugate() * (parent.Csys_dt.rot - Csys_dt.rot * local.Csys.rot);

        // rot_dtdt
        local.Csys_dtdt.rot = this->Csys.rot.GetConjugate() * (parent.Csys_dtdt.rot - Csys_dtdt.rot * local.Csys.rot -
                                                               (Csys_dt.rot * local.Csys_dt.rot) * 2);
    }

    // OTHER FUNCTIONS

    /// Returns true if this transform is identical to the other transform.
    bool Equals(const ChFrameMoving<Real>& other) const {
        return this->Csys.Equals(other.Csys) && Csys_dt.Equals(other.Csys_dt) && Csys_dtdt.Equals(other.Csys_dtdt);
    }

    /// Returns true if this transform is equal to the other transform, within a tolerance 'tol'.
    bool Equals(const ChFrameMoving<Real>& other, Real tol) const {
        return this->Csys.Equals(other.Csys, tol) && Csys_dt.Equals(other.Csys_dt, tol) &&
               Csys_dtdt.Equals(other.Csys_dtdt, tol);
    }

    /// Invert in place.
    /// If w=A*v, after A.Invert() we have v=A*w;
    virtual void Invert() override {
        ChFrameMoving<Real> tmp;
        ChFrameMoving<Real> unit;
        tmp = *this;
        tmp.TransformParentToLocal(unit, *this);
    }

    /// Return the inverse transform.
    ChFrameMoving<Real> GetInverse() const {
        ChFrameMoving<Real> tmp(*this);
        tmp.Invert();
        return tmp;
    }

    virtual void ArchiveOut(ChArchiveOut& archive_out) override {
        // version number
        archive_out.VersionWrite<ChFrameMoving>();

        // serialize parent class
        ChFrame<Real>::ArchiveOut(archive_out);

        // serialize all member data
        archive_out << CHNVP(Csys_dt);
        archive_out << CHNVP(Csys_dtdt);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override {
        // version number
        /*int version =*/archive_in.VersionRead<ChFrameMoving>();

        // deserialize parent class
        ChFrame<Real>::ArchiveIn(archive_in);

        // stream in all member data
        archive_in >> CHNVP(Csys_dt);
        archive_in >> CHNVP(Csys_dtdt);
    }

  protected:
    ChCoordsys<Real> Csys_dt;    ///< rotation and position velocity, as vector + quaternion
    ChCoordsys<Real> Csys_dtdt;  ///< rotation and position acceleration, as vector + quaternion
};

CH_CLASS_VERSION(ChFrameMoving<double>, 0)
CH_CLASS_VERSION(ChFrameMoving<float>, 0)

// MIXED ARGUMENT OPERATORS

// Mixing with ChFrame

/// The '*' operator that transforms a coordinate system of 'mixed' type:
///  frame_C = frame_A * frame_B;
/// where frame_A is  a ChFrameMoving
///       frame_B is  a ChFrame
/// Returns a ChFrame.
/// The effect is like applying the transformation frame_A to frame_B and get frame_C.
/// Also speeds and accelerations are transformed.
template <class Real>
ChFrame<Real> operator*(const ChFrameMoving<Real>& Fa, const ChFrame<Real>& Fb) {
    // note: it should be not needed: falling back to ChFrame = ChFrame * ChFrame
    // could be enough, but the compiler still needs this operator.. why?
    ChFrame<Real> res;
    Fa.ChFrame<Real>::TransformLocalToParent(Fb, res);
    return res;
}

/// The '*' operator that transforms a coordinate system of 'mixed' type:
///  frame_C = frame_A * frame_B;
/// where frame_A is  a ChFrame
///       frame_B is  a ChFrameMoving
/// Returns a ChFrameMoving.
/// The effect is like applying the transformation frame_A to frame_B and get frame_C.
/// Also speeds and accelerations are transformed.
/// Performance warning: this operator promotes frame_A to a temporary ChFrameMoving.
template <class Real>
ChFrameMoving<Real> operator*(const ChFrame<Real>& Fa, const ChFrameMoving<Real>& Fb) {
    ChFrameMoving<Real> res;
    ChFrameMoving<Real> Fam(Fa);
    Fam.TransformLocalToParent(Fb, res);
    return res;
}

/// The '>>' operator that transforms a coordinate system of 'mixed' type:
///  frame_C = frame_A >> frame_B;
/// where frame_A is  a ChFrameMoving
///       frame_B is  a ChFrame
/// Returns a ChFrameMoving.
/// The effect is like applying the transformation frame_B to frame_A and get frame_C.
/// Also speeds and accelerations are transformed.
/// Performance warning: this operator promotes frame_B to a temporary ChFrameMoving.
template <class Real>
ChFrameMoving<Real> operator>>(const ChFrameMoving<Real>& Fa, const ChFrame<Real>& Fb) {
    ChFrameMoving<Real> res;
    ChFrameMoving<Real> Fbm(Fb);
    Fbm.TransformLocalToParent(Fa, res);
    return res;
}

// Note: the missing
//   ChFrame = ChFrame >> ChFrameMoving
// is not necessary, just falls back to ChFrame = ChFrame >> ChFrame , see ChFrame.h

// Mixing with ChCoordsys

/// The '*' operator that transforms a coordinate system of 'mixed' type:
///  frame_C = frame_A * frame_B;
/// where frame_A is  a ChCoordsys
///       frame_B is  a ChFrameMoving
/// Returns a ChFrameMoving.
/// The effect is like applying the transformation frame_A to frame_B and get frame_C.
/// Also speeds and accelerations are transformed.
/// Performance warning: this operator promotes frame_A to a temporary ChFrameMoving.
template <class Real>
ChFrameMoving<Real> operator*(const ChCoordsys<Real>& Fa, const ChFrameMoving<Real>& Fb) {
    ChFrameMoving<Real> res;
    ChFrameMoving<Real> Fam(Fa);
    Fam.TransformLocalToParent(Fb, res);
    return res;
}

/// The '>>' operator that transforms a coordinate system of 'mixed' type:
///  frame_C = frame_A >> frame_B;
/// where frame_A is  a ChFrameMoving
///       frame_B is  a ChCoordsys
/// Returns a ChFrameMoving.
/// The effect is like applying the transformation frame_B to frame_A and get frame_C.
/// Also speeds and accelerations are transformed.
/// Performance warning: this operator promotes frame_B to a temporary ChFrameMoving.
template <class Real>
ChFrameMoving<Real> operator>>(const ChFrameMoving<Real>& Fa, const ChCoordsys<Real>& Fb) {
    ChFrameMoving<Real> res;
    ChFrameMoving<Real> Fbm(Fb);
    Fbm.TransformLocalToParent(Fa, res);
    return res;
}

// Note: the missing
//   ChCoordsys = ChCoordsys >> ChFrameMoving
// is not necessary, just falls back to ChCoordsys = ChCoordsys >> ChFrame , see ChFrame.h

// Note: the missing
//   ChCoordsys = ChFrameMoving * ChCoordsys
// is not necessary, just falls back to ChCoordsys = ChFrame * ChCoordsys  , see ChFrame.h

// Mixing with ChVector

/// The '*' operator that transforms 'mixed' types:
///  frame_C = vector_A * frame_B;
/// where vector_A is  a ChVector
///       frame_B  is  a ChFrame
/// Returns a ChFrameMoving.
/// The effect is like applying the translation vector_A to frame_B and get frame_C.
template <class Real>
ChFrameMoving<Real> operator*(const ChVector3<Real>& Fa, const ChFrameMoving<Real>& Fb) {
    ChFrameMoving<Real> res(Fb);
    res.Csys.pos += Fa;
    return res;
}

/// The '>>' operator that transforms 'mixed' types:
///  frame_C = frame_A >> vector_B;
/// where frame_A is  a ChFrame
///       frame_B is  a ChVector
/// Returns a ChFrameMoving.
/// The effect is like applying the translation vector_B to frame_A and get frame_C.
template <class Real>
ChFrameMoving<Real> operator>>(const ChFrameMoving<Real>& Fa, const ChVector3<Real>& Fb) {
    ChFrameMoving<Real> res(Fa);
    res.Csys.pos += Fb;
    return res;
}

// Note: the missing
//   ChVector = ChVector >> ChFrameMoving
// is not necessary, just falls back to ChVector = ChVector >> ChFrame , see ChFrame.h

// Note: the missing
//   ChVector = ChFrameMoving * ChVector
// is not necessary, just falls back to ChVector = ChFrame * ChVector  , see ChFrame.h

// Mixing with ChQuaternion

/// The '*' operator that transforms 'mixed' types:
///  frame_C = quat_A * frame_B;
/// where quat_A   is  a ChQuaternion
///       frame_B  is  a ChFrameMoving
/// Returns a ChFrameMoving.
/// The effect is like applying the rotation quat_A to frame_B and get frame_C.
/// Also speeds and accelerations are rotated.
/// Performance warning: this operator promotes quat_A to a temporary ChFrameMoving.
template <class Real>
ChFrameMoving<Real> operator*(const ChQuaternion<Real>& Fa, const ChFrameMoving<Real>& Fb) {
    ChFrameMoving<Real> res;
    ChFrameMoving<Real> Fam(VNULL, Fa);
    Fam.TransformLocalToParent(Fb, res);
    return res;
}

/// The '>>' operator that transforms 'mixed' types:
///  frame_C = frame_A >> quat_B;
/// where frame_A is  a ChFrameMoving
///       frame_B is  a ChQuaternion
/// Returns a ChFrameMoving.
/// The effect is like applying the rotation quat_B to frame_A and get frame_C.
/// Also speeds and accelerations are rotated.
/// Performance warning: this operator promotes quat_A to a temporary ChFrameMoving
template <class Real>
ChFrameMoving<Real> operator>>(const ChFrameMoving<Real>& Fa, const ChQuaternion<Real>& Fb) {
    ChFrameMoving<Real> res;
    ChFrameMoving<Real> Fbm(VNULL, Fb);
    Fbm.TransformLocalToParent(Fa, res);
    return res;
}

// Note: the missing
//   ChQuaternion = ChQuaternion >> ChFrameMoving
// is not necessary, just falls back to ChQuaternion = ChQuaternion >> ChFrame , see ChFrame.h

// Note: the missing
//   ChQuaternion = ChFrameMoving * ChQuaternion
// is not necessary, just falls back to ChQuaternion = ChFrame * ChQuaternion  , see ChFrame.h

}  // end namespace chrono

#endif
