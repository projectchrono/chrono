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
/// See @ref coordinate_systems manual page.
template <class Real = double>
class ChFrameMoving : public ChFrame<Real> {
  public:
    /// Construct from pos and rot (as a quaternion).
    explicit ChFrameMoving(const ChVector3<Real>& mv = ChVector3<Real>(0, 0, 0),
                           const ChQuaternion<Real>& mq = ChQuaternion<Real>(1, 0, 0, 0))
        : ChFrame<Real>(mv, mq) {
        m_csys_dt.rot = m_csys_dtdt.rot = ChQuaternion<Real>(0, 0, 0, 0);
    }

    /// Construct from pos and rotation (as a 3x3 matrix).
    ChFrameMoving(const ChVector3<Real>& mv, const ChMatrix33<Real>& ma) : ChFrame<Real>(mv, ma) {
        m_csys_dt.rot = m_csys_dtdt.rot = ChQuaternion<Real>(0, 0, 0, 0);
    }

    /// Construct from a coordsys.
    explicit ChFrameMoving(const ChCoordsys<Real>& mc) : ChFrame<Real>(mc) {
        m_csys_dt.rot = m_csys_dtdt.rot = ChQuaternion<Real>(0, 0, 0, 0);
    }

    /// Construct from a frame.
    explicit ChFrameMoving(const ChFrame<Real>& mc) : ChFrame<Real>(mc) {
        m_csys_dt.rot = m_csys_dtdt.rot = ChQuaternion<Real>(0, 0, 0, 0);
    }

    /// Copy constructor, build from another moving frame.
    ChFrameMoving(const ChFrameMoving<Real>& other)
        : ChFrame<Real>(other), m_csys_dt(other.m_csys_dt), m_csys_dtdt(other.m_csys_dtdt) {}

    /// Destructor.
    virtual ~ChFrameMoving() {}

    // OPERATORS OVERLOADING

    /// Assignment operator: copy from another moving frame.
    ChFrameMoving<Real>& operator=(const ChFrameMoving<Real>& other) {
        if (&other == this)
            return *this;
        ChFrame<Real>::operator=(other);
        m_csys_dt = other.m_csys_dt;
        m_csys_dtdt = other.m_csys_dtdt;
        return *this;
    }

    /// Assignment operator: copy from another frame.
    ChFrameMoving<Real>& operator=(const ChFrame<Real>& other) {
        if (&other == this)
            return *this;
        ChFrame<Real>::operator=(other);
        m_csys_dt.rot = m_csys_dtdt.rot = QNULL;
        return *this;
    }

    /// Returns true for identical frames.
    bool operator==(const ChFrameMoving<Real>& other) const { return Equals(other); }

    /// Returns true for different frames.
    bool operator!=(const ChFrameMoving<Real>& other) const { return !Equals(other); }

    /// Transform  another frame through this frame.
    /// If A is this frame and F another frame expressed in A, then G = F >> A is the frame F expresssed in the parent
    /// frame of A. For a sequence of transformations, i.e. a chain of coordinate systems, one can also write:
    ///   G = F >> F_3to2 >> F_2to1 >> F_1to0;
    /// i.e., just like done with a sequence of Denavitt-Hartemberg matrix multiplications (but reverting order).
    /// This operation is not commutative.
    /// Velocities and accelerations are also transformed.
    ChFrameMoving<Real> operator>>(const ChFrameMoving<Real>& F) const { return F.TransformLocalToParent(*this); }

    /// Transform another frame through this frame.
    /// If A is this frame and F another frame expressed in A, then G = A * F is the frame F expresssed in the parent
    /// frame of A. For a sequence of transformations, i.e. a chain of coordinate systems, one can also write:
    ///   G = F_1to0 * F_2to1 * F_3to2 * F;
    /// i.e., just like done with a sequence of Denavitt-Hartemberg matrix multiplications.
    /// This operation is not commutative.
    /// Velocities and accelerations are also transformed.
    ChFrameMoving<Real> operator*(const ChFrameMoving<Real>& F) const { return TransformLocalToParent(F); }

    /// Transform this frame by pre-multiplication with another frame.
    /// If A is this frame, then A >>= F means A' = F * A or A' = A >> F.
    ChFrameMoving<Real>& operator>>=(const ChFrameMoving<Real>& F) {
        ConcatenatePreTransformation(F);
        return *this;
    }

    /// Transform this frame by post-multiplication with another frame.
    /// If A is this frame, then A *= F means A' = A * F or A' = F >> A.
    ChFrameMoving<Real>& operator*=(const ChFrameMoving<Real>& F) {
        ConcatenatePostTransformation(F);
        return *this;
    }

    // Mixed type operators:

    /// Transform this frame by pre-multiplication with a given vector (translate frame).
    ChFrameMoving<Real>& operator>>=(const ChVector3<Real>& v) {
        this->m_csys.pos += v;
        return *this;
    }

    /// Transform this frame by pre-multiplication with a given quaternion (rotate frame).
    ChFrameMoving<Real>& operator>>=(const ChQuaternion<Real>& q) {
        ChFrameMoving<Real> F(VNULL, q);
        ConcatenatePreTransformation(F);
        return *this;
    }

    /// Transform this frame by pre-multiplication with a given coordinate system.
    ChFrameMoving<Real>& operator>>=(const ChCoordsys<Real>& C) {
        ChFrameMoving<Real> F(C);
        ConcatenatePreTransformation(F);
        return *this;
    }

    /// Transform this frame by pre-multiplication with another frame.
    ChFrameMoving<Real>& operator>>=(const ChFrame<Real>& F) {
        ChFrameMoving<Real> Fm(F);
        ConcatenatePreTransformation(Fm);
        return *this;
    }

    /// Return both rotation and translation velocities as a ChCoordsys object.
    const ChCoordsys<Real>& GetCoordsysDt() const { return m_csys_dt; }

    /// Return both rotation and translation accelerations as a ChCoordsys object.
    const ChCoordsys<Real>& GetCoordsysDt2() const { return m_csys_dtdt; }

    /// Return the linear velocity.
    const ChVector3<Real>& GetPosDt() const { return m_csys_dt.pos; }

    /// Return the linear velocity.
    const ChVector3<Real>& GetLinVel() const { return m_csys_dt.pos; }

    /// Return the linear acceleration.
    const ChVector3<Real>& GetPosDt2() const { return m_csys_dtdt.pos; }

    /// Return the linear acceleration.
    const ChVector3<Real>& GetLinAcc() const { return m_csys_dtdt.pos; }

    /// Return the rotation velocity as a quaternion.
    const ChQuaternion<Real>& GetRotDt() const { return m_csys_dt.rot; }

    /// Return the rotation acceleration as a quaternion.
    const ChQuaternion<Real>& GetRotDt2() const { return m_csys_dtdt.rot; }

    /// Compute the angular velocity (expressed in local coords).
    ChVector3<Real> GetAngVelLocal() const {
        ChGlMatrix34<> Gl(this->m_csys.rot);
        return Gl * m_csys_dt.rot;
    }

    /// Compute the actual angular velocity (expressed in parent coords).
    ChVector3<Real> GetAngVelParent() const {
        ChGwMatrix34<> Gw(this->m_csys.rot);
        return Gw * m_csys_dt.rot;
    }

    /// Compute the actual angular acceleration (expressed in local coords).
    ChVector3<Real> GetAngAccLocal() const {
        ChGlMatrix34<> Gl(this->m_csys.rot);
        return Gl * m_csys_dtdt.rot;
    }

    /// Compute the actual angular acceleration (expressed in parent coords).
    ChVector3<Real> GetAngAccParent() const {
        ChGwMatrix34<> Gw(this->m_csys.rot);
        return Gw * m_csys_dtdt.rot;
    }

    /// Set both linear and rotation velocities as a single ChCoordsys derivative.
    virtual void SetCoordsysDt(const ChCoordsys<Real>& csys_dt) { m_csys_dt = csys_dt; }

    /// Set the linear velocity.
    virtual void SetPosDt(const ChVector3<Real>& vel) { m_csys_dt.pos = vel; }

    /// Set the linear velocity.
    virtual void SetLinVel(const ChVector3<Real>& vel) { m_csys_dt.pos = vel; }

    /// Set the rotation velocity as a quaternion derivative.
    /// Note: the quaternion must satisfy: dot(q,q_dt)=0.
    virtual void SetRotDt(const ChQuaternion<Real>& q_dt) { m_csys_dt.rot = q_dt; }

    /// Set the rotation velocity from the given angular velocity (expressed in local coordinates).
    virtual void SetAngVelLocal(const ChVector3<Real>& w) {
        m_csys_dt.rot.Cross(this->m_csys.rot, ChQuaternion<Real>(0, w));
        m_csys_dt.rot *= (Real)0.5;  // q_dt = 1/2 * q * (0,w)
    }

    /// Set the rotation velocity from given angular velocity (expressed in parent coordinates).
    virtual void SetAngVelParent(const ChVector3<Real>& w) {
        m_csys_dt.rot.Cross(ChQuaternion<Real>(0, w), this->m_csys.rot);
        m_csys_dt.rot *= (Real)0.5;  // q_dt = 1/2 * (0,w) * q
    }

    /// Set the linear and rotation accelerations as a single ChCoordsys derivative.
    virtual void SetCoordsysDt2(const ChCoordsys<Real>& csys_dtdt) { m_csys_dtdt = csys_dtdt; }

    /// Set the linear acceleration.
    virtual void SetPosDt2(const ChVector3<Real>& acc) { m_csys_dtdt.pos = acc; }

    /// Set the linear acceleration.
    virtual void SetLinAcc(const ChVector3<Real>& acc) { m_csys_dtdt.pos = acc; }

    /// Set the rotation acceleration as a quaternion derivative.
    /// Note: the quaternion must satisfy: dot(q,q_dtdt)+dot(q_dt,q_dt)=0.
    virtual void SetRotDt2(const ChQuaternion<Real>& q_dtdt) { m_csys_dtdt.rot = q_dtdt; }

    /// Set the rotation acceleration from given angular acceleration (expressed in local coordinates).
    /// Note: even when the local angular acceleration is zero, this function should still be called because q_dtdt
    /// might be nonzero due to nonzero q_dt (in case of rotational motion).
    virtual void SetAngAccLocal(const ChVector3<Real>& a) {
        // q_dtdt = q_dt * q' * q_dt + 1/2 * q * (0,a)
        m_csys_dtdt.rot = (m_csys_dt.rot * this->m_csys.rot.GetConjugate() * m_csys_dt.rot) +
                          (this->m_csys.rot * ChQuaternion<Real>(0, a) * (Real)0.5);
    }

    /// Set the rotation acceleration from given angular acceleration (expressed in parent coordinates).
    virtual void SetAngAccParent(const ChVector3<Real>& a) {
        // q_dtdt = q_dt * q' * q_dt + 1/2 * (0,a) * q
        m_csys_dtdt.rot = (m_csys_dt.rot * this->m_csys.rot.GetConjugate() * m_csys_dt.rot) +
                          (ChQuaternion<Real>(0, a) * this->m_csys.rot * (Real)0.5);
    }

    /// Compute the time derivative of the rotation matrix.
    void ComputeRotMatDt(ChMatrix33<Real>& R_dt) const {
        //  [A_dt]=2[dFp/dt][Fm]'=2[Fp(q_dt)][Fm(q)]'
        ChFpMatrix34<Real> Fpdt(m_csys_dt.rot);
        ChFmMatrix34<Real> Fm(this->m_csys.rot);
        R_dt = 2 * Fpdt * Fm.transpose();
    }

    /// Compute the second time derivative of the rotation matrix.
    void ComputeRotMatDt2(ChMatrix33<Real>& R_dtdt) {
        //  [A_dtdt]=2[Fp(q_dtdt)][Fm(q)]'+2[Fp(q_dt)][Fm(q_dt)]'
        ChFpMatrix34<> Fpdtdt(m_csys_dtdt.rot);
        ChFmMatrix34<> Fm(this->m_csys.rot);
        ChFpMatrix34<> Fpdt(m_csys_dt.rot);
        ChFmMatrix34<> Fmdt(m_csys_dt.rot);
        R_dtdt = 2 * (Fpdtdt * Fm.transpose() + Fpdt * Fmdt.transpose());
    }

    /// Return the time derivative of the rotation matrix.
    ChMatrix33<Real> GetRotMatDt() {
        ChMatrix33<Real> res;
        ComputeRotMatDt(res);
        return res;
    }

    /// Return the second time derivative of the rotation matrix.
    ChMatrix33<Real> GetRotMatDt2() {
        ChMatrix33<Real> res;
        ComputeRotMatDt2(res);
        return res;
    }

    // FUNCTIONS TO TRANSFORM THE FRAME ITSELF

    /// Apply a transformation (rotation and translation) represented by another frame.
    /// This is equivalent to pre-multiply this frame by the other frame F:
    ///     this'= F * this
    ///  or
    ///     this' = this >> F
    void ConcatenatePreTransformation(const ChFrameMoving<Real>& F) {
        auto tmp = F.TransformLocalToParent(*this);
        *this = tmp;
    }

    /// Apply a transformation (rotation and translation) represented by another frame F in local coordinate.
    /// This is equivalent to post-multiply this frame by the other frame F:
    ///    this'= this * F
    ///  or
    ///    this'= F >> this
    void ConcatenatePostTransformation(const ChFrameMoving<Real>& F) {
        auto tmp = this->TransformLocalToParent(F);
        *this = tmp;
    }

    // FUNCTIONS FOR COORDINATE TRANSFORMATIONS

    /// Return the velocity in the parent frame of a point fixed to this frame and expressed in local coordinates.
    ChVector3<Real> PointSpeedLocalToParent(const ChVector3<Real>& localpos) const {
        return m_csys_dt.pos +
               ((m_csys_dt.rot * ChQuaternion<Real>(0, localpos) * this->m_csys.rot.GetConjugate()).GetVector() * 2);
    }

    /// Return the velocity in the parent frame of a moving point, given the point location and velocity expressed in
    /// local coordinates.
    ChVector3<Real> PointSpeedLocalToParent(const ChVector3<Real>& localpos, const ChVector3<Real>& localspeed) const {
        return m_csys_dt.pos + this->m_rmat * localspeed +
               ((m_csys_dt.rot * ChQuaternion<Real>(0, localpos) * this->m_csys.rot.GetConjugate()).GetVector() * 2);
    }

    /// Return the acceleration in the parent frame of a point fixed to this frame and expressed in local coordinates.
    ///
    /// Note:
    /// - the first and second derivatives of pos and rot are assumed to have been assigned.
    /// - when the local angular acceleration is zero, it's still necessary to call SetAngAccLocal(VNULL) because
    ///   q_dtdt may be nonzero due to nonzero q_dt in case of rotational motion.
    ChVector3<Real> PointAccelerationLocalToParent(const ChVector3<Real>& localpos) const {
        return m_csys_dtdt.pos +
               ((m_csys_dtdt.rot * ChQuaternion<Real>(0, localpos) * this->m_csys.rot.GetConjugate()).GetVector() * 2) +
               ((m_csys_dt.rot * ChQuaternion<Real>(0, localpos) * m_csys_dt.rot.GetConjugate()).GetVector() * 2);
    }

    /// Return the acceleration in the parent frame of a moving point, given the point location, velocity, and
    /// acceleration expressed in local coordinates.
    ChVector3<Real> PointAccelerationLocalToParent(const ChVector3<Real>& localpos,
                                                   const ChVector3<Real>& localspeed,
                                                   const ChVector3<Real>& localacc) const {
        return m_csys_dtdt.pos + this->m_rmat * localacc +
               ((m_csys_dtdt.rot * ChQuaternion<Real>(0, localpos) * this->m_csys.rot.GetConjugate()).GetVector() * 2) +
               ((m_csys_dt.rot * ChQuaternion<Real>(0, localpos) * m_csys_dt.rot.GetConjugate()).GetVector() * 2) +
               ((m_csys_dt.rot * ChQuaternion<Real>(0, localspeed) * this->m_csys.rot.GetConjugate()).GetVector() * 4);
    }

    /// Return the velocity of a point expressed in this frame, given the point location and velocity in the parent
    /// frame.
    ChVector3<Real> PointSpeedParentToLocal(const ChVector3<Real>& parentpos,
                                            const ChVector3<Real>& parentspeed) const {
        ChVector3<Real> localpos = ChFrame<Real>::TransformPointParentToLocal(parentpos);
        return this->m_rmat.transpose() *
               (parentspeed - m_csys_dt.pos -
                ((m_csys_dt.rot * ChQuaternion<Real>(0, localpos) * this->m_csys.rot.GetConjugate()).GetVector() * 2));
    }

    /// Return the acceleration of a point expressed in this frame, given the point location, velocity, and acceleration
    /// in the parent frame.
    ChVector3<Real> PointAccelerationParentToLocal(const ChVector3<Real>& parentpos,
                                                   const ChVector3<Real>& parentspeed,
                                                   const ChVector3<Real>& parentacc) const {
        ChVector3<Real> localpos = ChFrame<Real>::TransformPointParentToLocal(parentpos);
        ChVector3<Real> localspeed = PointSpeedParentToLocal(parentpos, parentspeed);
        return this->m_rmat.transpose() *
               (parentacc - m_csys_dtdt.pos -
                (m_csys_dtdt.rot * ChQuaternion<Real>(0, localpos) * this->m_csys.rot.GetConjugate()).GetVector() * 2 -
                (m_csys_dt.rot * ChQuaternion<Real>(0, localpos) * m_csys_dt.rot.GetConjugate()).GetVector() * 2 -
                (m_csys_dt.rot * ChQuaternion<Real>(0, localspeed) * this->m_csys.rot.GetConjugate()).GetVector() * 4);
    }

    /// Transform a moving frame from 'this' local coordinate system to parent frame coordinate system.
    ChFrameMoving<Real> TransformLocalToParent(const ChFrameMoving<Real>& F) const {
        ChFrameMoving<Real> Fp(this->TransformPointLocalToParent(F.m_csys.pos), this->m_csys.rot * F.m_csys.rot);

        // pos_dt
        Fp.m_csys_dt.pos = PointSpeedLocalToParent(F.m_csys.pos, F.m_csys_dt.pos);

        // pos_dtdt
        Fp.m_csys_dtdt.pos = PointAccelerationLocalToParent(F.m_csys.pos, F.m_csys_dt.pos, F.m_csys_dtdt.pos);

        // rot_dt
        Fp.m_csys_dt.rot = this->m_csys_dt.rot * F.m_csys.rot + this->m_csys.rot * F.m_csys_dt.rot;

        // rot_dtdt
        Fp.m_csys_dtdt.rot = this->m_csys_dtdt.rot * F.m_csys.rot + (this->m_csys_dt.rot * F.m_csys_dt.rot) * 2 +
                             this->m_csys.rot * F.m_csys_dtdt.rot;

        return Fp;
    }

    /// Transform a moving frame from the parent coordinate system to 'this' local frame coordinate system.
    ChFrameMoving<Real> TransformParentToLocal(const ChFrameMoving<Real>& F) const {
        ChFrameMoving<Real> Fl(this->TransformPointParentToLocal(F.m_csys.pos),
                               this->m_csys.rot.GetConjugate() * F.m_csys.rot);

        // pos_dt
        Fl.m_csys_dt.pos = PointSpeedParentToLocal(F.m_csys.pos, F.m_csys_dt.pos);

        // pos_dtdt
        Fl.m_csys_dtdt.pos = PointAccelerationParentToLocal(F.m_csys.pos, F.m_csys_dt.pos, F.m_csys_dtdt.pos);

        // rot_dt
        Fl.m_csys_dt.rot = this->m_csys.rot.GetConjugate() * (F.m_csys_dt.rot - this->m_csys_dt.rot * Fl.m_csys.rot);

        // rot_dtdt
        Fl.m_csys_dtdt.rot =
            this->m_csys.rot.GetConjugate() *
            (F.m_csys_dtdt.rot - this->m_csys_dtdt.rot * Fl.m_csys.rot - (this->m_csys_dt.rot * Fl.m_csys_dt.rot) * 2);

        return Fl;
    }

    // OTHER FUNCTIONS

    /// Returns true if this transform is identical to the other transform.
    bool Equals(const ChFrameMoving<Real>& other) const {
        return this->m_csys.Equals(other.m_csys) && m_csys_dt.Equals(other.m_csys_dt) &&
               m_csys_dtdt.Equals(other.m_csys_dtdt);
    }

    /// Returns true if this transform is equal to the other transform, within a tolerance 'tol'.
    bool Equals(const ChFrameMoving<Real>& other, Real tol) const {
        return this->m_csys.Equals(other.m_csys, tol) && m_csys_dt.Equals(other.m_csys_dt, tol) &&
               m_csys_dtdt.Equals(other.m_csys_dtdt, tol);
    }

    /// Invert in place.
    /// If w=A*v, after A.Invert() we have v=A*w;
    virtual void Invert() override {
        ChFrameMoving<Real> tmp = *this;
        ChFrameMoving<Real> unit(VNULL, QUNIT);
        *this = tmp.TransformParentToLocal(unit);
    }

    /// Return the inverse transform.
    ChFrameMoving<Real> GetInverse() const {
        ChFrameMoving<Real> tmp(*this);
        tmp.Invert();
        return tmp;
    }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override {
        // version number
        archive_out.VersionWrite<ChFrameMoving>();

        // serialize parent class
        ChFrame<Real>::ArchiveOut(archive_out);

        // serialize all member data
        archive_out << CHNVP(m_csys_dt);
        archive_out << CHNVP(m_csys_dtdt);
    }

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override {
        // version number
        /*int version =*/archive_in.VersionRead<ChFrameMoving>();

        // deserialize parent class
        ChFrame<Real>::ArchiveIn(archive_in);

        // stream in all member data
        archive_in >> CHNVP(m_csys_dt);
        archive_in >> CHNVP(m_csys_dtdt);
    }

  protected:
    ChCoordsys<Real> m_csys_dt;    ///< rotation and position velocity, as vector + quaternion
    ChCoordsys<Real> m_csys_dtdt;  ///< rotation and position acceleration, as vector + quaternion

    friend class FmuChronoComponentBase;
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
    return Fa.ChFrame<Real>::TransformLocalToParent(Fb);
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
    ChFrameMoving<Real> Fam(Fa);
    return Fam.TransformLocalToParent(Fb);
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
    ChFrameMoving<Real> Fbm(Fb);
    return Fbm.TransformLocalToParent(Fa);
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
ChFrameMoving<Real> operator*(const ChCoordsys<Real>& ca, const ChFrameMoving<Real>& Fb) {
    ChFrameMoving<Real> Fam(ca);
    return Fam.TransformLocalToParent(Fb);
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
ChFrameMoving<Real> operator>>(const ChFrameMoving<Real>& Fa, const ChCoordsys<Real>& cb) {
    ChFrameMoving<Real> Fbm(cb);
    return Fbm.TransformLocalToParent(Fa);
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
ChFrameMoving<Real> operator*(const ChVector3<Real>& va, const ChFrameMoving<Real>& Fb) {
    ChFrameMoving<Real> res(Fb);
    res.m_csys.pos += va;
    return res;
}

/// The '>>' operator that transforms 'mixed' types:
///  frame_C = frame_A >> vector_B;
/// where frame_A is  a ChFrame
///       frame_B is  a ChVector
/// Returns a ChFrameMoving.
/// The effect is like applying the translation vector_B to frame_A and get frame_C.
template <class Real>
ChFrameMoving<Real> operator>>(const ChFrameMoving<Real>& Fa, const ChVector3<Real>& vb) {
    ChFrameMoving<Real> res(Fa);
    res.m_csys.pos += vb;
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
ChFrameMoving<Real> operator*(const ChQuaternion<Real>& qa, const ChFrameMoving<Real>& Fb) {
    ChFrameMoving<Real> Fam(VNULL, qa);
    return Fam.TransformLocalToParent(Fb);
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
ChFrameMoving<Real> operator>>(const ChFrameMoving<Real>& Fa, const ChQuaternion<Real>& qb) {
    ChFrameMoving<Real> Fbm(VNULL, qb);
    return Fbm.TransformLocalToParent(Fa);
}

// Note: the missing
//   ChQuaternion = ChQuaternion >> ChFrameMoving
// is not necessary, just falls back to ChQuaternion = ChQuaternion >> ChFrame , see ChFrame.h

// Note: the missing
//   ChQuaternion = ChFrameMoving * ChQuaternion
// is not necessary, just falls back to ChQuaternion = ChFrame * ChQuaternion  , see ChFrame.h

}  // end namespace chrono

#endif
