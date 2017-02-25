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

#ifndef CHFRAMEMOVING_H
#define CHFRAMEMOVING_H

#include "chrono/core/ChFrame.h"

namespace chrono {

/// ChFrameMoving: a class for coordinate systems in 3D space.
///
///  A 'frame' coordinate system has a translation and
/// a rotation respect to a 'parent' coordinate system,
/// usually the absolute (world) coordinates.
///
/// Differently from a simple ChCoordsys() object, however,
/// the ChFrame implements some optimizations because
/// each ChFrame stores also a 3x3 rotation matrix, which
/// can speed up coordinate transformations when a large
/// amount of vectors must be transformed by the same
/// coordinate frame.
///
/// Further info at the @ref coordinate_transformations manual page.

template <class Real = double>
class ChFrameMoving : public ChFrame<Real> {
  public:
    ChCoordsys<Real> coord_dt;    ///< Rotation and position speed, as vector+quaternion
    ChCoordsys<Real> coord_dtdt;  ///< Rotation and position acceleration, as vector+quaternion

    /// Construct from pos and rot (as a quaternion)
    explicit ChFrameMoving(const ChVector<Real>& mv = ChVector<Real>(0, 0, 0),
                           const ChQuaternion<Real>& mq = ChQuaternion<Real>(1, 0, 0, 0))
        : ChFrame<Real>(mv, mq) {
        coord_dt.rot = coord_dtdt.rot = ChQuaternion<Real>(0, 0, 0, 0);
    }

    /// Construct from pos and rotation (as a 3x3 matrix)
    ChFrameMoving(const ChVector<Real>& mv, const ChMatrix33<Real>& ma) : ChFrame<Real>(mv, ma) {
        coord_dt.rot = coord_dtdt.rot = ChQuaternion<Real>(0, 0, 0, 0);
    }

    /// Construct from a coordsys
    explicit ChFrameMoving(const ChCoordsys<Real>& mc) : ChFrame<Real>(mc) {
        coord_dt.rot = coord_dtdt.rot = ChQuaternion<Real>(0, 0, 0, 0);
    }

    /// Construct from a frame
    explicit ChFrameMoving(const ChFrame<Real>& mc) : ChFrame<Real>(mc) {
        coord_dt.rot = coord_dtdt.rot = ChQuaternion<Real>(0, 0, 0, 0);
    }

    /// Copy constructor, build from another moving frame
    ChFrameMoving(const ChFrameMoving<Real>& other)
        : ChFrame<Real>(other), coord_dt(other.coord_dt), coord_dtdt(other.coord_dtdt) {}

    /// Destructor
    virtual ~ChFrameMoving() {}

    //
    // OPERATORS OVERLOADING
    //

    /// Assignment operator: copy from another moving frame
    ChFrameMoving<Real>& operator=(const ChFrameMoving<Real>& other) {
        if (&other == this)
            return *this;
        ChFrame<Real>::operator=(other);
        coord_dt = other.coord_dt;
        coord_dtdt = other.coord_dtdt;
        return *this;
    }

    /// Assignment operator: copy from another frame
    ChFrameMoving<Real>& operator=(const ChFrame<Real>& other) {
        if (&other == this)
            return *this;
        ChFrame<Real>::operator=(other);
        coord_dt.rot = coord_dtdt.rot = QNULL;
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
    ChFrameMoving<Real>& operator>>=(const ChVector<Real>& D) {
        this->coord.pos += D;
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

    //
    // FUNCTIONS
    //

    // GET-FUNCTIONS

    /// Return both current rotation and translation speeds as
    /// a coordsystem object, with vector and quaternion
    ChCoordsys<Real>& GetCoord_dt() { return coord_dt; }
    const ChCoordsys<Real>& GetCoord_dt() const { return coord_dt; }

    /// Return both current rotation and translation accelerations as
    /// a coordsystem object, with vector and quaternion
    ChCoordsys<Real>& GetCoord_dtdt() { return coord_dtdt; }
    const ChCoordsys<Real>& GetCoord_dtdt() const { return coord_dtdt; }

    /// Return the current speed as a 3d vector
    ChVector<Real>& GetPos_dt() { return coord_dt.pos; }
    const ChVector<Real>& GetPos_dt() const { return coord_dt.pos; }

    /// Return the current acceleration as a 3d vector
    ChVector<Real>& GetPos_dtdt() { return coord_dtdt.pos; }
    const ChVector<Real>& GetPos_dtdt() const { return coord_dtdt.pos; }

    /// Return the current rotation speed as a quaternion
    ChQuaternion<Real>& GetRot_dt() { return coord_dt.rot; }
    const ChQuaternion<Real>& GetRot_dt() const { return coord_dt.rot; }

    /// Return the current rotation acceleration as a quaternion
    ChQuaternion<Real>& GetRot_dtdt() { return coord_dtdt.rot; }
    const ChQuaternion<Real>& GetRot_dtdt() const { return coord_dtdt.rot; }

    /// Computes the actual angular speed (expressed in local coords)
    ChVector<Real> GetWvel_loc() const {
        ChMatrixNM<Real, 3, 4> tempGl;
        ChFrame<Real>::SetMatrix_Gl(tempGl, this->coord.rot);
        return tempGl.Matr34_x_Quat(coord_dt.rot);  // wl=[Gl]*q_dt
    }

    /// Computes the actual angular speed (expressed in parent coords)
    ChVector<Real> GetWvel_par() const {
        ChMatrixNM<Real, 3, 4> tempGw;
        ChFrame<Real>::SetMatrix_Gw(tempGw, this->coord.rot);
        return tempGw.Matr34_x_Quat(coord_dt.rot);  // ww=[Gw]*q_dt
    }

    /// Computes the actual angular acceleration (expressed in local coords)
    ChVector<Real> GetWacc_loc() const {
        ChMatrixNM<Real, 3, 4> tempGl;
        ChFrame<Real>::SetMatrix_Gl(tempGl, this->coord.rot);
        return tempGl.Matr34_x_Quat(coord_dtdt.rot);  // al=[Gl]*q_dtdt
    }

    /// Computes the actual angular acceleration (expressed in parent coords)
    ChVector<Real> GetWacc_par() const {
        ChMatrixNM<Real, 3, 4> tempGw;
        ChFrame<Real>::SetMatrix_Gw(tempGw, this->coord.rot);
        return tempGw.Matr34_x_Quat(coord_dtdt.rot);  // aw=[Gw]*q_dtdt
    }

    // SET-FUNCTIONS

    /// Set both linear speed and rotation speed as a
    /// single ChCoordsys derivative.
    virtual void SetCoord_dt(const ChCoordsys<Real>& mcoord_dt) { coord_dt = mcoord_dt; }

    /// Set the linear speed
    virtual void SetPos_dt(const ChVector<Real>& mvel) { coord_dt.pos = mvel; }

    /// Set the rotation speed as a quaternion.
    /// Note: the quaternion must already satisfy  dot(q,q_dt)=0
    virtual void SetRot_dt(const ChQuaternion<Real>& mrot_dt) { coord_dt.rot = mrot_dt; }

    /// Set the rotation speed from given angular speed
    /// (expressed in local csys)
    virtual void SetWvel_loc(const ChVector<Real>& wl) {
        coord_dt.rot.Cross(this->coord.rot, ChQuaternion<Real>(0, wl));
        coord_dt.rot *= (Real)0.5;  // q_dt = 1/2 * q * (0,wl)
    }

    /// Set the rotation speed from given angular speed
    /// (expressed in parent csys)
    virtual void SetWvel_par(const ChVector<Real>& wp) {
        coord_dt.rot.Cross(ChQuaternion<Real>(0, wp), this->coord.rot);
        coord_dt.rot *= (Real)0.5;  // q_dt = 1/2 * (0,wp) * q
    }

    /// Set both linear acceleration and rotation acceleration as a
    /// single ChCoordsys derivative.
    virtual void SetCoord_dtdt(const ChCoordsys<Real>& mcoord_dtdt) { coord_dtdt = mcoord_dtdt; }

    /// Set the linear acceleration
    virtual void SetPos_dtdt(const ChVector<Real>& macc) { coord_dtdt.pos = macc; }

    /// Set the rotation acceleration as a quaternion derivative.
    /// Note: the quaternion must already satisfy  dot(q,q_dt)=0
    virtual void SetRot_dtdt(const ChQuaternion<Real>& mrot_dtdt) { coord_dtdt.rot = mrot_dtdt; }

    /// Set the rotation acceleration from given angular acceleration
    /// (expressed in local csys)
    virtual void SetWacc_loc(const ChVector<Real>& al) {
        // q_dtdt = q_dt * q' * q_dt + 1/2 * q * (0,al)
        coord_dtdt.rot = (coord_dt.rot % this->coord.rot.GetConjugate() % coord_dt.rot) +
                         (this->coord.rot % ChQuaternion<Real>(0, al) * (Real)0.5);
    }

    /// Set the rotation speed from given angular speed
    /// (expressed in parent csys)
    virtual void SetWacc_par(ChVector<Real>& ap) {
        // q_dtdt = q_dt * q' * q_dt + 1/2 * (0,ap) * q
        coord_dtdt.rot = (coord_dt.rot % this->coord.rot.GetConjugate() % coord_dt.rot) +
                         (ChQuaternion<Real>(0, ap) % this->coord.rot * (Real)0.5);
    }

    /// Computes the time derivative of rotation matrix, mAdt.
    void Compute_Adt(ChMatrix33<Real>& mA_dt) const {
        //  [A_dt]=2[dFp/dt][Fm]'=2[Fp(q_dt)][Fm(q)]'
        ChMatrixNM<Real, 3, 4> Fpdt;
        ChMatrixNM<Real, 3, 4> Fm;
        ChFrame<Real>::SetMatrix_Fp(Fpdt, coord_dt.rot);
        ChFrame<Real>::SetMatrix_Fm(Fm, this->coord.rot);
        mA_dt.MatrMultiplyT(Fpdt, Fm);
        mA_dt.MatrScale(2);
    }

    /// Computes the 2nd time derivative of rotation matrix, mAdtdt.
    void Compute_Adtdt(ChMatrix33<Real>& mA_dtdt) {
        //  [A_dtdt]=2[Fp(q_dtdt)][Fm(q)]'+2[Fp(q_dt)][Fm(q_dt)]'
        ChMatrixNM<Real, 3, 4> ma;
        ChMatrixNM<Real, 3, 4> mb;
        ChMatrix33<Real> mr;

        ChFrame<Real>::SetMatrix_Fp(ma, coord_dtdt.rot);
        ChFrame<Real>::SetMatrix_Fm(mb, this->coord.rot);
        mr.MatrMultiplyT(ma, mb);
        ChFrame<Real>::SetMatrix_Fp(ma, coord_dt.rot);
        ChFrame<Real>::SetMatrix_Fm(mb, coord_dt.rot);
        mA_dtdt.MatrMultiplyT(ma, mb);
        mA_dtdt.MatrInc(mr);
        mA_dtdt.MatrScale(2);
    }

    /// Computes and returns an Adt matrix (-note: prefer using
    /// Compute_Adt() directly for better performance)
    ChMatrix33<Real> GetA_dt() {
        ChMatrix33<Real> res;
        Compute_Adt(res);
        return res;
    }

    /// Computes and returns an Adt matrix (-note: prefer using
    /// Compute_Adtdt() directly for better performance)
    ChMatrix33<Real> GetA_dtdt() {
        ChMatrix33<Real> res;
        Compute_Adtdt(res);
        return res;
    }

    // FUNCTIONS TO TRANSFORM THE FRAME ITSELF

    /// Apply a transformation (rotation and translation) represented by
    /// another ChFrameMoving T. This is equivalent to pre-multiply this frame
    /// by the other frame T:   this'= T * this;  or this' = this >> T
    void ConcatenatePreTransformation(const ChFrameMoving<Real>& T) {
        ChFrameMoving<Real> res;
        T.TransformLocalToParent(*this, res);
        *this = res;
    }

    /// Apply a transformation (rotation and translation) represented by
    /// another ChFrameMoving T in local coordinate. This is equivalent to
    /// post-multiply this frame by the other frame T:   this'= this * T; or this' = T >> this
    void ConcatenatePostTransformation(const ChFrameMoving<Real>& T) {
        ChFrameMoving<Real> res;
        this->TransformLocalToParent(T, res);
        *this = res;
    }

    // FUNCTIONS FOR COORDINATE TRANSFORMATIONS

    /// Given the position of a point in local frame coords, and
    /// assuming it is sticky to frame, return the speed in parent coords.
    ChVector<Real> PointSpeedLocalToParent(const ChVector<Real>& localpos) const {
        return coord_dt.pos +
               ((coord_dt.rot % ChQuaternion<Real>(0, localpos) % this->coord.rot.GetConjugate()).GetVector() * 2);
    }

    /// Given the position localpos of a point in the local reference frame, assuming
    /// that the point moves in the local reference frame with localspeed,
    /// return the speed in the parent reference frame.
    ChVector<Real> PointSpeedLocalToParent(const ChVector<Real>& localpos, const ChVector<Real>& localspeed) const {
        return coord_dt.pos + this->Amatrix.Matr_x_Vect(localspeed) +
               ((coord_dt.rot % ChQuaternion<Real>(0, localpos) % this->coord.rot.GetConjugate()).GetVector() * 2);
    }

    /// Given the position of a point in local frame coords, and
    /// assuming it is sticky to frame, return the acceleration in parent coords.
    ChVector<Real> PointAccelerationLocalToParent(const ChVector<Real>& localpos) const {
        return coord_dtdt.pos +
               ((coord_dtdt.rot % ChQuaternion<Real>(0, localpos) % this->coord.rot.GetConjugate()).GetVector() * 2) +
               ((coord_dt.rot % ChQuaternion<Real>(0, localpos) % coord_dt.rot.GetConjugate()).GetVector() * 2);
    }

    /// Given the position of a point in local frame coords, and
    /// assuming it has a frame-relative speed localspeed and frame-relative
    /// acceleration localacc, return the acceleration in parent coords.
    ChVector<Real> PointAccelerationLocalToParent(const ChVector<Real>& localpos,
                                                  const ChVector<Real>& localspeed,
                                                  const ChVector<Real>& localacc) const {
        return coord_dtdt.pos + this->Amatrix.Matr_x_Vect(localacc) +
               ((coord_dtdt.rot % ChQuaternion<Real>(0, localpos) % this->coord.rot.GetConjugate()).GetVector() * 2) +
               ((coord_dt.rot % ChQuaternion<Real>(0, localpos) % coord_dt.rot.GetConjugate()).GetVector() * 2) +
               ((coord_dt.rot % ChQuaternion<Real>(0, localspeed) % this->coord.rot.GetConjugate()).GetVector() * 4);
    }

    /// Given the position of a point in parent frame coords, and
    /// assuming it has an absolute speed parentspeed,
    /// return the speed in local coords.
    ChVector<Real> PointSpeedParentToLocal(const ChVector<Real>& parentpos, const ChVector<Real>& parentspeed) const {
        ChVector<Real> localpos = ChFrame<Real>::TransformParentToLocal(parentpos);
        return this->Amatrix.MatrT_x_Vect(
            parentspeed - coord_dt.pos -
            ((coord_dt.rot % ChQuaternion<Real>(0, localpos) % this->coord.rot.GetConjugate()).GetVector() * 2));
    }

    /// Given the position of a point in parent frame coords, and
    /// assuming it has an absolute speed parentspeed and absolute
    /// acceleration parentacc, return the acceleration in local coords.
    ChVector<Real> PointAccelerationParentToLocal(const ChVector<Real>& parentpos,
                                                  const ChVector<Real>& parentspeed,
                                                  const ChVector<Real>& parentacc) const {
        ChVector<Real> localpos = ChFrame<Real>::TransformParentToLocal(parentpos);
        ChVector<Real> localspeed = PointSpeedParentToLocal(parentpos, parentspeed);
        return this->Amatrix.MatrT_x_Vect(
            parentacc - coord_dtdt.pos -
            (coord_dtdt.rot % ChQuaternion<Real>(0, localpos) % this->coord.rot.GetConjugate()).GetVector() * 2 -
            (coord_dt.rot % ChQuaternion<Real>(0, localpos) % coord_dt.rot.GetConjugate()).GetVector() * 2 -
            (coord_dt.rot % ChQuaternion<Real>(0, localspeed) % this->coord.rot.GetConjugate()).GetVector() * 4);
    }

    /// This function transforms a frame from 'this' local coordinate
    /// system to parent frame coordinate system, and also transforms the speed
    /// and acceleration of the frame.
    void TransformLocalToParent(
        const ChFrameMoving<Real>& local,  ///< frame to transform, given in local frame coordinates
        ChFrameMoving<Real>& parent        ///< transformed frame, in parent coordinates, will be stored here
        ) const {
        // pos & rot
        ChFrame<Real>::TransformLocalToParent(local, parent);

        // pos_dt
        parent.coord_dt.pos = PointSpeedLocalToParent(local.coord.pos, local.coord_dt.pos);

        // pos_dtdt
        parent.coord_dtdt.pos =
            PointAccelerationLocalToParent(local.coord.pos, local.coord_dt.pos, local.coord_dtdt.pos);

        // rot_dt
        parent.coord_dt.rot = coord_dt.rot % local.coord.rot + this->coord.rot % local.coord_dt.rot;

        // rot_dtdt
        parent.coord_dtdt.rot = coord_dtdt.rot % local.coord.rot + (coord_dt.rot % local.coord_dt.rot) * 2 +
                                this->coord.rot % local.coord_dtdt.rot;
    }

    /// This function transforms a frame from the parent coordinate
    /// system to 'this' local frame coordinate system.
    void TransformParentToLocal(
        const ChFrameMoving<Real>& parent,  ///< frame to transform, given in parent coordinates
        ChFrameMoving<Real>& local          ///< transformed frame, in local coordinates, will be stored here
        ) const {
        // pos & rot
        ChFrame<Real>::TransformParentToLocal(parent, local);

        // pos_dt
        local.coord_dt.pos = PointSpeedParentToLocal(parent.coord.pos, parent.coord_dt.pos);

        // pos_dtdt
        local.coord_dtdt.pos =
            PointAccelerationParentToLocal(parent.coord.pos, parent.coord_dt.pos, parent.coord_dtdt.pos);

        // rot_dt
        local.coord_dt.rot = this->coord.rot.GetConjugate() % (parent.coord_dt.rot - coord_dt.rot % local.coord.rot);

        // rot_dtdt
        local.coord_dtdt.rot =
            this->coord.rot.GetConjugate() %
            (parent.coord_dtdt.rot - coord_dtdt.rot % local.coord.rot - (coord_dt.rot % local.coord_dt.rot) * 2);
    }

    // OTHER FUNCTIONS

    /// Returns true if coordsys is identical to other coordsys
    bool Equals(const ChFrameMoving<Real>& other) const {
        return this->coord.Equals(other.coord) && coord_dt.Equals(other.coord_dt) &&
               coord_dtdt.Equals(other.coord_dtdt);
    }

    /// Returns true if coordsys is equal to other coordsys, within a tolerance 'tol'
    bool Equals(const ChFrameMoving<Real>& other, Real tol) const {
        return this->coord.Equals(other.coord, tol) && coord_dt.Equals(other.coord_dt, tol) &&
               coord_dtdt.Equals(other.coord_dtdt, tol);
    }

    /// The transformation (also for speeds, accelerations) is
    /// inverted in place.
    /// That is if w=A*v, then A.Invert();v=A*w;
    virtual void Invert() override {
        ChFrameMoving<Real> tmp;
        ChFrameMoving<Real> unit;
        tmp = *this;
        tmp.TransformParentToLocal(unit, *this);
    }

    ChFrameMoving<Real> GetInverse() const {
        ChFrameMoving<Real> tmp(*this);
        tmp.Invert();
        return tmp;
    }

    //
    // STREAMING
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChFrameMoving>();

        // serialize parent class
        ChFrame<Real>::ArchiveOUT(marchive);

        // serialize all member data
        marchive << CHNVP(coord_dt);
        marchive << CHNVP(coord_dtdt);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChFrameMoving>();

        // deserialize parent class
        ChFrame<Real>::ArchiveIN(marchive);

        // stream in all member data
        marchive >> CHNVP(coord_dt);
        marchive >> CHNVP(coord_dtdt);
    }
};

CH_CLASS_VERSION(ChFrameMoving<double>,0)
CH_CLASS_VERSION(ChFrameMoving<float>,0)

//
// MIXED ARGUMENT OPERATORS
//

// Mixing with ChFrame :

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

// Mixing with ChCoordsys :

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

// Mixing with ChVector :

/// The '*' operator that transforms 'mixed' types:
///  frame_C = vector_A * frame_B;
/// where vector_A is  a ChVector
///       frame_B  is  a ChFrame
/// Returns a ChFrameMoving.
/// The effect is like applying the translation vector_A to frame_B and get frame_C.
template <class Real>
ChFrameMoving<Real> operator*(const ChVector<Real>& Fa, const ChFrameMoving<Real>& Fb) {
    ChFrameMoving<Real> res(Fb);
    res.coord.pos += Fa;
    return res;
}

/// The '>>' operator that transforms 'mixed' types:
///  frame_C = frame_A >> vector_B;
/// where frame_A is  a ChFrame
///       frame_B is  a ChVector
/// Returns a ChFrameMoving.
/// The effect is like applying the translation vector_B to frame_A and get frame_C.
template <class Real>
ChFrameMoving<Real> operator>>(const ChFrameMoving<Real>& Fa, const ChVector<Real>& Fb) {
    ChFrameMoving<Real> res(Fa);
    res.coord.pos += Fb;
    return res;
}

// Note: the missing
//   ChVector = ChVector >> ChFrameMoving
// is not necessary, just falls back to ChVector = ChVector >> ChFrame , see ChFrame.h

// Note: the missing
//   ChVector = ChFrameMoving * ChVector
// is not necessary, just falls back to ChVector = ChFrame * ChVector  , see ChFrame.h

// Mixing with ChQuaternion :

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
