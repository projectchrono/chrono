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

#ifndef CHBODYAUXREF_H
#define CHBODYAUXREF_H

#include "chrono/physics/ChBody.h"

namespace chrono {

/// Class for rigid bodies defined with respect to a non-centroidal reference frame.
///
/// An auxiliary reference frame is added to the base ChBody class offering the flexibility of placing collision
/// and visual shapes, as well as markers, relative to a potentially more conveniant frame than relative to the
/// centroidal reference frame. The caller is responsible for specifying the location and orientation of the
/// centroidal frame at the body Center Of Mass (COM).
///
/// Additional information can be found in the @ref rigid_bodies manual page.
class ChApi ChBodyAuxRef : public ChBody {
  public:
    ChBodyAuxRef() : ChBody() {}
    ChBodyAuxRef(const ChBodyAuxRef& other);
    ~ChBodyAuxRef() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChBodyAuxRef* Clone() const override { return new ChBodyAuxRef(*this); }

    /// Set the auxiliary reference frame with respect to the absolute frame.
    /// This moves the entire body; the body COM is rigidly moved as well.
    void SetFrameRefToAbs(const ChFrame<>& frame);

    /// Get the auxiliary reference frame with respect to the absolute frame.
    /// Note that, in general, this is different from GetFrameCOMToAbs().
    virtual const ChFrameMoving<>& GetFrameRefToAbs() const override { return ref_to_abs; }

    /// Set the body COM frame with respect to the absolute frame.
    /// This moves the entire body; the body REF is rigidly moved as well.
    void SetFrameCOMToAbs(const ChFrame<>& frame);

    /// Set the COM frame with respect to the auxiliary reference frame.
    /// Note that this also moves the body absolute COM (the REF is fixed).
    /// The position of contained ChMarker objects, if any, is not changed with respect to the reference.
    void SetFrameCOMToRef(const ChFrame<>& frame);

    /// Get the COM frame with respect to the auxiliary reference frame.
    ChFrame<> GetFrameCOMToRef() const { return ref_to_com.GetInverse(); }

    /// Set the auxiliary reference frame with respect to the COM frame.
    /// Note that this does not move the body absolute COM (the COM is fixed).
    void SetFrameRefToCOM(const ChFrame<>& frame) { ref_to_com = frame; }

    /// Get the auxiliary reference frame with respect to the COM frame.
    const ChFrame<>& GetFrameRefToCOM() const { return ref_to_com; }

    /// Update all auxiliary data of the rigid body and of
    /// its children (markers, forces..)
    virtual void Update(double time, bool update_assets) override;

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  public:
    // These functions override the ChBodyFrame (ChFrame) functions for setting position and rotation.
    // In addition to setting the COM frame, they also must adjust ref_to_abs. Indeed, any of these 
    // functions move the entire body and as such the body REF frame must also be moved.

    virtual void SetPos(const ChVector3<>& pos) override;
    virtual void SetRot(const ChMatrix33<>& R) override;
    virtual void SetRot(const ChQuaternion<>& q) override;
    virtual void SetCoordsys(const ChCoordsys<>& C) override;
    virtual void SetCoordsys(const ChVector3<>& v, const ChQuaternion<>& q) override;

  private:
    ChFrameMoving<> ref_to_com;  ///< auxiliary REF location, relative to COM
    ChFrameMoving<> ref_to_abs;  ///< auxiliary REF location, relative to abs coords
};

CH_CLASS_VERSION(ChBodyAuxRef, 0)

}  // end namespace chrono

#endif
