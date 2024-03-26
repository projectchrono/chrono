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

/// Class for Rigid Bodies with an auxiliary Reference Frame.
///
/// An auxiliary reference frame is added to the base ChBody class thus offering the flexibility of placing
/// collision and visual shapes, as well as ChMarkers, in a different position with respect to the Center of Mass.
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
    /// This moves the entire body; the body COG is rigidly moved as well.
    void SetFrameRefToAbs(const ChFrame<>& frame);

    /// Get the auxiliary reference frame with respect to the absolute frame.
    /// Note that, in general, this is different from GetFrameCOMToAbs().
    virtual const ChFrameMoving<>& GetFrameRefToAbs() const override { return ref_to_abs; }

    /// Set the COG frame with respect to the auxiliary reference frame.
    /// Note that this also moves the body absolute COG (the REF is fixed).
    /// The position of contained ChMarker objects, if any, is not changed with respect
    /// to the reference.
    void SetFrameCOMToRef(const ChFrame<>& frame);

    /// Get the COG frame with respect to the auxiliary reference frame.
    ChFrame<> GetFrameCOMToRef() const { return ref_to_com.GetInverse(); }

    /// Set the auxiliary reference frame with respect to the COG frame.
    /// Note that this does not move the body absolute COG (the COG is fixed).
    void SetFrameRefToCOM(const ChFrame<>& frame) { ref_to_com = frame; }

    /// Get the auxiliary reference frame with respect to the COG frame.
    const ChFrame<>& GetFrameRefToCOM() const { return ref_to_com; }

    /// Update all auxiliary data of the rigid body and of
    /// its children (markers, forces..)
    virtual void Update(bool update_assets = true) override;

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    ChFrameMoving<> ref_to_com;  ///< auxiliary REF location, relative to COM
    ChFrameMoving<> ref_to_abs;  ///< auxiliary REF location, relative to abs coords
};

CH_CLASS_VERSION(ChBodyAuxRef, 0)

}  // end namespace chrono

#endif
