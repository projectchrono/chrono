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

#ifndef CHBODYAUXREF_H
#define CHBODYAUXREF_H

#include "chrono/physics/ChBody.h"

namespace chrono {

/// Class for rigid bodies with an auxiliary reference frame.
/// Unlike ChBody, where the COG frame is used as the reference frame, the
/// auxiliary reference frame of a ChBodyAuxRef can be different from its
/// COG frame.  This spcialization is provided for situations where it is more
/// convenient to specify collision shapes, visualization assets, and marker
/// positions with respect to a reference frame other than the COG frame.
/// Note that, because of the auxilary reference, this type of rigid bodies
/// can be slightly less efficient than the base ChBody object.
///
/// Additional information can be found in the @ref rigid_bodies manual page.

class ChApi ChBodyAuxRef : public ChBody {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChBodyAuxRef)

  private:
    ChFrameMoving<> auxref_to_cog;  ///< auxiliary REF location, relative to COG
    ChFrameMoving<> auxref_to_abs;  ///< auxiliary REF location, relative to abs coords (needs Update() )

  public:
    ChBodyAuxRef(ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI)
        : ChBody(contact_method) {}
    ChBodyAuxRef(std::shared_ptr<collision::ChCollisionModel> new_coll_model,
                 ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI)
        : ChBody(new_coll_model, contact_method) {}
    ChBodyAuxRef(const ChBodyAuxRef& other);
    ~ChBodyAuxRef() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChBodyAuxRef* Clone() const override { return new ChBodyAuxRef(*this); }

    /// Set the auxiliary reference frame with respect to the absolute frame.
    /// This moves the entire body; the body COG is rigidly moved as well.
    void SetFrame_REF_to_abs(const ChFrame<>& mfra);

    /// Get the auxiliary reference frame with respect to the absolute frame.
    /// Note that, in general, this is different from GetFrame_COG_to_abs().
    virtual const ChFrameMoving<>& GetFrame_REF_to_abs() const override { return auxref_to_abs; }

    /// Set the COG frame with respect to the auxiliary reference frame.
    /// Note that this also moves the body absolute COG (the REF is fixed).
    /// The position of contained ChMarker objects, if any, is not changed with respect
    /// to the reference.
    void SetFrame_COG_to_REF(const ChFrame<>& mloc);

    /// Get the COG frame with respect to the auxiliary reference frame.
    ChFrame<> GetFrame_COG_to_REF() const { return auxref_to_cog.GetInverse(); }

    /// Set the auxiliary reference frame with respect to the COG frame.
    /// Note that this does not move the body absolute COG (the COG is fixed).
    void SetFrame_REF_to_COG(const ChFrame<>& mloc) { auxref_to_cog = mloc; }

    /// Get the auxiliary reference frame with respect to the COG frame.
    const ChFrame<>& GetFrame_REF_to_COG() const { return auxref_to_cog; }

    /// Update all auxiliary data of the rigid body and of
    /// its children (markers, forces..)
    virtual void Update(bool update_assets = true) override;

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};


CH_CLASS_VERSION(ChBodyAuxRef,0)


}  // end namespace chrono

#endif
