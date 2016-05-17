//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHBODYAUXREF_H
#define CHBODYAUXREF_H

#include "physics/ChBody.h"

namespace chrono {

/// Class for rigid bodies with an auxiliary reference that can be used
/// for the collision shapes and marker positions; that reference can be
/// different from the center of gravity (COG), differently from the
/// base class ChBody where the COG is used also as reference.
/// Because of the auxilary reference, this type of rigid bodies can be
/// a bit less efficient thatn the ChBody simple class.
///
/// Further info at the @ref rigid_bodies  manual page.

class ChApi ChBodyAuxRef : public ChBody {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChBodyAuxRef, ChBody);

  private:
    //
    // DATA
    //
    ChFrameMoving<> auxref_to_cog;  // auxiliary REF location, relative to COG
    ChFrameMoving<> auxref_to_abs;  // for speeding up code: REF location relative to abs coords (needs Update() )

  public:
    //
    // CONSTRUCTORS
    //

    /// Nothing to be done, going with default values for the two frames
    ChBodyAuxRef(ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI)
        : ChBody(contact_method) {}
    ChBodyAuxRef(collision::ChCollisionModel* new_coll_model,
                 ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI)
        : ChBody(new_coll_model, contact_method) {}

    /// Destructor
    ~ChBodyAuxRef() {}

    /// Copy from another ChBodyAuxRef.
    /// NOTE: all settings of the body are copied, but the
    /// child hierarchy of ChForces and ChMarkers (if any) are NOT copied.
    void Copy(ChBodyAuxRef* source);

    /// Get the location of the auxiliary reference respect to COG.
    /// Viceversa, if you need to know the COG respect to auxiliary
    /// reference, use GetREF_to_COG().GetInverse()
    virtual const ChFrame<>& GetFrame_REF_to_COG() { return auxref_to_cog; };

    /// Set the location of the auxiliary reference respect to COG,
    /// and do not move the body absolute COG (the COG is fixed).
    virtual void SetFrame_REF_to_COG(const ChFrame<>& mloc) { auxref_to_cog = mloc; };

    /// Set the location of the COG respect to the auxiliary reference,
    /// and move the body absolute COG (the REF is fixed).
    /// Note! the position of contained ChMarker objects, if any, is
    /// not changed respect to the reference!
    virtual void SetFrame_COG_to_REF(const ChFrame<>& mloc);

    /// Set the absolute location of the auxiliary reference,
    /// moving the entire body. The body COG is rigidly moved as well.
    virtual void SetFrame_REF_to_abs(const ChFrame<>& mfra);

    /// Get the rigid body coordinate system that is used for
    /// defining the collision shapes and the ChMarker objects, respect
    /// to the absolute system.
    /// In this ChBodyAuxRef class, differently form ChBody, this is
    /// not necessarily the same reference of GetFrame_COG_to_abs().
    virtual const ChFrameMoving<>& GetFrame_REF_to_abs() const { return auxref_to_abs; }

    /// Update all auxiliary data of the rigid body and of
    /// its children (markers, forces..)
    virtual void Update(bool update_assets = true);

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);

};

}  // END_OF_NAMESPACE____

#endif
