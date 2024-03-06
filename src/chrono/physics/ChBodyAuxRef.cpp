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

#include "chrono/physics/ChBodyAuxRef.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChBodyAuxRef)
CH_UPCASTING(ChBodyAuxRef, ChBody)

ChBodyAuxRef::ChBodyAuxRef(const ChBodyAuxRef& other) : ChBody(other) {
    auxref_to_cog = other.auxref_to_cog;
    auxref_to_abs = other.auxref_to_abs;
}

void ChBodyAuxRef::SetFrame_COG_to_REF(const ChFrame<>& cog_to_ref) {
    ChFrameMoving<> old_cog_to_abs = *this;

    ChFrameMoving<> ref_to_abs = this->TransformLocalToParent(this->auxref_to_cog);

    ChFrameMoving<> new_cog_to_abs = ref_to_abs.TransformLocalToParent(ChFrameMoving<>(cog_to_ref));

    this->SetCoordsys(new_cog_to_abs.GetCoordsys());
    this->SetCoordsysDer(new_cog_to_abs.GetCoordsysDer());
    this->SetCoordsysDer2(new_cog_to_abs.GetCoordsysDer2());

    this->auxref_to_cog = cog_to_ref.GetInverse();
    this->auxref_to_abs = this->TransformLocalToParent(this->auxref_to_cog);

    // Restore marker/forces positions, keeping unchanged respect to aux ref.
    ChFrameMoving<> cog_oldnew = old_cog_to_abs >> new_cog_to_abs.GetInverse();

    for (auto& marker : marklist) {
        marker->ConcatenatePreTransformation(cog_oldnew);
        marker->Update(ChTime);
    }

    // Forces: ?? to do...
    /*
    HIER_FORCE_INIT
    while (HIER_FORCE_NOSTOP)
    {
        FORCEpointer->
        FORCEpointer->Update (mytime);

        HIER_FORCE_NEXT
    }
    */
}

void ChBodyAuxRef::SetFrame_REF_to_abs(const ChFrame<>& ref_to_abs) {
    auto cog_to_abs = ref_to_abs.TransformLocalToParent(this->auxref_to_cog.GetInverse());
    this->SetCoordsys(cog_to_abs.GetCoordsys());
    this->auxref_to_abs = ref_to_abs;
}

void ChBodyAuxRef::Update(bool update_assets) {
    // update parent class
    ChBody::Update(update_assets);

    // update own data
    this->auxref_to_abs = this->TransformLocalToParent(this->auxref_to_cog);
}

//////// FILE I/O

void ChBodyAuxRef::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChBodyAuxRef>();

    // serialize parent class
    ChBody::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(auxref_to_cog);
    archive_out << CHNVP(auxref_to_abs);
}

/// Method to allow de serialization of transient data from archives.
void ChBodyAuxRef::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/ archive_in.VersionRead<ChBodyAuxRef>();

    // deserialize parent class
    ChBody::ArchiveIn(archive_in);

    // stream in all member data:
    archive_in >> CHNVP(auxref_to_cog);
    archive_in >> CHNVP(auxref_to_abs);

}

}  // end namespace chrono
