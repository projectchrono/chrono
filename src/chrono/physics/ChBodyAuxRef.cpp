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
    ref_to_com = other.ref_to_com;
    ref_to_abs = other.ref_to_abs;
}

void ChBodyAuxRef::SetFrameCOMToRef(const ChFrame<>& frame) {
    ChFrameMoving<> old_cog_to_abs = *this;

    ChFrameMoving<> ref_to_abs = TransformLocalToParent(ref_to_com);

    ChFrameMoving<> new_cog_to_abs = ref_to_abs.TransformLocalToParent(ChFrameMoving<>(frame));

    SetCoordsys(new_cog_to_abs.GetCoordsys());
    SetCoordsysDt(new_cog_to_abs.GetCoordsysDt());
    SetCoordsysDt2(new_cog_to_abs.GetCoordsysDt2());

    ref_to_com = frame.GetInverse();
    ref_to_abs = TransformLocalToParent(ref_to_com);

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

void ChBodyAuxRef::SetFrameRefToAbs(const ChFrame<>& frame) {
    auto cog_to_abs = frame.TransformLocalToParent(ref_to_com.GetInverse());
    SetCoordsys(cog_to_abs.GetCoordsys());
    ref_to_abs = frame;
}

void ChBodyAuxRef::Update(bool update_assets) {
    // update parent class
    ChBody::Update(update_assets);

    // update own data
    ref_to_abs = TransformLocalToParent(ref_to_com);
}

//////// FILE I/O

void ChBodyAuxRef::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChBodyAuxRef>();

    // serialize parent class
    ChBody::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(ref_to_com);
    archive_out << CHNVP(ref_to_abs);
}

/// Method to allow de serialization of transient data from archives.
void ChBodyAuxRef::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChBodyAuxRef>();

    // deserialize parent class
    ChBody::ArchiveIn(archive_in);

    // stream in all member data:
    archive_in >> CHNVP(ref_to_com);
    archive_in >> CHNVP(ref_to_abs);
}

}  // end namespace chrono
