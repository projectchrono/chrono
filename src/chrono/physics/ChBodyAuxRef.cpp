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
    ChFrameMoving<> old_com_to_abs = *this;

    ref_to_abs = TransformLocalToParent(ref_to_com);

    ChFrameMoving<> new_com_to_abs = ref_to_abs.TransformLocalToParent(ChFrameMoving<>(frame));

    ChBody::SetCoordsys(new_com_to_abs.GetCoordsys());
    ChBody::SetCoordsysDt(new_com_to_abs.GetCoordsysDt());
    ChBody::SetCoordsysDt2(new_com_to_abs.GetCoordsysDt2());

    ref_to_com = frame.GetInverse();
    ref_to_abs = TransformLocalToParent(ref_to_com);

    // Restore marker/forces positions, keeping unchanged respect to aux ref.
    ChFrameMoving<> com_oldnew = old_com_to_abs >> new_com_to_abs.GetInverse();

    for (auto& marker : marklist) {
        marker->ConcatenatePreTransformation(com_oldnew);
        marker->Update(GetChTime(), true);
    }
}

void ChBodyAuxRef::SetFrameRefToAbs(const ChFrame<>& frame) {
    auto cog_to_abs = frame.TransformLocalToParent(ref_to_com.GetInverse());
    ChBody::SetCoordsys(cog_to_abs.GetCoordsys());
    ref_to_abs = frame;
}

void ChBodyAuxRef::SetFrameCOMToAbs(const ChFrame<>& frame) {
    ChBody::SetCoordsys(frame.GetCoordsys());
    ref_to_abs = frame.TransformLocalToParent(ref_to_com);
}

void ChBodyAuxRef::Update(double time, bool update_assets) {
    // update parent class
    ChBody::Update(time, update_assets);

    // update own data
    ref_to_abs = TransformLocalToParent(ref_to_com);
}

// -----------------------------------------------------------------------------

void ChBodyAuxRef::SetPos(const ChVector3<>& pos) {
    SetFrameCOMToAbs(ChFramed(pos, GetRot()));
}

void ChBodyAuxRef::SetRot(const ChMatrix33<>& R) {
    SetFrameCOMToAbs(ChFramed(GetPos(), R));
}

void ChBodyAuxRef::SetRot(const ChQuaternion<>& q) {
    SetFrameCOMToAbs(ChFramed(GetPos(), q));
}

void ChBodyAuxRef::SetCoordsys(const ChCoordsys<>& C) {
    SetFrameCOMToAbs(ChFramed(C));
}

void ChBodyAuxRef::SetCoordsys(const ChVector3<>& v, const ChQuaternion<>& q) {
    SetFrameCOMToAbs(ChFramed(v, q));
}

// -----------------------------------------------------------------------------

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
