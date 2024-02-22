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

#include "chrono/physics/ChLinkRackpinion.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkRackpinion)

ChLinkRackpinion::ChLinkRackpinion()
    : ChLinkMateGeneric(true, false, false, false, false, false),
      R(0.1),
      alpha(0),
      beta(0),
      phase(0),
      checkphase(false),
      a1(0),
      contact_pt(VNULL) {
    local_pinion.SetIdentity();
    local_rack.SetIdentity();
}

ChLinkRackpinion::ChLinkRackpinion(const ChLinkRackpinion& other) : ChLinkMateGeneric(other) {
    R = other.R;
    alpha = other.alpha;
    beta = other.beta;
    phase = other.phase;
    a1 = other.a1;
    checkphase = other.checkphase;

    contact_pt = other.contact_pt;
    local_pinion = other.local_pinion;
    local_rack = other.local_rack;
}

ChVector3d ChLinkRackpinion::GetAbsPinionDir() {
    if (this->Body1) {
        ChFrame<double> absframe;
        ((ChFrame<double>*)Body1)->TransformLocalToParent(local_pinion, absframe);
        return absframe.GetRotMat().GetAxisZ();
    } else
        return VECT_Z;
}

ChVector3d ChLinkRackpinion::GetAbsPinionPos() {
    if (this->Body1) {
        ChFrame<double> absframe;
        ((ChFrame<double>*)Body1)->TransformLocalToParent(local_pinion, absframe);
        return absframe.GetPos();
    } else
        return VNULL;
}

ChVector3d ChLinkRackpinion::GetAbsRackDir() {
    if (this->Body2) {
        ChFrame<double> absframe;
        ((ChFrame<double>*)Body2)->TransformLocalToParent(local_rack, absframe);
        return absframe.GetRotMat().GetAxisZ();
    } else
        return VECT_Z;
}

ChVector3d ChLinkRackpinion::GetAbsRackPos() {
    if (this->Body2) {
        ChFrame<double> absframe;
        ((ChFrame<double>*)Body2)->TransformLocalToParent(local_rack, absframe);
        return absframe.GetPos();
    } else
        return VNULL;
}

void ChLinkRackpinion::UpdateTime(double mytime) {
    // First, inherit to parent class
    ChLinkMateGeneric::UpdateTime(mytime);

    ChFrame<double> abs_pinion;
    ChFrame<double> abs_rack;

    ((ChFrame<double>*)Body1)->TransformLocalToParent(local_pinion, abs_pinion);
    ((ChFrame<double>*)Body2)->TransformLocalToParent(local_rack, abs_rack);

    ChVector3d abs_distpr = abs_pinion.GetPos() - abs_rack.GetPos();
    ChVector3d abs_Dpin = abs_pinion.GetRotMat().GetAxisZ();
    ChVector3d abs_Dx;
    ChVector3d abs_Dy;
    ChVector3d abs_Dz;
    abs_Dpin.DirToDxDyDz(abs_Dz, abs_Dx, abs_Dy,
                         abs_rack.GetRotMat().GetAxisX());  // with z as pinion shaft and x as suggested rack X dir

    /*
    std::cout << "abs_distpr " << abs_distpr << std::endl;
    std::cout << "abs_rack Xaxis()" << abs_rack.GetRotMat()->GetAxisX() << std::endl;
    std::cout << "abs_Dpin " << abs_Dpin << std::endl;
    std::cout << "abs_Dx " << abs_Dx << std::endl;
    */

    ChVector3d abs_Ro = abs_Dy * Vdot(abs_Dy, abs_distpr);

    if (Vdot(abs_Ro, abs_distpr) > 0)
        abs_Ro = -abs_Ro;

    ChVector3d abs_Dr = abs_Ro.GetNormalized();
    ChVector3d abs_R = abs_Dr * this->GetPinionRadius();
    this->contact_pt = abs_pinion.GetPos() + abs_R;

    // Absolute frame of link reference
    ChMatrix33<> ma1(abs_Dx, abs_Dy, abs_Dz);
    ChFrame<> abs_contact(this->contact_pt, ma1);

    ChMatrix33<> mrot;

    // rotate link frame on its Y because of beta
    mrot.SetFromCardanAnglesXYZ(ChVector3d(0, this->beta, 0));
    ChFrame<> mrotframe(VNULL, mrot);
    abs_contact.ConcatenatePostTransformation(mrotframe);  // or: abs_contact *= mrotframe;

    // rotate link frame on its Z because of alpha
    if (this->react_force.x() < 0)
        mrot.SetFromCardanAnglesXYZ(ChVector3d(0, 0, this->alpha));
    else
        mrot.SetFromCardanAnglesXYZ(ChVector3d(0, 0, -this->alpha));
    mrotframe.SetRot(mrot);
    abs_contact.ConcatenatePostTransformation(mrotframe);  // or: abs_contact *= mrotframe;

    // Set the link frame 'abs_contact' to relative frames to the two connected ChBodyFrame
    ((ChFrame<double>*)Body1)->TransformParentToLocal(abs_contact, this->frame1);
    ((ChFrame<double>*)Body2)->TransformParentToLocal(abs_contact, this->frame2);
}

void ChLinkRackpinion::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkRackpinion>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(R);
    archive_out << CHNVP(alpha);
    archive_out << CHNVP(beta);
    archive_out << CHNVP(phase);
    archive_out << CHNVP(checkphase);
    archive_out << CHNVP(a1);
    archive_out << CHNVP(local_pinion);
    archive_out << CHNVP(local_rack);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkRackpinion::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/ archive_in.VersionRead<ChLinkRackpinion>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(R);
    archive_in >> CHNVP(alpha);
    archive_in >> CHNVP(beta);
    archive_in >> CHNVP(phase);
    archive_in >> CHNVP(checkphase);
    archive_in >> CHNVP(a1);
    archive_in >> CHNVP(local_pinion);
    archive_in >> CHNVP(local_rack);
}

}  // end namespace chrono
