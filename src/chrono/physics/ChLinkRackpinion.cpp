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

ChVector<> ChLinkRackpinion::GetAbsPinionDir() {
    if (this->Body1) {
        ChFrame<double> absframe;
        ((ChFrame<double>*)Body1)->TransformLocalToParent(local_pinion, absframe);
        return absframe.GetA().Get_A_Zaxis();
    } else
        return VECT_Z;
}

ChVector<> ChLinkRackpinion::GetAbsPinionPos() {
    if (this->Body1) {
        ChFrame<double> absframe;
        ((ChFrame<double>*)Body1)->TransformLocalToParent(local_pinion, absframe);
        return absframe.GetPos();
    } else
        return VNULL;
}

ChVector<> ChLinkRackpinion::GetAbsRackDir() {
    if (this->Body2) {
        ChFrame<double> absframe;
        ((ChFrame<double>*)Body2)->TransformLocalToParent(local_rack, absframe);
        return absframe.GetA().Get_A_Zaxis();
    } else
        return VECT_Z;
}

ChVector<> ChLinkRackpinion::GetAbsRackPos() {
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

    ChVector<> abs_distpr = abs_pinion.GetPos() - abs_rack.GetPos();
    ChVector<> abs_Dpin = abs_pinion.GetA().Get_A_Zaxis();
    ChVector<> abs_Dx;
    ChVector<> abs_Dy;
    ChVector<> abs_Dz;
    abs_Dpin.DirToDxDyDz(abs_Dz, abs_Dx, abs_Dy,
                         abs_rack.GetA().Get_A_Xaxis());  // with z as pinion shaft and x as suggested rack X dir

    /*
    GetLog() << "abs_distpr " << abs_distpr << "\n";
    GetLog() << "abs_rack Xaxis()" << abs_rack.GetA()->Get_A_Xaxis() << "\n";
    GetLog() << "abs_Dpin " << abs_Dpin << "\n";
    GetLog() << "abs_Dx " << abs_Dx << "\n";
    */

    ChVector<> abs_Ro = abs_Dy * Vdot(abs_Dy, abs_distpr);

    if (Vdot(abs_Ro, abs_distpr) > 0)
        abs_Ro = -abs_Ro;

    ChVector<> abs_Dr = abs_Ro.GetNormalized();
    ChVector<> abs_R = abs_Dr * this->GetPinionRadius();
    this->contact_pt = abs_pinion.GetPos() + abs_R;

    double runX = Vdot(abs_distpr, abs_Dx);

    // ChVector<> abs_runX    = Vdot( abs_distpr, abs_Dx );

    // Absolute frame of link reference
    ChMatrix33<> ma1;
    ma1.Set_A_axis(abs_Dx, abs_Dy, abs_Dz);
    ChFrame<> abs_contact(this->contact_pt, ma1);

    ChMatrix33<> mrot;

    // rotate link frame on its Y because of beta
    mrot.Set_A_Rxyz(ChVector<>(0, this->beta, 0));
    ChFrame<> mrotframe(VNULL, mrot);
    abs_contact.ConcatenatePostTransformation(mrotframe);  // or: abs_contact *= mrotframe;

    // rotate link frame on its Z because of alpha
    if (this->react_force.x() < 0)
        mrot.Set_A_Rxyz(ChVector<>(0, 0, this->alpha));
    else
        mrot.Set_A_Rxyz(ChVector<>(0, 0, -this->alpha));
    mrotframe.SetRot(mrot);
    abs_contact.ConcatenatePostTransformation(mrotframe);  // or: abs_contact *= mrotframe;

    // Set the link frame 'abs_contact' to relative frames to the two connected ChBodyFrame
    ((ChFrame<double>*)Body1)->TransformParentToLocal(abs_contact, this->frame1);
    ((ChFrame<double>*)Body2)->TransformParentToLocal(abs_contact, this->frame2);
}

void ChLinkRackpinion::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkRackpinion>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(R);
    marchive << CHNVP(alpha);
    marchive << CHNVP(beta);
    marchive << CHNVP(phase);
    marchive << CHNVP(checkphase);
    marchive << CHNVP(a1);
    marchive << CHNVP(local_pinion);
    marchive << CHNVP(local_rack);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkRackpinion::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkRackpinion>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(R);
    marchive >> CHNVP(alpha);
    marchive >> CHNVP(beta);
    marchive >> CHNVP(phase);
    marchive >> CHNVP(checkphase);
    marchive >> CHNVP(a1);
    marchive >> CHNVP(local_pinion);
    marchive >> CHNVP(local_rack);
}

}  // end namespace chrono
