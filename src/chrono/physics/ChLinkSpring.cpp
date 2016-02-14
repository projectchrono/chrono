//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include "physics/ChLinkSpring.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkSpring> a_registration_ChLinkSpring;

ChLinkSpring::ChLinkSpring() {
    spr_restlength = 0;
    spr_k = 100;
    spr_r = 5;
    spr_f = 0;

    mod_f_time = std::make_shared<ChFunction_Const>(1);
    mod_k_d = std::make_shared<ChFunction_Const>(1);
    mod_k_speed = std::make_shared<ChFunction_Const>(1);
    mod_r_d = std::make_shared<ChFunction_Const>(1);
    mod_r_speed = std::make_shared<ChFunction_Const>(1);

    spr_react = 0.0;
}

ChLinkSpring::~ChLinkSpring() {
}

void ChLinkSpring::Copy(ChLinkSpring* source) {
    // first copy the parent class data...
    ChLinkMarkers::Copy(source);

    // copy custom data:
    spr_restlength = source->spr_restlength;
    spr_f = source->spr_f;
    spr_k = source->spr_k;
    spr_r = source->spr_r;
    spr_react = source->spr_react;

    mod_f_time = std::shared_ptr<ChFunction>(source->mod_f_time->new_Duplicate());
    mod_k_d = std::shared_ptr<ChFunction>(source->mod_k_d->new_Duplicate());
    mod_k_speed = std::shared_ptr<ChFunction>(source->mod_k_speed->new_Duplicate());
    mod_r_d = std::shared_ptr<ChFunction>(source->mod_r_d->new_Duplicate());
    mod_r_speed = std::shared_ptr<ChFunction>(source->mod_r_speed->new_Duplicate());
}

ChLink* ChLinkSpring::new_Duplicate() {
    ChLinkSpring* m_l;
    m_l = new ChLinkSpring;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}

void ChLinkSpring::Initialize(
    std::shared_ptr<ChBody> mbody1,  // first body to link
    std::shared_ptr<ChBody> mbody2,  // second body to link
    bool pos_are_relative,       // true: following posit. are considered relative to bodies. false: pos.are absolute
    ChVector<> mpos1,            // position of distance endpoint, for 1st body (rel. or abs., see flag above)
    ChVector<> mpos2,            // position of distance endpoint, for 2nd body (rel. or abs., see flag above)
    bool auto_rest_length,       // if true, initializes the imposed distance as the distance between mpos1 and mpos2
    double mrest_length          // imposed distance (no need to define, if auto_distance=true.)
    ) {
    // First, initialize as all constraint with markers.
    // In this case, create the two markers also!.
    ChLinkMarkers::Initialize(mbody1, mbody2, CSYSNORM);

    if (pos_are_relative) {
        marker1->Impose_Rel_Coord(ChCoordsys<>(mpos1, QUNIT));
        marker2->Impose_Rel_Coord(ChCoordsys<>(mpos2, QUNIT));
    } else {
        marker1->Impose_Abs_Coord(ChCoordsys<>(mpos1, QUNIT));
        marker2->Impose_Abs_Coord(ChCoordsys<>(mpos2, QUNIT));
    }

    ChVector<> AbsDist = marker1->GetAbsCoord().pos - marker2->GetAbsCoord().pos;
    dist = AbsDist.Length();

    spr_restlength = auto_rest_length ? dist : mrest_length;
}

void ChLinkSpring::UpdateForces(double mytime) {
    // Inherit force computation:
    // also base class can add its own forces.
    ChLinkMarkers::UpdateForces(mytime);

    spr_react = 0.0;
    Vector m_force;
    double deform = Get_SpringDeform();

    spr_react = spr_f * mod_f_time->Get_y(ChTime);
    spr_react -= (spr_k * mod_k_d->Get_y(deform) * mod_k_speed->Get_y(dist_dt)) * (deform);
    spr_react -= (spr_r * mod_r_d->Get_y(deform) * mod_r_speed->Get_y(dist_dt)) * (dist_dt);

    m_force = Vmul(Vnorm(relM.pos), spr_react);

    C_force = Vadd(C_force, m_force);
}


void ChLinkSpring::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChLinkMarkers::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(spr_restlength);
    marchive << CHNVP(spr_f);
    marchive << CHNVP(spr_k);
    marchive << CHNVP(spr_r);
    marchive << CHNVP(mod_f_time);
    marchive << CHNVP(mod_k_d);
    marchive << CHNVP(mod_k_speed);
    marchive << CHNVP(mod_r_d);
    marchive << CHNVP(mod_r_speed);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkSpring::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class
    ChLinkMarkers::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(spr_restlength);
    marchive >> CHNVP(spr_f);
    marchive >> CHNVP(spr_k);
    marchive >> CHNVP(spr_r);
    marchive >> CHNVP(mod_f_time);
    marchive >> CHNVP(mod_k_d);
    marchive >> CHNVP(mod_k_speed);
    marchive >> CHNVP(mod_r_d);
    marchive >> CHNVP(mod_r_speed);
}


}  // END_OF_NAMESPACE____
