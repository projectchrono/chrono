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

#include "chrono/physics/ChLinkSpring.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkSpring)

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

ChLinkSpring::ChLinkSpring(const ChLinkSpring& other) : ChLinkMarkers(other) {
    spr_restlength = other.spr_restlength;
    spr_f = other.spr_f;
    spr_k = other.spr_k;
    spr_r = other.spr_r;
    spr_react = other.spr_react;

    mod_f_time = std::shared_ptr<ChFunction>(other.mod_f_time->Clone());
    mod_k_d = std::shared_ptr<ChFunction>(other.mod_k_d->Clone());
    mod_k_speed = std::shared_ptr<ChFunction>(other.mod_k_speed->Clone());
    mod_r_d = std::shared_ptr<ChFunction>(other.mod_r_d->Clone());
    mod_r_speed = std::shared_ptr<ChFunction>(other.mod_r_speed->Clone());
}

void ChLinkSpring::Initialize(std::shared_ptr<ChBody> mbody1,
                              std::shared_ptr<ChBody> mbody2,
                              bool pos_are_relative,
                              ChVector<> mpos1,
                              ChVector<> mpos2,
                              bool auto_rest_length,
                              double mrest_length) {
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

void ChLinkSpring::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkSpring>();

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
void ChLinkSpring::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkSpring>();

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

}  // end namespace chrono
