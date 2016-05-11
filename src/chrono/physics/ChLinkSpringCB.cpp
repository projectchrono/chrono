//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include "physics/ChLinkSpringCB.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and
// persistence
ChClassRegister<ChLinkSpringCB> a_registration_ChLinkSpringCB;

ChLinkSpringCB::ChLinkSpringCB() : m_rest_length(0), m_force(0), m_force_fun(NULL) {
}

ChLinkSpringCB::~ChLinkSpringCB() {
}

void ChLinkSpringCB::Copy(ChLinkSpringCB* source) {
    // first copy the parent class data
    ChLinkMarkers::Copy(source);

    // copy custom data:
    m_force_fun = source->m_force_fun;
    m_rest_length = source->m_rest_length;
    m_force = source->m_force;
}

ChLink* ChLinkSpringCB::new_Duplicate() {
    ChLinkSpringCB* m_l = new ChLinkSpringCB;
    m_l->Copy(this);
    return (m_l);
}

void ChLinkSpringCB::Initialize(
    std::shared_ptr<ChBody> body1,  // first body to link
    std::shared_ptr<ChBody> body2,  // second body to link
    bool pos_are_relative,      // true: following pos. are considered relative to bodies. false: pos. are absolute
    ChVector<> pos1,            // position of spring endpoint for 1st body (rel. or abs., see flag above)
    ChVector<> pos2,            // position of spring endpoint for 2nd body (rel. or abs., see flag above)
    bool auto_rest_length,      // if true, initializes the rest length as the distance between pos1 and pos2
    double rest_length          // rest length (no need to define if auto_rest_length=true.)
    ) {
    // First, initialize as all constraint with markers.
    // In this case, create the two markers also!.
    ChLinkMarkers::Initialize(body1, body2, CSYSNORM);

    if (pos_are_relative) {
        marker1->Impose_Rel_Coord(ChCoordsys<>(pos1, QUNIT));
        marker2->Impose_Rel_Coord(ChCoordsys<>(pos2, QUNIT));
    } else {
        marker1->Impose_Abs_Coord(ChCoordsys<>(pos1, QUNIT));
        marker2->Impose_Abs_Coord(ChCoordsys<>(pos2, QUNIT));
    }

    ChVector<> AbsDist = marker1->GetAbsCoord().pos - marker2->GetAbsCoord().pos;
    dist = AbsDist.Length();

    m_rest_length = auto_rest_length ? dist : rest_length;
}

void ChLinkSpringCB::UpdateForces(double time) {
    // Allow the base class to update itself (possibly adding its own forces)
    ChLinkMarkers::UpdateForces(time);

    // Invoke the provided functor to evaluate force
    m_force = m_force_fun ? (*m_force_fun)(time, m_rest_length, dist, dist_dt) : 0;

    // Add to existing force.
    C_force += m_force * relM.pos.GetNormalized();
}

void ChLinkSpringCB::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChLinkMarkers::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(m_rest_length);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkSpringCB::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class
    ChLinkMarkers::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(m_rest_length);
}


}  // END_OF_NAMESPACE____
