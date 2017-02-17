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
// Authors: Radu Serban
// =============================================================================

#include "chrono/physics/ChLinkSpringCB.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and
// persistence
CH_FACTORY_REGISTER(ChLinkSpringCB)

ChLinkSpringCB::ChLinkSpringCB() : m_rest_length(0), m_force(0), m_force_fun(NULL) {}

ChLinkSpringCB::ChLinkSpringCB(const ChLinkSpringCB& other) : ChLinkMarkers(other) {
    m_rest_length = other.m_rest_length;
    m_force = other.m_force;
    m_force_fun = other.m_force_fun;  //// do we need a deep copy?
}

void ChLinkSpringCB::Initialize(std::shared_ptr<ChBody> body1,
                                std::shared_ptr<ChBody> body2,
                                bool pos_are_relative,
                                ChVector<> pos1,
                                ChVector<> pos2,
                                bool auto_rest_length,
                                double rest_length) {
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

void ChLinkSpringCB::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkSpringCB>();

    // serialize parent class
    ChLinkMarkers::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(m_rest_length);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkSpringCB::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkSpringCB>();

    // deserialize parent class
    ChLinkMarkers::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(m_rest_length);
}

}  // end namespace chrono
