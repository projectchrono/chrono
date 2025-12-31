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

#include "chrono/physics/ChContact.h"
#include "chrono/physics/ChContactContainer.h"

namespace chrono {

ChContact::ChContact(ChContactContainer* contact_container, ChContactable* obj_A, ChContactable* obj_B)
    : container(contact_container), objA(obj_A), objB(obj_B) {
    assert(contact_container);
    assert(obj_A);
    assert(obj_B);
}

void ChContact::Reset_cinfo(ChContactable* obj_A, ChContactable* obj_B, const ChCollisionInfo& cinfo) {
    assert(obj_A);
    assert(obj_B);

    objA = obj_A;
    objB = obj_B;

    p1 = cinfo.vpA;
    p2 = cinfo.vpB;
    normal = cinfo.vN;
    norm_dist = cinfo.distance;
    eff_radius = cinfo.eff_radius;

    // Contact plane
    contact_plane.SetFromAxisX(normal, VECT_Y);
}

ChCoordsys<> ChContact::GetContactCoords() const {
    ChCoordsys<> mcsys;
    ChQuaternion<> mrot = this->contact_plane.GetQuaternion();
    mcsys.rot.Set(mrot.e0(), mrot.e1(), mrot.e2(), mrot.e3());
    mcsys.pos = p2;
    return mcsys;
}

}  // end namespace chrono
