// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================

#include "chrono_parallel/collision/ChContactContainerParallel.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
//
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChParticlesClones.h"

namespace chrono {

using namespace collision;
using namespace geometry;

ChContactContainerParallel::ChContactContainerParallel(ChParallelDataManager* dc) : data_manager(dc) {
    contactlist_6_6.clear();
    n_added_6_6 = 0;
}

ChContactContainerParallel::ChContactContainerParallel(const ChContactContainerParallel& other)
    : ChContactContainer(other) {
    //// TODO
}

ChContactContainerParallel::~ChContactContainerParallel() {
    RemoveAllContacts();
}

void ChContactContainerParallel::RemoveAllContacts() {
    std::list<ChContact_6_6*>::iterator itercontact = contactlist_6_6.begin();
    while (itercontact != contactlist_6_6.end()) {
        delete (*itercontact);
        (*itercontact) = 0;
        ++itercontact;
    }
    contactlist_6_6.clear();
    lastcontact_6_6 = contactlist_6_6.begin();
    n_added_6_6 = 0;
}

void ChContactContainerParallel::BeginAddContact() {
    lastcontact_6_6 = contactlist_6_6.begin();
    n_added_6_6 = 0;
}

void ChContactContainerParallel::EndAddContact() {
    // remove contacts that are beyond last contact
    while (lastcontact_6_6 != contactlist_6_6.end()) {
        delete (*lastcontact_6_6);
        lastcontact_6_6 = contactlist_6_6.erase(lastcontact_6_6);
    }
}

static inline chrono::ChVector<> ToChVector(const real3& a) {
    return chrono::ChVector<>(a.x, a.y, a.z);
}

void ChContactContainerParallel::ReportAllContacts(ReportContactCallback* callback) {
    // Readibility
    auto& ptA = data_manager->host_data.cpta_rigid_rigid;
    auto& ptB = data_manager->host_data.cptb_rigid_rigid;
    auto& nrm = data_manager->host_data.norm_rigid_rigid;
    auto& depth = data_manager->host_data.dpth_rigid_rigid;
    auto& erad = data_manager->host_data.erad_rigid_rigid;
    auto& bids = data_manager->host_data.bids_rigid_rigid;

    // Grab the list of bodies.
    // NOTE: we assume that bodies were added in the order of their IDs!
    auto bodylist = GetSystem()->Get_bodylist();

    // No reaction forces or torques reported!
    ChVector<> zero(0, 0, 0);

    // Contact plane
    ChVector<> plane_x, plane_y, plane_z;
    ChMatrix33<> contact_plane;

    for (uint i = 0; i < data_manager->num_rigid_contacts; i++) {
        // Contact plane coordinate system (normal in x direction)
        XdirToDxDyDz(ToChVector(nrm[i]), VECT_Y, plane_x, plane_y, plane_z);
        contact_plane.Set_A_axis(plane_x, plane_y, plane_z);

        // Invoke callback function
        bool proceed =
            callback->OnReportContact(ToChVector(ptA[i]), ToChVector(ptB[i]), contact_plane, depth[i], erad[i], zero,
                                      zero, bodylist[bids[i].x].get(), bodylist[bids[i].y].get());
        if (!proceed)
            break;
    }
}

void ChContactContainerParallel::ComputeContactForces() {
    // Defer to associated system
    static_cast<ChSystemParallel*>(GetSystem())->CalculateContactForces();
}

ChVector<> ChContactContainerParallel::GetContactableForce(ChContactable* contactable) {
    // If contactable is a body, defer to associated system
    if (auto body = dynamic_cast<ChBody*>(contactable)) {
        real3 frc = static_cast<ChSystemParallel*>(GetSystem())->GetBodyContactForce(body->GetId());
        return ToChVector(frc);
    }

    return ChVector<>(0, 0, 0);
}

ChVector<> ChContactContainerParallel::GetContactableTorque(ChContactable* contactable) {
    // If contactable is a body, defer to associated system
    if (auto body = dynamic_cast<ChBody*>(contactable)) {
        real3 trq = static_cast<ChSystemParallel*>(GetSystem())->GetBodyContactTorque(body->GetId());
        return ToChVector(trq);
    }

    return ChVector<>(0, 0, 0);
}

}  // end namespace chrono
