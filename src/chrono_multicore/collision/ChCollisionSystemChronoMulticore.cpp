// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Chrono custom multicore collision system for Chrono::Multicore.
//
// =============================================================================

#include "chrono_multicore/collision/ChCollisionSystemChronoMulticore.h"
#include "chrono_multicore/collision/ChContactContainerMulticore.h"

namespace chrono {
namespace collision {

ChCollisionSystemChronoMulticore::ChCollisionSystemChronoMulticore(ChMulticoreDataManager* dc)
    : data_manager(dc) {
    // Create the shared data structure with external state data
    cd_data = chrono_types::make_shared<ChCollisionData>(false);
    cd_data->collision_envelope = ChCollisionModel::GetDefaultSuggestedEnvelope();

    broadphase.cd_data = cd_data;
    narrowphase.cd_data = cd_data;
    aabb_generator.cd_data = cd_data;

    // Store a pointer to the shared data structure in the data manager.
    data_manager->cd_data = cd_data;
}

ChCollisionSystemChronoMulticore::~ChCollisionSystemChronoMulticore() {}

void ChCollisionSystemChronoMulticore::SetNumThreads(int nthreads) {
    // Nothing to do here.
    // The Chrono::Multicore collision system uses the number of threads set by ChSystemMulticore.
}

void ChCollisionSystemChronoMulticore::PreProcess() {
    assert(!cd_data->owns_state_data);

    cd_data->state_data.pos_rigid = &data_manager->host_data.pos_rigid;
    cd_data->state_data.rot_rigid = &data_manager->host_data.rot_rigid;
    cd_data->state_data.active_rigid = &data_manager->host_data.active_rigid;
    cd_data->state_data.collide_rigid = &data_manager->host_data.collide_rigid;

    cd_data->state_data.pos_3dof = &data_manager->host_data.pos_3dof;
    cd_data->state_data.sorted_pos_3dof = &data_manager->host_data.sorted_pos_3dof;
    cd_data->state_data.vel_3dof = &data_manager->host_data.vel_3dof;
    cd_data->state_data.sorted_vel_3dof = &data_manager->host_data.sorted_vel_3dof;

    cd_data->state_data.num_rigid_bodies = data_manager->num_rigid_bodies;
    cd_data->state_data.num_fluid_bodies = data_manager->num_fluid_bodies;
}

void ChCollisionSystemChronoMulticore::ReportContacts(ChContactContainer* container) {
    assert(dynamic_cast<ChContactContainerMulticore*>(container));

    // Resize global arrays with composite material properties.
    // NOTE: important to do this here, to set size to zero if no contacts (in case some other added by a custom user
    // callback)
    container->BeginAddContact();

    uint num_contacts = cd_data->num_rigid_contacts;
    if (num_contacts <= 0) {
        container->EndAddContact();
        return;
    }

    auto container_mc = static_cast<ChContactContainerMulticore*>(container);

    auto& bids = cd_data->host_data.bids_rigid_rigid;  // global IDs of bodies in contact
    auto& sids = cd_data->host_data.contact_shapeIDs;  // global IDs of shapes in contact
    ////auto& sindex = cd_data->shape_data.local_rigid;    // collision model indexes of shapes in contact

    // Loop over all current contacts, create the composite material, and load material properties in the data manager
    // (on a per contact basis). Snce this is contact method-specific, we defer to the underlying contact container.
#pragma omp parallel for
    for (int i = 0; i < (signed)num_contacts; i++) {
        auto b1 = bids[i].x;                  // global IDs of bodies in contact
        auto b2 = bids[i].y;                  //
        auto s1 = int(sids[i] >> 32);         // global IDs of shapes in contact
        auto s2 = int(sids[i] & 0xffffffff);  //
        ////auto s1_index = sindex[s1];           // collision model indexes of shapes in contact
        ////auto s2_index = sindex[s2];           //

        container_mc->AddContact(i, b1, s1, b2, s2);
    }

    container->EndAddContact();
}

}  // end namespace collision
}  // end namespace chrono
