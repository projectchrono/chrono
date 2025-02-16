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

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChParticleCloud.h"

#include "chrono/multicore_math/thrust.h"

#include "chrono_multicore/collision/ChContactContainerMulticore.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"

namespace chrono {

ChContactContainerMulticore::ChContactContainerMulticore(ChMulticoreDataManager* dc) : data_manager(dc) {
    contactlist_6_6.clear();
    n_added_6_6 = 0;
}

ChContactContainerMulticore::ChContactContainerMulticore(const ChContactContainerMulticore& other)
    : ChContactContainer(other) {
    //// TODO
}

ChContactContainerMulticore::~ChContactContainerMulticore() {
    RemoveAllContacts();
}

void ChContactContainerMulticore::RemoveAllContacts() {
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

void ChContactContainerMulticore::BeginAddContact() {
    lastcontact_6_6 = contactlist_6_6.begin();
    n_added_6_6 = 0;
}

void ChContactContainerMulticore::EndAddContact() {
    // remove contacts that are beyond last contact
    while (lastcontact_6_6 != contactlist_6_6.end()) {
        delete (*lastcontact_6_6);
        lastcontact_6_6 = contactlist_6_6.erase(lastcontact_6_6);
    }
}

void ChContactContainerMulticore::ReportAllContacts(std::shared_ptr<ReportContactCallback> callback) {
    // Readibility
    auto& cd_data = data_manager->cd_data;
    const auto& ptA = cd_data->cpta_rigid_rigid;
    const auto& ptB = cd_data->cptb_rigid_rigid;
    const auto& nrm = cd_data->norm_rigid_rigid;
    const auto& depth = cd_data->dpth_rigid_rigid;
    const auto& erad = cd_data->erad_rigid_rigid;
    const auto& bids = cd_data->bids_rigid_rigid;

    // NSC-specific
    auto mode = data_manager->settings.solver.local_solver_mode;
    const DynamicVector<real>& gamma_u =
        blaze::subvector(data_manager->host_data.gamma, 0, data_manager->num_unilaterals);

    // SMC-specific
    const auto& ct_force = data_manager->host_data.ct_force;
    const auto& ct_torque = data_manager->host_data.ct_torque;

    // Grab the list of bodies.
    // NOTE: we assume that bodies were added in the order of their IDs!
    auto bodylist = GetSystem()->GetBodies();

    // Contact forces
    ChVector3d force;
    ChVector3d torque;

    // Contact plane
    ChMatrix33<> contact_plane;

    for (uint i = 0; i < cd_data->num_rigid_contacts; i++) {
        auto bodyA = bodylist[bids[i].x].get();
        auto bodyB = bodylist[bids[i].y].get();

        auto pA = ToChVector(ptA[i]);  // in absolute frame
        auto pB = ToChVector(ptB[i]);  // in absolute frame

        // Contact plane coordinate system (normal in x direction from pB to pA)
        contact_plane.SetFromAxisX(ToChVector(nrm[i]), VECT_Y);

        // Contact force and torque expressed in the contact plane
        switch (GetSystem()->GetContactMethod()) {
            case ChContactMethod::NSC: {
                double f_n = (double)(gamma_u[i] / data_manager->settings.step_size);
                double f_u = 0;
                double f_v = 0;
                if (mode == SolverMode::SLIDING || mode == SolverMode::SPINNING) {
                    f_u = (double)(gamma_u[cd_data->num_rigid_contacts + 2 * i + 0] / data_manager->settings.step_size);
                    f_v = (double)(gamma_u[cd_data->num_rigid_contacts + 2 * i + 1] / data_manager->settings.step_size);
                }
                force = ChVector3d(f_n, f_u, f_v);
                double t_n = 0;
                double t_u = 0;
                double t_v = 0;
                if (mode == SolverMode::SPINNING) {
                    t_n = (double)(gamma_u[3 * cd_data->num_rigid_contacts + 3 * i + 0] /
                                   data_manager->settings.step_size);
                    t_u = (double)(gamma_u[3 * cd_data->num_rigid_contacts + 3 * i + 1] /
                                   data_manager->settings.step_size);
                    t_v = (double)(gamma_u[3 * cd_data->num_rigid_contacts + 3 * i + 2] /
                                   data_manager->settings.step_size);
                }
                torque = ChVector3d(t_n, t_u, t_v);
                break;
            }
            case ChContactMethod::SMC: {
                // Convert force and torque to the contact frame.
                // Consistent with the normal direction, use force and torque on body B.
                auto force_abs = ToChVector(ct_force[2 * i + 1]);                       // in abs frame
                auto torque_loc = ToChVector(ct_torque[2 * i + 1]);                     // in body frame, at body origin
                auto force_loc = bodyB->TransformDirectionParentToLocal(force_abs);     // in body frame
                auto ptB_loc = bodyB->TransformPointParentToLocal(pB);                  // in body frame
                force = contact_plane.transpose() * force_abs;                          // in contact frame
                auto torque_loc1 = torque_loc - ptB_loc.Cross(force_loc);               // in body frame, at contact
                auto torque_abs = bodyB->TransformDirectionLocalToParent(torque_loc1);  // in abs frame, at contact
                torque = contact_plane.transpose() * torque_abs;                        // in contact frame, at contact
                break;
            }
        }

        // Invoke callback function
        bool proceed = callback->OnReportContact(pA, pB, contact_plane, depth[i], erad[i], force, torque, bodyA, bodyB);
        if (!proceed)
            break;
    }
}

void ChContactContainerMulticore::ComputeContactForces() {
    // Defer to associated system
    static_cast<ChSystemMulticore*>(GetSystem())->CalculateContactForces();
}

ChVector3d ChContactContainerMulticore::GetContactableForce(ChContactable* contactable) {
    // If contactable is a body, defer to associated system
    if (auto body = dynamic_cast<ChBody*>(contactable)) {
        std::shared_ptr<ChBody> pbody(body, [](ChBody*) {}); 
        real3 frc = static_cast<ChSystemMulticore*>(GetSystem())->GetBodyContactForce(pbody);
        return ToChVector(frc);
    }

    return ChVector3d(0, 0, 0);
}

ChVector3d ChContactContainerMulticore::GetContactableTorque(ChContactable* contactable) {
    // If contactable is a body, defer to associated system
    if (auto body = dynamic_cast<ChBody*>(contactable)) {
        std::shared_ptr<ChBody> pbody(body, [](ChBody*) {});
        real3 trq = static_cast<ChSystemMulticore*>(GetSystem())->GetBodyContactTorque(pbody);
        return ToChVector(trq);
    }

    return ChVector3d(0, 0, 0);
}

}  // end namespace chrono
