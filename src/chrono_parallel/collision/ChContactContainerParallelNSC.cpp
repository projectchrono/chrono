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
// Authors: Radu Serban
// =============================================================================

#include "chrono_parallel/collision/ChContactContainerParallelNSC.h"

namespace chrono {

using namespace collision;
using namespace geometry;

ChContactContainerParallelNSC::ChContactContainerParallelNSC(ChParallelDataManager* dc)
    : ChContactContainerParallel(dc) {}

ChContactContainerParallelNSC::ChContactContainerParallelNSC(const ChContactContainerParallelNSC& other)
    : ChContactContainerParallel(other) {
    //// TODO
}

void ChContactContainerParallelNSC::BeginAddContact() {
    ChContactContainerParallel::BeginAddContact();

    // Resize global arrays for composite material properties
    uint num_contacts = data_manager->num_rigid_contacts;

    data_manager->host_data.fric_rigid_rigid.resize(num_contacts);
    data_manager->host_data.coh_rigid_rigid.resize(num_contacts);
    data_manager->host_data.compliance_rigid_rigid.resize(num_contacts);
}

void ChContactContainerParallelNSC::EndAddContact() {
    ChContactContainerParallel::EndAddContact();
    //// Anything else here?!?
}

void ChContactContainerParallelNSC::AddContact(const collision::ChCollisionInfo& cinfo,
                                               std::shared_ptr<ChMaterialSurface> mat1,
                                               std::shared_ptr<ChMaterialSurface> mat2) {
    assert(cinfo.modelA->GetContactable());
    assert(cinfo.modelB->GetContactable());

    // Return now if both objects are inactive
    bool inactiveA = !cinfo.modelA->GetContactable()->IsContactActive();
    bool inactiveB = !cinfo.modelB->GetContactable()->IsContactActive();

    if (inactiveA && inactiveB)
        return;

    // Currently, we only consider contacts between rigid bodies
    ChContactable_1vars<6>* mmboA = dynamic_cast<ChContactable_1vars<6>*>(cinfo.modelA->GetContactable());
    ChContactable_1vars<6>* mmboB = dynamic_cast<ChContactable_1vars<6>*>(cinfo.modelB->GetContactable());

    if (mmboA && mmboB) {
        // Geometric information for added contact. Make sure body IDs are ordered smallest first!
        int b1 = ((ChBody*)(cinfo.modelA->GetPhysicsItem()))->GetId();
        int b2 = ((ChBody*)(cinfo.modelB->GetPhysicsItem()))->GetId();
        if (b1 < b2) {
            data_manager->host_data.norm_rigid_rigid.push_back(real3(cinfo.vN.x(), cinfo.vN.y(), cinfo.vN.z()));
            data_manager->host_data.cpta_rigid_rigid.push_back(real3(cinfo.vpA.x(), cinfo.vpA.y(), cinfo.vpA.z()));
            data_manager->host_data.cptb_rigid_rigid.push_back(real3(cinfo.vpB.x(), cinfo.vpB.y(), cinfo.vpB.z()));
            data_manager->host_data.dpth_rigid_rigid.push_back(cinfo.distance);
            data_manager->host_data.erad_rigid_rigid.push_back(cinfo.eff_radius);  // not really needed here
            data_manager->host_data.bids_rigid_rigid.push_back(vec2(b1, b2));
        } else {
            ChCollisionInfo cinfoS(cinfo, true);  // swap information in cinfo
            data_manager->host_data.norm_rigid_rigid.push_back(real3(cinfoS.vN.x(), cinfoS.vN.y(), cinfoS.vN.z()));
            data_manager->host_data.cpta_rigid_rigid.push_back(real3(cinfoS.vpA.x(), cinfoS.vpA.y(), cinfoS.vpA.z()));
            data_manager->host_data.cptb_rigid_rigid.push_back(real3(cinfoS.vpB.x(), cinfoS.vpB.y(), cinfoS.vpB.z()));
            data_manager->host_data.dpth_rigid_rigid.push_back(cinfoS.distance);
            data_manager->host_data.erad_rigid_rigid.push_back(cinfoS.eff_radius);  // not really needed here
            data_manager->host_data.bids_rigid_rigid.push_back(vec2(b2, b1));
        }

        // Composite material for added contact
        ChMaterialCompositeNSC cmat(data_manager->composition_strategy.get(),
                                    std::static_pointer_cast<ChMaterialSurfaceNSC>(mat1),
                                    std::static_pointer_cast<ChMaterialSurfaceNSC>(mat2));

        // Load composite material properties in global data structure
        data_manager->host_data.fric_rigid_rigid.push_back(
            real3(cmat.sliding_friction, cmat.rolling_friction, cmat.spinning_friction));
        data_manager->host_data.coh_rigid_rigid.push_back(cmat.cohesion);
        data_manager->host_data.compliance_rigid_rigid.push_back(
            real4(cmat.compliance, cmat.complianceT, cmat.complianceRoll, cmat.complianceSpin));

        // Increment number of contacts
        data_manager->num_rigid_contacts++;
    }
}

void ChContactContainerParallelNSC::AddContact(const collision::ChCollisionInfo& cinfo) {
    assert(cinfo.modelA->GetContactable());
    assert(cinfo.modelB->GetContactable());

    // Return now if both objects are inactive
    bool inactiveA = !cinfo.modelA->GetContactable()->IsContactActive();
    bool inactiveB = !cinfo.modelB->GetContactable()->IsContactActive();

    if (inactiveA && inactiveB)
        return;

    // Currently, we only consider contacts between rigid bodies
    ChContactable_1vars<6>* mmboA = dynamic_cast<ChContactable_1vars<6>*>(cinfo.modelA->GetContactable());
    ChContactable_1vars<6>* mmboB = dynamic_cast<ChContactable_1vars<6>*>(cinfo.modelB->GetContactable());

    if (mmboA && mmboB) {
        data_manager->host_data.norm_rigid_rigid.push_back(real3(cinfo.vN.x(), cinfo.vN.y(), cinfo.vN.z()));
        data_manager->host_data.cpta_rigid_rigid.push_back(real3(cinfo.vpA.x(), cinfo.vpA.y(), cinfo.vpA.z()));
        data_manager->host_data.cptb_rigid_rigid.push_back(real3(cinfo.vpB.x(), cinfo.vpB.y(), cinfo.vpB.z()));
        data_manager->host_data.dpth_rigid_rigid.push_back(cinfo.distance);
        data_manager->host_data.erad_rigid_rigid.push_back(cinfo.eff_radius);
        data_manager->host_data.bids_rigid_rigid.push_back(vec2(((ChBody*)(cinfo.modelA->GetPhysicsItem()))->GetId(),
                                                                ((ChBody*)(cinfo.modelB->GetPhysicsItem()))->GetId()));
        data_manager->num_rigid_contacts++;
    }
}

void ChContactContainerParallelNSC::AddContact(int index, int b1, int s1, int b2, int s2) {
    auto& blist = *data_manager->body_list;  // list of bodies in system

    // Identify shapes in their respective collision models
    auto s1_index = data_manager->shape_data.local_rigid[s1];
    auto s2_index = data_manager->shape_data.local_rigid[s2];

    auto mat1 = std::static_pointer_cast<ChMaterialSurfaceNSC>(
        blist[b1]->GetCollisionModel()->GetShape(s1_index)->GetMaterial());
    auto mat2 = std::static_pointer_cast<ChMaterialSurfaceNSC>(
        blist[b2]->GetCollisionModel()->GetShape(s2_index)->GetMaterial());

    // Composite material
    ChMaterialCompositeNSC cmat(data_manager->composition_strategy.get(), mat1, mat2);

    // Allow user to override composite material
    if (data_manager->add_contact_callback) {
        const real3& vN = data_manager->host_data.norm_rigid_rigid[index];
        real3 vpA = data_manager->host_data.cpta_rigid_rigid[index] + data_manager->host_data.pos_rigid[b1];
        real3 vpB = data_manager->host_data.cptb_rigid_rigid[index] + data_manager->host_data.pos_rigid[b2];

        chrono::collision::ChCollisionInfo cinfo;
        cinfo.modelA = blist[b1]->GetCollisionModel().get();
        cinfo.modelB = blist[b2]->GetCollisionModel().get();
        cinfo.shapeA = cinfo.modelA->GetShape(s1_index).get();
        cinfo.shapeB = cinfo.modelB->GetShape(s2_index).get();
        cinfo.vN = ChVector<>(vN.x, vN.y, vN.z);
        cinfo.vpA = ChVector<>(vpA.x, vpA.y, vpA.z);
        cinfo.vpB = ChVector<>(vpB.x, vpB.y, vpB.z);
        cinfo.distance = data_manager->host_data.dpth_rigid_rigid[index];
        cinfo.eff_radius = data_manager->host_data.erad_rigid_rigid[index];

        data_manager->add_contact_callback->OnAddContact(cinfo, &cmat);
    }

    // Load composite material properties in global data structure
    real3 mu(cmat.sliding_friction, cmat.rolling_friction, cmat.spinning_friction);
    real4 compliance(cmat.compliance, cmat.complianceT, cmat.complianceRoll, cmat.complianceSpin);
    data_manager->host_data.coh_rigid_rigid[index] = real(cmat.cohesion);
    data_manager->host_data.fric_rigid_rigid[index] = mu;
    data_manager->host_data.compliance_rigid_rigid[index] = compliance;
}

}  // end namespace chrono
