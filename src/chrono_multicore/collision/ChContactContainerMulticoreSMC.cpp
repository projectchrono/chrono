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

#include "chrono_multicore/collision/ChContactContainerMulticoreSMC.h"
#include "chrono/collision/multicore/ChCollisionModelMulticore.h"

namespace chrono {

using namespace geometry;

ChContactContainerMulticoreSMC::ChContactContainerMulticoreSMC(ChMulticoreDataManager* dc)
    : ChContactContainerMulticore(dc) {}

ChContactContainerMulticoreSMC::ChContactContainerMulticoreSMC(const ChContactContainerMulticoreSMC& other)
    : ChContactContainerMulticore(other) {
    //// TODO
}

void ChContactContainerMulticoreSMC::BeginAddContact() {
    ChContactContainerMulticore::BeginAddContact();

    // Resize global arrays for composite material properties
    uint num_contacts = data_manager->cd_data->num_rigid_contacts;

    data_manager->host_data.fric_rigid_rigid.resize(num_contacts);
    data_manager->host_data.modulus_rigid_rigid.resize(num_contacts);
    data_manager->host_data.adhesion_rigid_rigid.resize(num_contacts);
    data_manager->host_data.cr_rigid_rigid.resize(num_contacts);
    data_manager->host_data.smc_rigid_rigid.resize(num_contacts);
}

void ChContactContainerMulticoreSMC::EndAddContact() {
    ChContactContainerMulticore::EndAddContact();
    //// Anything else here?!?
}

void ChContactContainerMulticoreSMC::AddContact(const ChCollisionInfo& cinfo,
                                                std::shared_ptr<ChMaterialSurface> mat1,
                                                std::shared_ptr<ChMaterialSurface> mat2) {
    assert(cinfo.modelA->GetContactable());
    assert(cinfo.modelB->GetContactable());

    // Return now if both objects are inactive
    bool inactiveA = !cinfo.modelA->GetContactable()->IsContactActive();
    bool inactiveB = !cinfo.modelB->GetContactable()->IsContactActive();

    if (inactiveA && inactiveB)
        return;

    auto& cd_data = data_manager->cd_data;  // collision system data

    // Currently, we only consider contacts between rigid bodies
    ChContactable_1vars<6>* mmboA = dynamic_cast<ChContactable_1vars<6>*>(cinfo.modelA->GetContactable());
    ChContactable_1vars<6>* mmboB = dynamic_cast<ChContactable_1vars<6>*>(cinfo.modelB->GetContactable());

    if (mmboA && mmboB) {
        // Geometric information for added contact.
        int b1 = ((ChBody*)(cinfo.modelA->GetPhysicsItem()))->GetId();
        int b2 = ((ChBody*)(cinfo.modelB->GetPhysicsItem()))->GetId();
        cd_data->norm_rigid_rigid.push_back(real3(cinfo.vN.x(), cinfo.vN.y(), cinfo.vN.z()));
        cd_data->cpta_rigid_rigid.push_back(real3(cinfo.vpA.x(), cinfo.vpA.y(), cinfo.vpA.z()));
        cd_data->cptb_rigid_rigid.push_back(real3(cinfo.vpB.x(), cinfo.vpB.y(), cinfo.vpB.z()));
        cd_data->dpth_rigid_rigid.push_back(cinfo.distance);
        cd_data->erad_rigid_rigid.push_back(cinfo.eff_radius);
        cd_data->bids_rigid_rigid.push_back(vec2(b1, b2));

        // Composite material for added contact
        ChMaterialCompositeSMC cmat(data_manager->composition_strategy.get(),
                                    std::static_pointer_cast<ChMaterialSurfaceSMC>(mat1),
                                    std::static_pointer_cast<ChMaterialSurfaceSMC>(mat2));

        // Load composite material properties in global data structure
        data_manager->host_data.fric_rigid_rigid.push_back(real3(cmat.mu_eff, cmat.muRoll_eff, cmat.muSpin_eff));
        data_manager->host_data.modulus_rigid_rigid.push_back(real2(cmat.E_eff, cmat.G_eff));
        data_manager->host_data.adhesion_rigid_rigid.push_back(
            real3(cmat.adhesion_eff, cmat.adhesionMultDMT_eff, cmat.adhesionSPerko_eff));
        data_manager->host_data.cr_rigid_rigid.push_back(real(cmat.cr_eff));
        data_manager->host_data.smc_rigid_rigid.push_back(real4(cmat.kn, cmat.kt, cmat.gn, cmat.gt));

        // Increment number of contacts
        cd_data->num_rigid_contacts++;
    }
}

void ChContactContainerMulticoreSMC::AddContact(const ChCollisionInfo& cinfo) {
    assert(cinfo.modelA->GetContactable());
    assert(cinfo.modelB->GetContactable());

    // Return now if both objects are inactive
    bool inactiveA = !cinfo.modelA->GetContactable()->IsContactActive();
    bool inactiveB = !cinfo.modelB->GetContactable()->IsContactActive();

    if (inactiveA && inactiveB)
        return;

    auto& cd_data = data_manager->cd_data;  // collision system data

    // Currently, we only consider contacts between rigid bodies
    ChContactable_1vars<6>* mmboA = dynamic_cast<ChContactable_1vars<6>*>(cinfo.modelA->GetContactable());
    ChContactable_1vars<6>* mmboB = dynamic_cast<ChContactable_1vars<6>*>(cinfo.modelB->GetContactable());

    if (mmboA && mmboB) {
        cd_data->norm_rigid_rigid.push_back(real3(cinfo.vN.x(), cinfo.vN.y(), cinfo.vN.z()));
        cd_data->cpta_rigid_rigid.push_back(real3(cinfo.vpA.x(), cinfo.vpA.y(), cinfo.vpA.z()));
        cd_data->cptb_rigid_rigid.push_back(real3(cinfo.vpB.x(), cinfo.vpB.y(), cinfo.vpB.z()));
        cd_data->dpth_rigid_rigid.push_back(cinfo.distance);
        cd_data->erad_rigid_rigid.push_back(cinfo.eff_radius);
        cd_data->bids_rigid_rigid.push_back(vec2(((ChBody*)(cinfo.modelA->GetPhysicsItem()))->GetId(),
                                                 ((ChBody*)(cinfo.modelB->GetPhysicsItem()))->GetId()));
        cd_data->num_rigid_contacts++;
    }
}

void ChContactContainerMulticoreSMC::AddContact(int index, int b1, int s1, int b2, int s2) {
    auto& cd_data = data_manager->cd_data;   // collision system data
    auto& blist = *data_manager->body_list;  // list of bodies in system

    // Identify shapes in their respective collision models
    auto s1_index = cd_data->shape_data.local_rigid[s1];
    auto s2_index = cd_data->shape_data.local_rigid[s2];

    // Chrono multicore collision model implementation
    auto modelA = (ChCollisionModelMulticore*)blist[b1]->GetCollisionModel()->GetImplementation();
    auto modelB = (ChCollisionModelMulticore*)blist[b2]->GetCollisionModel()->GetImplementation();

    // Collsion shapes in contact
    auto shape1 = modelA->m_shapes[s1_index].get();
    auto shape2 = modelB->m_shapes[s2_index].get();

    // Contact materials of the two colliding shapes
    auto mat1 = std::static_pointer_cast<ChMaterialSurfaceSMC>(shape1->GetMaterial());
    auto mat2 = std::static_pointer_cast<ChMaterialSurfaceSMC>(shape2->GetMaterial());

    // Composite material
    ChMaterialCompositeSMC cmat(data_manager->composition_strategy.get(), mat1, mat2);

    // Allow user to override composite material
    if (data_manager->add_contact_callback) {
        const real3& vN = cd_data->norm_rigid_rigid[index];
        //// TODO:  check this!!!  Why do we add the body position?!?
        real3 vpA = cd_data->cpta_rigid_rigid[index] + data_manager->host_data.pos_rigid[b1];
        real3 vpB = cd_data->cptb_rigid_rigid[index] + data_manager->host_data.pos_rigid[b2];

        chrono::ChCollisionInfo cinfo;
        cinfo.modelA = blist[b1]->GetCollisionModel().get();
        cinfo.modelB = blist[b2]->GetCollisionModel().get();
        cinfo.shapeA = shape1;
        cinfo.shapeB = shape2;
        cinfo.vN = ChVector<>(vN.x, vN.y, vN.z);
        cinfo.vpA = ChVector<>(vpA.x, vpA.y, vpA.z);
        cinfo.vpB = ChVector<>(vpB.x, vpB.y, vpB.z);
        cinfo.distance = cd_data->dpth_rigid_rigid[index];
        cinfo.eff_radius = cd_data->erad_rigid_rigid[index];

        data_manager->add_contact_callback->OnAddContact(cinfo, &cmat);
    }

    // Load composite material properties in global data structure
    data_manager->host_data.fric_rigid_rigid[index] = real3(cmat.mu_eff, cmat.muRoll_eff, cmat.muSpin_eff);
    data_manager->host_data.modulus_rigid_rigid[index] = real2(cmat.E_eff, cmat.G_eff);
    data_manager->host_data.adhesion_rigid_rigid[index] =
        real3(cmat.adhesion_eff, cmat.adhesionMultDMT_eff, cmat.adhesionSPerko_eff);
    data_manager->host_data.cr_rigid_rigid[index] = real(cmat.cr_eff);
    data_manager->host_data.smc_rigid_rigid[index] = real4(cmat.kn, cmat.kt, cmat.gn, cmat.gt);
}

}  // end namespace chrono
