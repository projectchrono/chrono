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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChContactContainerNSC.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChContactContainerNSC)

ChContactContainerNSC::ChContactContainerNSC() : n_added(0), n_added_rolling(0) {}

ChContactContainerNSC::ChContactContainerNSC(const ChContactContainerNSC& other)
    : ChContactContainer(other), n_added(0), n_added_rolling(0) {}

ChContactContainerNSC::~ChContactContainerNSC() {
    RemoveAllContacts();
}

void ChContactContainerNSC::Update(double time, bool update_assets) {
    // Inherit time changes of parent class, basically doing nothing :)
    ChContactContainer::Update(time, update_assets);
}

void ChContactContainerNSC::RemoveAllContacts() {
    {
        auto iter = contacts.begin();
        while (iter != contacts.end()) {
            delete *iter;
            *iter = 0;
            ++iter;
        }
        contacts.clear();
        last_contact = contacts.begin();
        n_added = 0;
    }
    {
        auto iter = contacts_rolling.begin();
        while (iter != contacts_rolling.end()) {
            delete *iter;
            *iter = nullptr;
            ++iter;
        }
        contacts_rolling.clear();
        last_contact_rolling = contacts_rolling.begin();
        n_added_rolling = 0;
    }
}

void ChContactContainerNSC::BeginAddContact() {
    last_contact = contacts.begin();
    n_added = 0;

    last_contact_rolling = contacts_rolling.begin();
    n_added_rolling = 0;
}

void ChContactContainerNSC::EndAddContact() {
    // Remove contacts that are beyond last contact
    while (last_contact != contacts.end()) {
        delete *last_contact;
        last_contact = contacts.erase(last_contact);
    }

    while (last_contact_rolling != contacts_rolling.end()) {
        delete (*last_contact_rolling);
        last_contact_rolling = contacts_rolling.erase(last_contact_rolling);
    }
}

void ChContactContainerNSC::AddContact(const ChCollisionInfo& cinfo,
                                       std::shared_ptr<ChContactMaterial> mat1,
                                       std::shared_ptr<ChContactMaterial> mat2) {
    assert(cinfo.modelA->GetContactable());
    assert(cinfo.modelB->GetContactable());

    auto contactableA = cinfo.modelA->GetContactable();
    auto contactableB = cinfo.modelB->GetContactable();

    // Do nothing if any of the contactables is not contact-active
    if (!contactableA->IsContactActive() && !contactableB->IsContactActive())
        return;

    // Check that the two collision models are compatible with penalty contact.
    if (mat1->GetContactMethod() != ChContactMethod::NSC || mat2->GetContactMethod() != ChContactMethod::NSC) {
        return;
    }

    // Create the composite material
    ChContactMaterialCompositeNSC cmat(GetSystem()->composition_strategy.get(),
                                       std::static_pointer_cast<ChContactMaterialNSC>(mat1),
                                       std::static_pointer_cast<ChContactMaterialNSC>(mat2));

    InsertContact(cinfo, cmat);
}

void ChContactContainerNSC::AddContact(const ChCollisionInfo& cinfo) {
    assert(cinfo.modelA->GetContactable());
    assert(cinfo.modelB->GetContactable());

    auto contactableA = cinfo.modelA->GetContactable();
    auto contactableB = cinfo.modelB->GetContactable();

    // Do nothing if any of the contactables is not contact-active
    if (!contactableA->IsContactActive() && !contactableB->IsContactActive())
        return;

    // Check that the two collision models are compatible with complementarity contact.
    if (cinfo.shapeA->GetContactMethod() != ChContactMethod::NSC ||
        cinfo.shapeB->GetContactMethod() != ChContactMethod::NSC) {
        return;
    }

    // Create the composite material
    ChContactMaterialCompositeNSC cmat(GetSystem()->composition_strategy.get(),
                                       std::static_pointer_cast<ChContactMaterialNSC>(cinfo.shapeA->GetMaterial()),
                                       std::static_pointer_cast<ChContactMaterialNSC>(cinfo.shapeB->GetMaterial()));

    // Check for a user-provided callback to modify the material
    if (GetAddContactCallback()) {
        GetAddContactCallback()->OnAddContact(cinfo, &cmat);
    }

    InsertContact(cinfo, cmat);
}

void ChContactContainerNSC::InsertContact(const ChCollisionInfo& cinfo, const ChContactMaterialCompositeNSC& cmat) {
    auto contactableA = cinfo.modelA->GetContactable();
    auto contactableB = cinfo.modelB->GetContactable();

    auto typeA = contactableA->GetContactableType();
    auto typeB = contactableB->GetContactableType();

    bool rolling = (typeA == ChContactable::Type::ONE_6 && typeB == ChContactable::Type::ONE_6) &&
                   (cmat.rolling_friction > 0 || cmat.spinning_friction > 0);

    if (rolling) {
        if (last_contact_rolling != contacts_rolling.end()) {
            // reuse old contacts
            (*last_contact_rolling)->Reset(contactableA, contactableB, cinfo, cmat, GetMinBounceSpeed());
            last_contact++;
        } else {
            // add new contact
            auto c = new ChContactNSCrolling(this, contactableA, contactableB, cinfo, cmat, GetMinBounceSpeed());
            contacts_rolling.push_back(c);
            last_contact_rolling = contacts_rolling.end();
        }
        n_added_rolling++;
    } else  {
        if (last_contact != contacts.end()) {
            // reuse old contacts
            (*last_contact)->Reset(contactableA, contactableB, cinfo, cmat, GetMinBounceSpeed());
            last_contact++;
        } else {
            // add new contact
            auto c = new ChContactNSC(this, contactableA, contactableB, cinfo, cmat, GetMinBounceSpeed());
            contacts.push_back(c);
            last_contact = contacts.end();
        }
        n_added++;
    }
}

void ChContactContainerNSC::ComputeContactForces() {
    contact_forces.clear();
    SumAllContactForces(contacts, contact_forces);
    SumAllContactForces(contacts_rolling, contact_forces);
}

ChVector3d ChContactContainerNSC::GetContactableForce(ChContactable* contactable) {
    std::unordered_map<ChContactable*, ForceTorque>::const_iterator Iterator = contact_forces.find(contactable);
    if (Iterator != contact_forces.end()) {
        return Iterator->second.force;
    }
    return ChVector3d(0);
}

ChVector3d ChContactContainerNSC::GetContactableTorque(ChContactable* contactable) {
    std::unordered_map<ChContactable*, ForceTorque>::const_iterator Iterator = contact_forces.find(contactable);
    if (Iterator != contact_forces.end()) {
        return Iterator->second.torque;
    }
    return ChVector3d(0);
}

void ChContactContainerNSC::ReportAllContacts(std::shared_ptr<ReportContactCallback> callback) {
    {
        auto iter = contacts.begin();
        while (iter != contacts.end()) {
            bool proceed =
                callback->OnReportContact((*iter)->GetContactP1(), (*iter)->GetContactP2(),                       //
                                          (*iter)->GetContactPlane(),                                             //
                                          (*iter)->GetContactDistance(), (*iter)->GetEffectiveCurvatureRadius(),  //
                                          (*iter)->GetContactForce(), VNULL,                                      //
                                          (*iter)->GetObjA(), (*iter)->GetObjB(),                                 //
                                          (*iter)->GetConstraintNx()->GetOffset());
            if (!proceed)
                break;
            ++iter;
        }
    }
    {
        auto iter = contacts_rolling.begin();
        while (iter != contacts_rolling.end()) {
            bool proceed =
                callback->OnReportContact((*iter)->GetContactP1(), (*iter)->GetContactP2(),                       //
                                          (*iter)->GetContactPlane(),                                             //
                                          (*iter)->GetContactDistance(), (*iter)->GetEffectiveCurvatureRadius(),  //
                                          (*iter)->GetContactForce(), (*iter)->GetContactTorque(),                //
                                          (*iter)->GetObjA(), (*iter)->GetObjB(),                                 //
                                          (*iter)->GetConstraintNx()->GetOffset());
            if (!proceed)
                break;
            ++iter;
        }
    }
}

////////// STATE INTERFACE ////

void ChContactContainerNSC::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    unsigned int coffset = 0;
    {
        auto iter = contacts.begin();
        while (iter != contacts.end()) {
            (*iter)->ContIntStateGatherReactions(off_L + coffset, L);
            coffset += 3;
            ++iter;
        }
    }
    {
        auto iter = contacts_rolling.begin();
        while (iter != contacts_rolling.end()) {
            (*iter)->ContIntStateGatherReactions(off_L + coffset, L);
            coffset += 6;
            ++iter;
        }
    }
}

void ChContactContainerNSC::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    unsigned int coffset = 0;

    {
        auto iter = contacts.begin();
        while (iter != contacts.end()) {
            (*iter)->ContIntStateScatterReactions(off_L + coffset, L);
            coffset += 3;
            ++iter;
        }
    }
    {
        auto iter = contacts_rolling.begin();
        while (iter != contacts_rolling.end()) {
            (*iter)->ContIntStateScatterReactions(off_L + coffset, L);
            coffset += 6;
            ++iter;
        }
    }
}

void ChContactContainerNSC::IntLoadResidual_CqL(const unsigned int off_L,
                                                ChVectorDynamic<>& R,
                                                const ChVectorDynamic<>& L,
                                                const double c) {
    unsigned int coffset = 0;

    {
        auto iter = contacts.begin();
        while (iter != contacts.end()) {
            (*iter)->ContIntLoadResidual_CqL(off_L + coffset, R, L, c);
            coffset += 3;
            ++iter;
        }
    }
    {
        auto iter = contacts_rolling.begin();
        while (iter != contacts_rolling.end()) {
            (*iter)->ContIntLoadResidual_CqL(off_L + coffset, R, L, c);
            coffset += 6;
            ++iter;
        }
    }
}

void ChContactContainerNSC::IntLoadConstraint_C(const unsigned int off,
                                                ChVectorDynamic<>& Qc,
                                                const double c,
                                                bool do_clamp,
                                                double recovery_clamp) {
    unsigned int coffset = 0;

    {
        auto iter = contacts.begin();
        while (iter != contacts.end()) {
            (*iter)->ContIntLoadConstraint_C(off + coffset, Qc, c, do_clamp, recovery_clamp);
            coffset += 3;
            ++iter;
        }
    }
    {
        auto iter = contacts_rolling.begin();
        while (iter != contacts_rolling.end()) {
            (*iter)->ContIntLoadConstraint_C(off + coffset, Qc, c, do_clamp, recovery_clamp);
            coffset += 6;
            ++iter;
        }
    }
}

void ChContactContainerNSC::IntToDescriptor(const unsigned int off_v,
                                            const ChStateDelta& v,
                                            const ChVectorDynamic<>& R,
                                            const unsigned int off_L,
                                            const ChVectorDynamic<>& L,
                                            const ChVectorDynamic<>& Qc) {
    unsigned int coffset = 0;

    {
        auto iter = contacts.begin();
        while (iter != contacts.end()) {
            (*iter)->ContIntToDescriptor(off_L + coffset, L, Qc);
            coffset += 3;
            ++iter;
        }
    }
    {
        auto iter = contacts_rolling.begin();
        while (iter != contacts_rolling.end()) {
            (*iter)->ContIntToDescriptor(off_L + coffset, L, Qc);
            coffset += 6;
            ++iter;
        }
    }
}

void ChContactContainerNSC::IntFromDescriptor(const unsigned int off_v,
                                              ChStateDelta& v,
                                              const unsigned int off_L,
                                              ChVectorDynamic<>& L) {
    unsigned int coffset = 0;

    {
        auto iter = contacts.begin();
        while (iter != contacts.end()) {
            (*iter)->ContIntFromDescriptor(off_L + coffset, L);
            coffset += 3;
            ++iter;
        }
    }
    {
        {
            auto iter = contacts_rolling.begin();
            while (iter != contacts_rolling.end()) {
                (*iter)->ContIntFromDescriptor(off_L + coffset, L);
                coffset += 6;
                ++iter;
            }
        }
    }
}

// SOLVER INTERFACES

void ChContactContainerNSC::InjectConstraints(ChSystemDescriptor& descriptor) {
    {
        auto iter = contacts.begin();
        while (iter != contacts.end()) {
            (*iter)->InjectConstraints(descriptor);
            ++iter;
        }
    }
    {
        auto iter = contacts_rolling.begin();
        while (iter != contacts_rolling.end()) {
            (*iter)->InjectConstraints(descriptor);
            ++iter;
        }
    }
}

void ChContactContainerNSC::ConstraintsBiReset() {
    {
        auto iter = contacts.begin();
        while (iter != contacts.end()) {
            (*iter)->ConstraintsBiReset();
            ++iter;
        }
    }
    {
        auto iter = contacts_rolling.begin();
        while (iter != contacts_rolling.end()) {
            (*iter)->ConstraintsBiReset();
            ++iter;
        }
    }
}

void ChContactContainerNSC::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    {
        auto iter = contacts.begin();
        while (iter != contacts.end()) {
            (*iter)->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
            ++iter;
        }
    }
    {
        auto iter = contacts_rolling.begin();
        while (iter != contacts_rolling.end()) {
            (*iter)->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
            ++iter;
        }
    }
}

void ChContactContainerNSC::LoadConstraintJacobians() {
    // already loaded when contact objects are created
}

void ChContactContainerNSC::ConstraintsFetch_react(double factor) {
    {
        auto iter = contacts.begin();
        while (iter != contacts.end()) {
            (*iter)->ConstraintsFetch_react(factor);
            ++iter;
        }
    }

    {
        auto iter = contacts_rolling.begin();
        while (iter != contacts_rolling.end()) {
            (*iter)->ConstraintsFetch_react(factor);
            ++iter;
        }
    }
}

void ChContactContainerNSC::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChContactContainerNSC>();
    // serialize parent class
    ChContactContainer::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(min_bounce_speed);

    // NO SERIALIZATION of contact list because assume it is volatile and generated when needed
}

/// Method to allow de serialization of transient data from archives.
void ChContactContainerNSC::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChContactContainerNSC>();
    // deserialize parent class
    ChContactContainer::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(min_bounce_speed);

    RemoveAllContacts();
    // NO SERIALIZATION of contact list because assume it is volatile and generated when needed
}
}  // end namespace chrono
