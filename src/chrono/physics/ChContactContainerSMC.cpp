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

#include "chrono/physics/ChContactContainerSMC.h"
#include "chrono/physics/ChSystemSMC.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChContactContainerSMC)

ChContactContainerSMC::ChContactContainerSMC()
    : n_added(0) {}

ChContactContainerSMC::ChContactContainerSMC(const ChContactContainerSMC& other)
    : ChContactContainer(other),
      n_added(0) {}

ChContactContainerSMC::~ChContactContainerSMC() {
    RemoveAllContacts();
}

void ChContactContainerSMC::Update(double time, bool update_assets) {
    // Inherit time changes of parent class, basically doing nothing :)
    ChContactContainer::Update(time, update_assets);
}

void ChContactContainerSMC::RemoveAllContacts() {
    auto iter = contacts.begin();
    while (iter != contacts.end()) {
        delete *iter;
        *iter = nullptr;
        ++iter;
    }
    contacts.clear();
    last_contact = contacts.begin();
    n_added = 0;
}

void ChContactContainerSMC::BeginAddContact() {
    last_contact = contacts.begin();
    n_added = 0;
}

void ChContactContainerSMC::EndAddContact() {
    // Remove contacts that are beyond last contact
    while (last_contact != contacts.end()) {
        delete *last_contact;
        last_contact = contacts.erase(last_contact);
    }
}

void ChContactContainerSMC::AddContact(const ChCollisionInfo& cinfo,
                                       std::shared_ptr<ChContactMaterial> mat1,
                                       std::shared_ptr<ChContactMaterial> mat2) {
    assert(cinfo.modelA->GetContactable());
    assert(cinfo.modelB->GetContactable());

    // Do nothing if the shapes are separated
    if (cinfo.distance >= 0)
        return;

    auto contactableA = cinfo.modelA->GetContactable();
    auto contactableB = cinfo.modelB->GetContactable();

    // Do nothing if any of the contactables is not contact-active
    if (!contactableA->IsContactActive() && !contactableB->IsContactActive())
        return;

    // Check that the two collision models are compatible with penalty contact.
    if (mat1->GetContactMethod() != ChContactMethod::SMC || mat2->GetContactMethod() != ChContactMethod::SMC) {
        return;
    }

    // Create the composite material
    ChContactMaterialCompositeSMC cmat(GetSystem()->composition_strategy.get(),
                                       std::static_pointer_cast<ChContactMaterialSMC>(mat1),
                                       std::static_pointer_cast<ChContactMaterialSMC>(mat2));

    InsertContact(cinfo, cmat);
}

void ChContactContainerSMC::AddContact(const ChCollisionInfo& cinfo) {
    assert(cinfo.modelA->GetContactable());
    assert(cinfo.modelB->GetContactable());

    // Do nothing if the shapes are separated
    if (cinfo.distance >= 0)
        return;

    auto contactableA = cinfo.modelA->GetContactable();
    auto contactableB = cinfo.modelB->GetContactable();

    // Do nothing if any of the contactables is not contact-active
    if (!contactableA->IsContactActive() && !contactableB->IsContactActive())
        return;

    // Check that the two collision models are compatible with penalty contact.
    if (cinfo.shapeA->GetContactMethod() != ChContactMethod::SMC ||
        cinfo.shapeB->GetContactMethod() != ChContactMethod::SMC) {
        return;
    }

    // Create the composite material
    ChContactMaterialCompositeSMC cmat(GetSystem()->composition_strategy.get(),
                                       std::static_pointer_cast<ChContactMaterialSMC>(cinfo.shapeA->GetMaterial()),
                                       std::static_pointer_cast<ChContactMaterialSMC>(cinfo.shapeB->GetMaterial()));

    // Check for a user-provided callback to modify the material
    if (GetAddContactCallback()) {
        GetAddContactCallback()->OnAddContact(cinfo, &cmat);
    }

    InsertContact(cinfo, cmat);
}

void ChContactContainerSMC::InsertContact(const ChCollisionInfo& cinfo, const ChContactMaterialCompositeSMC& cmat) {
    auto contactableA = cinfo.modelA->GetContactable();
    auto contactableB = cinfo.modelB->GetContactable();

    if (last_contact != contacts.end()) {
        // reuse old contacts
        (*last_contact)->Reset(contactableA, contactableB, cinfo, cmat);
        last_contact++;
    } else {
        // add new contact
        auto c = new ChContactSMC(this, contactableA, contactableB, cinfo, cmat);
        contacts.push_back(c);
        last_contact = contacts.end();
    }
    n_added++;
}

void ChContactContainerSMC::ComputeContactForces() {
    contact_forces.clear();
    SumAllContactForces(contacts, contact_forces);
}

ChVector3d ChContactContainerSMC::GetContactableForce(ChContactable* contactable) {
    std::unordered_map<ChContactable*, ForceTorque>::const_iterator Iterator = contact_forces.find(contactable);
    if (Iterator != contact_forces.end()) {
        return Iterator->second.force;
    }
    return ChVector3d(0);
}

ChVector3d ChContactContainerSMC::GetContactableTorque(ChContactable* contactable) {
    std::unordered_map<ChContactable*, ForceTorque>::const_iterator Iterator = contact_forces.find(contactable);
    if (Iterator != contact_forces.end()) {
        return Iterator->second.torque;
    }
    return ChVector3d(0);
}

void ChContactContainerSMC::ReportAllContacts(std::shared_ptr<ReportContactCallback> callback) {
    auto iter = contacts.begin();
    while (iter != contacts.end()) {
        bool proceed =
            callback->OnReportContact((*iter)->GetContactP1(), (*iter)->GetContactP2(),                       //
                                      (*iter)->GetContactPlane(),                                             //
                                      (*iter)->GetContactDistance(), (*iter)->GetEffectiveCurvatureRadius(),  //
                                      (*iter)->GetContactForce(), VNULL,                                      //
                                      (*iter)->GetObjA(), (*iter)->GetObjB(),                                 //
                                      -1);
        if (!proceed)
            break;
        ++iter;
    }
}

// STATE INTERFACE

void ChContactContainerSMC::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    auto iter = contacts.begin();
    while (iter != contacts.end()) {
        (*iter)->ContIntLoadResidual_F(R, c);
        ++iter;
    }
}

void ChContactContainerSMC::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    auto iter = contacts.begin();
    while (iter != contacts.end()) {
        (*iter)->ContKRMmatricesLoad(Kfactor, Rfactor);
        ++iter;
    }
}

void ChContactContainerSMC::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    auto iter = contacts.begin();
    while (iter != contacts.end()) {
        (*iter)->ContInjectKRMmatrices(descriptor);
        ++iter;
    }
}

// OBSOLETE
void ChContactContainerSMC::ConstraintsFbLoadForces(double factor) {
    std::cerr << "ChContactContainerSMC::ConstraintsFbLoadForces OBSOLETE - use new bookkeeping!" << std::endl;
}

void ChContactContainerSMC::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChContactContainerSMC>();
    // serialize parent class
    ChContactContainer::ArchiveOut(archive_out);
    // serialize all member data:
    // NO SERIALIZATION of contact list because assume it is volatile and generated when needed
}

/// Method to allow de serialization of transient data from archives.
void ChContactContainerSMC::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChContactContainerSMC>();
    // deserialize parent class
    ChContactContainer::ArchiveIn(archive_in);
    // stream in all member data:
    RemoveAllContacts();
    // NO SERIALIZATION of contact list because assume it is volatile and generated when needed
}

}  // end namespace chrono
