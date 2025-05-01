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
    : n_added_3_3(0),
      n_added_6_3(0),
      n_added_6_6(0),
      n_added_333_3(0),
      n_added_333_6(0),
      n_added_333_333(0),
      n_added_666_3(0),
      n_added_666_6(0),
      n_added_666_333(0),
      n_added_666_666(0),
      n_added_33_3(0),
      n_added_33_6(0),
      n_added_33_333(0),
      n_added_33_666(0),
      n_added_33_33(0),
      n_added_66_3(0),
      n_added_66_6(0),
      n_added_66_333(0),
      n_added_66_666(0),
      n_added_66_33(0),
      n_added_66_66(0) {}

ChContactContainerSMC::ChContactContainerSMC(const ChContactContainerSMC& other)
    : ChContactContainer(other),
      n_added_3_3(0),
      n_added_6_3(0),
      n_added_6_6(0),
      n_added_333_3(0),
      n_added_333_6(0),
      n_added_333_333(0),
      n_added_666_3(0),
      n_added_666_6(0),
      n_added_666_333(0),
      n_added_666_666(0),
      n_added_33_3(0),
      n_added_33_6(0),
      n_added_33_333(0),
      n_added_33_666(0),
      n_added_33_33(0),
      n_added_66_3(0),
      n_added_66_6(0),
      n_added_66_333(0),
      n_added_66_666(0),
      n_added_66_33(0),
      n_added_66_66(0) {}

ChContactContainerSMC::~ChContactContainerSMC() {
    RemoveAllContacts();
}

void ChContactContainerSMC::Update(double time, bool update_assets) {
    // Inherit time changes of parent class, basically doing nothing :)
    ChContactContainer::Update(time, update_assets);
}

template <class Tcont, class Titer>
void _RemoveAllContacts(std::list<Tcont*>& contactlist, Titer& lastcontact, int& n_added) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        delete (*itercontact);
        (*itercontact) = 0;
        ++itercontact;
    }
    contactlist.clear();
    lastcontact = contactlist.begin();
    n_added = 0;
}

void ChContactContainerSMC::RemoveAllContacts() {
    _RemoveAllContacts(contactlist_3_3, lastcontact_3_3, n_added_3_3);

    _RemoveAllContacts(contactlist_6_3, lastcontact_6_3, n_added_6_3);
    _RemoveAllContacts(contactlist_6_6, lastcontact_6_6, n_added_6_6);

    _RemoveAllContacts(contactlist_333_3, lastcontact_333_3, n_added_333_3);
    _RemoveAllContacts(contactlist_333_6, lastcontact_333_6, n_added_333_6);
    _RemoveAllContacts(contactlist_333_333, lastcontact_333_333, n_added_333_333);

    _RemoveAllContacts(contactlist_666_3, lastcontact_666_3, n_added_666_3);
    _RemoveAllContacts(contactlist_666_6, lastcontact_666_6, n_added_666_6);
    _RemoveAllContacts(contactlist_666_333, lastcontact_666_333, n_added_666_333);
    _RemoveAllContacts(contactlist_666_666, lastcontact_666_666, n_added_666_666);

    _RemoveAllContacts(contactlist_33_3, lastcontact_33_3, n_added_33_3);
    _RemoveAllContacts(contactlist_33_6, lastcontact_33_6, n_added_33_6);
    _RemoveAllContacts(contactlist_33_333, lastcontact_33_333, n_added_33_333);
    _RemoveAllContacts(contactlist_33_666, lastcontact_33_666, n_added_33_666);
    _RemoveAllContacts(contactlist_33_33, lastcontact_33_33, n_added_33_33);

    _RemoveAllContacts(contactlist_66_3, lastcontact_66_3, n_added_66_3);
    _RemoveAllContacts(contactlist_66_6, lastcontact_66_6, n_added_66_6);
    _RemoveAllContacts(contactlist_66_333, lastcontact_66_333, n_added_66_333);
    _RemoveAllContacts(contactlist_66_666, lastcontact_66_666, n_added_66_666);
    _RemoveAllContacts(contactlist_66_33, lastcontact_66_33, n_added_66_33);
    _RemoveAllContacts(contactlist_66_66, lastcontact_66_66, n_added_66_66);
}

void ChContactContainerSMC::BeginAddContact() {
    lastcontact_3_3 = contactlist_3_3.begin();
    n_added_3_3 = 0;

    lastcontact_6_3 = contactlist_6_3.begin();
    n_added_6_3 = 0;
    lastcontact_6_6 = contactlist_6_6.begin();
    n_added_6_6 = 0;

    lastcontact_333_3 = contactlist_333_3.begin();
    n_added_333_3 = 0;
    lastcontact_333_6 = contactlist_333_6.begin();
    n_added_333_6 = 0;
    lastcontact_333_333 = contactlist_333_333.begin();
    n_added_333_333 = 0;

    lastcontact_666_3 = contactlist_666_3.begin();
    n_added_666_3 = 0;
    lastcontact_666_6 = contactlist_666_6.begin();
    n_added_666_6 = 0;
    lastcontact_666_333 = contactlist_666_333.begin();
    n_added_666_333 = 0;
    lastcontact_666_666 = contactlist_666_666.begin();
    n_added_666_666 = 0;

    lastcontact_33_3 = contactlist_33_3.begin();
    n_added_33_3 = 0;
    lastcontact_33_6 = contactlist_33_6.begin();
    n_added_33_6 = 0;
    lastcontact_33_333 = contactlist_33_333.begin();
    n_added_33_333 = 0;
    lastcontact_33_666 = contactlist_33_666.begin();
    n_added_33_666 = 0;
    lastcontact_33_33 = contactlist_33_33.begin();
    n_added_33_33 = 0;

    lastcontact_66_3 = contactlist_66_3.begin();
    n_added_66_3 = 0;
    lastcontact_66_6 = contactlist_66_6.begin();
    n_added_66_6 = 0;
    lastcontact_66_333 = contactlist_66_333.begin();
    n_added_66_333 = 0;
    lastcontact_66_666 = contactlist_66_666.begin();
    n_added_66_666 = 0;
    lastcontact_66_33 = contactlist_66_33.begin();
    n_added_66_33 = 0;
    lastcontact_66_66 = contactlist_66_66.begin();
    n_added_66_66 = 0;
}

void ChContactContainerSMC::EndAddContact() {
    // remove contacts that are beyond last contact
    while (lastcontact_3_3 != contactlist_3_3.end()) {
        delete (*lastcontact_3_3);
        lastcontact_3_3 = contactlist_3_3.erase(lastcontact_3_3);
    }

    while (lastcontact_6_3 != contactlist_6_3.end()) {
        delete (*lastcontact_6_3);
        lastcontact_6_3 = contactlist_6_3.erase(lastcontact_6_3);
    }
    while (lastcontact_6_6 != contactlist_6_6.end()) {
        delete (*lastcontact_6_6);
        lastcontact_6_6 = contactlist_6_6.erase(lastcontact_6_6);
    }

    while (lastcontact_333_3 != contactlist_333_3.end()) {
        delete (*lastcontact_333_3);
        lastcontact_333_3 = contactlist_333_3.erase(lastcontact_333_3);
    }
    while (lastcontact_333_6 != contactlist_333_6.end()) {
        delete (*lastcontact_333_6);
        lastcontact_333_6 = contactlist_333_6.erase(lastcontact_333_6);
    }
    while (lastcontact_333_333 != contactlist_333_333.end()) {
        delete (*lastcontact_333_333);
        lastcontact_333_333 = contactlist_333_333.erase(lastcontact_333_333);
    }

    while (lastcontact_666_3 != contactlist_666_3.end()) {
        delete (*lastcontact_666_3);
        lastcontact_666_3 = contactlist_666_3.erase(lastcontact_666_3);
    }
    while (lastcontact_666_6 != contactlist_666_6.end()) {
        delete (*lastcontact_666_6);
        lastcontact_666_6 = contactlist_666_6.erase(lastcontact_666_6);
    }
    while (lastcontact_666_333 != contactlist_666_333.end()) {
        delete (*lastcontact_666_333);
        lastcontact_666_333 = contactlist_666_333.erase(lastcontact_666_333);
    }
    while (lastcontact_666_666 != contactlist_666_666.end()) {
        delete (*lastcontact_666_666);
        lastcontact_666_666 = contactlist_666_666.erase(lastcontact_666_666);
    }

    while (lastcontact_33_3 != contactlist_33_3.end()) {
        delete (*lastcontact_33_3);
        lastcontact_33_3 = contactlist_33_3.erase(lastcontact_33_3);
    }
    while (lastcontact_33_6 != contactlist_33_6.end()) {
        delete (*lastcontact_33_6);
        lastcontact_33_6 = contactlist_33_6.erase(lastcontact_33_6);
    }
    while (lastcontact_33_333 != contactlist_33_333.end()) {
        delete (*lastcontact_33_333);
        lastcontact_33_333 = contactlist_33_333.erase(lastcontact_33_333);
    }
    while (lastcontact_33_666 != contactlist_33_666.end()) {
        delete (*lastcontact_33_666);
        lastcontact_33_666 = contactlist_33_666.erase(lastcontact_33_666);
    }
    while (lastcontact_33_33 != contactlist_33_33.end()) {
        delete (*lastcontact_33_33);
        lastcontact_33_33 = contactlist_33_33.erase(lastcontact_33_33);
    }

    while (lastcontact_66_3 != contactlist_66_3.end()) {
        delete (*lastcontact_66_3);
        lastcontact_66_3 = contactlist_66_3.erase(lastcontact_66_3);
    }
    while (lastcontact_66_6 != contactlist_66_6.end()) {
        delete (*lastcontact_66_6);
        lastcontact_66_6 = contactlist_66_6.erase(lastcontact_66_6);
    }
    while (lastcontact_66_333 != contactlist_66_333.end()) {
        delete (*lastcontact_66_333);
        lastcontact_66_333 = contactlist_66_333.erase(lastcontact_66_333);
    }
    while (lastcontact_66_666 != contactlist_66_666.end()) {
        delete (*lastcontact_66_666);
        lastcontact_66_666 = contactlist_66_666.erase(lastcontact_66_666);
    }
    while (lastcontact_66_33 != contactlist_66_33.end()) {
        delete (*lastcontact_66_33);
        lastcontact_66_33 = contactlist_66_33.erase(lastcontact_66_33);
    }
    while (lastcontact_66_66 != contactlist_66_66.end()) {
        delete (*lastcontact_66_66);
        lastcontact_66_66 = contactlist_66_66.erase(lastcontact_66_66);
    }
}

template <class Tcont, class Titer, class Ta, class Tb>
void _OptimalContactInsert(std::list<Tcont*>& contactlist,            // contact list
                           Titer& lastcontact,                        // last contact acquired
                           int& n_added,                              // number of contacts inserted
                           ChContactContainerSMC* container,          // contact container
                           Ta* objA,                                  // collidable object A
                           Tb* objB,                                  // collidable object B
                           const ChCollisionInfo& cinfo,              // collision information
                           const ChContactMaterialCompositeSMC& cmat  // composite material
) {
    if (lastcontact != contactlist.end()) {
        // reuse old contacts
        (*lastcontact)->Reset(objA, objB, cinfo, cmat);
        lastcontact++;
    } else {
        // add new contact
        Tcont* mc = new Tcont(container, objA, objB, cinfo, cmat);
        contactlist.push_back(mc);
        lastcontact = contactlist.end();
    }
    n_added++;
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

    // CREATE THE CONTACTS
    //
    // Switch among the various cases of contacts: i.e. between a 6-dof variable and another 6-dof variable,
    // or 6 vs 3, etc.
    // These cases are made distinct to exploit the optimization coming from templates and static data sizes
    // in contact types.
    //
    // Notes:
    // 1. this was previously implemented using dynamic casting and introduced a performance bottleneck.
    // 2. use a switch only for the outer level (nested switch negatively affects performance)

    switch (contactableA->GetContactableType()) {
        case ChContactable::CONTACTABLE_3: {
            auto objA = static_cast<ChContactable_1vars<3>*>(contactableA);

            if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_3) {
                auto objB = static_cast<ChContactable_1vars<3>*>(contactableB);
                // 3_3
                _OptimalContactInsert(contactlist_3_3, lastcontact_3_3, n_added_3_3, this, objA, objB, cinfo, cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_6) {
                auto objB = static_cast<ChContactable_1vars<6>*>(contactableB);
                // 3_6 -> 6_3
                ChCollisionInfo swapped_cinfo(cinfo, true);
                _OptimalContactInsert(contactlist_6_3, lastcontact_6_3, n_added_6_3, this, objB, objA, swapped_cinfo,
                                      cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_333) {
                auto objB = static_cast<ChContactable_3vars<3, 3, 3>*>(contactableB);
                // 3_333 -> 333_3
                ChCollisionInfo swapped_cinfo(cinfo, true);
                _OptimalContactInsert(contactlist_333_3, lastcontact_333_3, n_added_333_3, this, objB, objA,
                                      swapped_cinfo, cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_666) {
                auto objB = static_cast<ChContactable_3vars<6, 6, 6>*>(contactableB);
                // 3_666 -> 666_3
                ChCollisionInfo swapped_cinfo(cinfo, true);
                _OptimalContactInsert(contactlist_666_3, lastcontact_666_3, n_added_666_3, this, objB, objA,
                                      swapped_cinfo, cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_33) {
                auto objB = static_cast<ChContactable_2vars<3, 3>*>(contactableB);
                // 3_33 -> 33_3
                ChCollisionInfo swapped_cinfo(cinfo, true);
                _OptimalContactInsert(contactlist_33_3, lastcontact_33_3, n_added_33_3, this, objB, objA, swapped_cinfo,
                                      cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_66) {
                auto objB = static_cast<ChContactable_2vars<6, 6>*>(contactableB);
                // 3_66 -> 66_3
                ChCollisionInfo swapped_cinfo(cinfo, true);
                _OptimalContactInsert(contactlist_66_3, lastcontact_66_3, n_added_66_3, this, objB, objA, swapped_cinfo,
                                      cmat);
            }
        } break;

        case ChContactable::CONTACTABLE_6: {
            auto objA = static_cast<ChContactable_1vars<6>*>(contactableA);

            if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_3) {
                auto objB = static_cast<ChContactable_1vars<3>*>(contactableB);
                // 6_3
                _OptimalContactInsert(contactlist_6_3, lastcontact_6_3, n_added_6_3, this, objA, objB, cinfo, cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_6) {
                auto objB = static_cast<ChContactable_1vars<6>*>(contactableB);
                // 6_6
                _OptimalContactInsert(contactlist_6_6, lastcontact_6_6, n_added_6_6, this, objA, objB, cinfo, cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_333) {
                auto objB = static_cast<ChContactable_3vars<3, 3, 3>*>(contactableB);
                // 6_333 -> 333_6
                ChCollisionInfo swapped_cinfo(cinfo, true);
                _OptimalContactInsert(contactlist_333_6, lastcontact_333_6, n_added_333_6, this, objB, objA,
                                      swapped_cinfo, cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_666) {
                auto objB = static_cast<ChContactable_3vars<6, 6, 6>*>(contactableB);
                // 6_666 -> 666_6
                ChCollisionInfo swapped_cinfo(cinfo, true);
                _OptimalContactInsert(contactlist_666_6, lastcontact_666_6, n_added_666_6, this, objB, objA,
                                      swapped_cinfo, cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_33) {
                auto objB = static_cast<ChContactable_2vars<3, 3>*>(contactableB);
                // 6_33 -> 33_6
                ChCollisionInfo swapped_cinfo(cinfo, true);
                _OptimalContactInsert(contactlist_33_6, lastcontact_33_6, n_added_33_6, this, objB, objA, swapped_cinfo,
                                      cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_66) {
                auto objB = static_cast<ChContactable_2vars<6, 6>*>(contactableB);
                // 6_66 -> 66_6
                ChCollisionInfo swapped_cinfo(cinfo, true);
                _OptimalContactInsert(contactlist_66_6, lastcontact_66_6, n_added_66_6, this, objB, objA, swapped_cinfo,
                                      cmat);
            }
        } break;

        case ChContactable::CONTACTABLE_333: {
            auto objA = static_cast<ChContactable_3vars<3, 3, 3>*>(contactableA);

            if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_3) {
                auto objB = static_cast<ChContactable_1vars<3>*>(contactableB);
                // 333_3
                _OptimalContactInsert(contactlist_333_3, lastcontact_333_3, n_added_333_3, this, objA, objB, cinfo,
                                      cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_6) {
                auto objB = static_cast<ChContactable_1vars<6>*>(contactableB);
                // 333_6
                _OptimalContactInsert(contactlist_333_6, lastcontact_333_6, n_added_333_6, this, objA, objB, cinfo,
                                      cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_333) {
                auto objB = static_cast<ChContactable_3vars<3, 3, 3>*>(contactableB);
                // 333_333
                _OptimalContactInsert(contactlist_333_333, lastcontact_333_333, n_added_333_333, this, objA, objB,
                                      cinfo, cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_666) {
                auto objB = static_cast<ChContactable_3vars<6, 6, 6>*>(contactableB);
                // 333_666 -> 666_333
                ChCollisionInfo swapped_cinfo(cinfo, true);
                _OptimalContactInsert(contactlist_666_333, lastcontact_666_333, n_added_666_333, this, objB, objA,
                                      swapped_cinfo, cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_33) {
                auto objB = static_cast<ChContactable_2vars<3, 3>*>(contactableB);
                // 333_33 -> 33_333
                ChCollisionInfo swapped_cinfo(cinfo, true);
                _OptimalContactInsert(contactlist_33_333, lastcontact_33_333, n_added_33_333, this, objB, objA,
                                      swapped_cinfo, cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_66) {
                auto objB = static_cast<ChContactable_2vars<6, 6>*>(contactableB);
                // 333_66 -> 66_333
                ChCollisionInfo swapped_cinfo(cinfo, true);
                _OptimalContactInsert(contactlist_66_333, lastcontact_66_333, n_added_66_333, this, objB, objA,
                                      swapped_cinfo, cmat);
            }
        } break;

        case ChContactable::CONTACTABLE_666: {
            auto objA = static_cast<ChContactable_3vars<6, 6, 6>*>(contactableA);
            if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_3) {
                auto objB = static_cast<ChContactable_1vars<3>*>(contactableB);
                // 666_3
                _OptimalContactInsert(contactlist_666_3, lastcontact_666_3, n_added_666_3, this, objA, objB, cinfo,
                                      cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_6) {
                auto objB = static_cast<ChContactable_1vars<6>*>(contactableB);
                // 666_6
                _OptimalContactInsert(contactlist_666_6, lastcontact_666_6, n_added_666_6, this, objA, objB, cinfo,
                                      cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_333) {
                auto objB = static_cast<ChContactable_3vars<3, 3, 3>*>(contactableB);
                // 666_333
                _OptimalContactInsert(contactlist_666_333, lastcontact_666_333, n_added_666_333, this, objA, objB,
                                      cinfo, cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_666) {
                auto objB = static_cast<ChContactable_3vars<6, 6, 6>*>(contactableB);
                // 666_666
                _OptimalContactInsert(contactlist_666_666, lastcontact_666_666, n_added_666_666, this, objA, objB,
                                      cinfo, cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_33) {
                auto objB = static_cast<ChContactable_2vars<3, 3>*>(contactableB);
                // 666_33 -> 33_666
                ChCollisionInfo swapped_cinfo(cinfo, true);
                _OptimalContactInsert(contactlist_33_666, lastcontact_33_666, n_added_33_666, this, objB, objA,
                                      swapped_cinfo, cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_66) {
                auto objB = static_cast<ChContactable_2vars<6, 6>*>(contactableB);
                // 666_66 -> 66_666
                ChCollisionInfo swapped_cinfo(cinfo, true);
                _OptimalContactInsert(contactlist_66_666, lastcontact_66_666, n_added_66_666, this, objB, objA,
                                      swapped_cinfo, cmat);
            }
        } break;

        case ChContactable::CONTACTABLE_33: {
            auto objA = static_cast<ChContactable_2vars<3, 3>*>(contactableA);

            if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_3) {
                auto objB = static_cast<ChContactable_1vars<3>*>(contactableB);
                // 33_3
                _OptimalContactInsert(contactlist_33_3, lastcontact_33_3, n_added_33_3, this, objA, objB, cinfo, cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_6) {
                auto objB = static_cast<ChContactable_1vars<6>*>(contactableB);
                // 33_6
                _OptimalContactInsert(contactlist_33_6, lastcontact_33_6, n_added_33_6, this, objA, objB, cinfo, cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_333) {
                auto objB = static_cast<ChContactable_3vars<3, 3, 3>*>(contactableB);
                // 33_333
                _OptimalContactInsert(contactlist_33_333, lastcontact_33_333, n_added_33_333, this, objA, objB, cinfo,
                                      cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_666) {
                auto objB = static_cast<ChContactable_3vars<6, 6, 6>*>(contactableB);
                // 33_666
                _OptimalContactInsert(contactlist_33_666, lastcontact_33_666, n_added_33_666, this, objA, objB, cinfo,
                                      cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_33) {
                auto objB = static_cast<ChContactable_2vars<3, 3>*>(contactableB);
                // 33_33
                _OptimalContactInsert(contactlist_33_33, lastcontact_33_33, n_added_33_33, this, objA, objB, cinfo,
                                      cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_66) {
                auto objB = static_cast<ChContactable_2vars<6, 6>*>(contactableB);
                // 33_66 -> 66_33
                ChCollisionInfo swapped_cinfo(cinfo, true);
                _OptimalContactInsert(contactlist_66_33, lastcontact_66_33, n_added_66_33, this, objB, objA,
                                      swapped_cinfo, cmat);
            }
        } break;

        case ChContactable::CONTACTABLE_66: {
            auto objA = static_cast<ChContactable_2vars<6, 6>*>(contactableA);

            if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_3) {
                auto objB = static_cast<ChContactable_1vars<3>*>(contactableB);
                // 66_3
                _OptimalContactInsert(contactlist_66_3, lastcontact_66_3, n_added_66_3, this, objA, objB, cinfo, cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_6) {
                auto objB = static_cast<ChContactable_1vars<6>*>(contactableB);
                // 66_6
                _OptimalContactInsert(contactlist_66_6, lastcontact_66_6, n_added_66_6, this, objA, objB, cinfo, cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_333) {
                auto objB = static_cast<ChContactable_3vars<3, 3, 3>*>(contactableB);
                // 66_333
                _OptimalContactInsert(contactlist_66_333, lastcontact_66_333, n_added_66_333, this, objA, objB, cinfo,
                                      cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_666) {
                auto objB = static_cast<ChContactable_3vars<6, 6, 6>*>(contactableB);
                // 66_666
                _OptimalContactInsert(contactlist_66_666, lastcontact_66_666, n_added_66_666, this, objA, objB, cinfo,
                                      cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_33) {
                auto objB = static_cast<ChContactable_2vars<3, 3>*>(contactableB);
                // 66_33
                _OptimalContactInsert(contactlist_66_33, lastcontact_66_33, n_added_66_33, this, objA, objB, cinfo,
                                      cmat);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_66) {
                auto objB = static_cast<ChContactable_2vars<6, 6>*>(contactableB);
                // 66_66
                _OptimalContactInsert(contactlist_66_66, lastcontact_66_66, n_added_66_66, this, objA, objB, cinfo,
                                      cmat);
            }
        } break;

        default: {
            //// TODO Fallback to some dynamic-size allocated constraint for cases that were not trapped by the switch
        } break;

    }  // switch(contactableA->GetContactableType())
}

void ChContactContainerSMC::ComputeContactForces() {
    contact_forces.clear();

    SumAllContactForces(contactlist_3_3, contact_forces);

    SumAllContactForces(contactlist_6_3, contact_forces);
    SumAllContactForces(contactlist_6_6, contact_forces);

    SumAllContactForces(contactlist_333_3, contact_forces);
    SumAllContactForces(contactlist_333_6, contact_forces);
    SumAllContactForces(contactlist_333_333, contact_forces);

    SumAllContactForces(contactlist_666_3, contact_forces);
    SumAllContactForces(contactlist_666_6, contact_forces);
    SumAllContactForces(contactlist_666_333, contact_forces);
    SumAllContactForces(contactlist_666_666, contact_forces);

    SumAllContactForces(contactlist_33_3, contact_forces);
    SumAllContactForces(contactlist_33_6, contact_forces);
    SumAllContactForces(contactlist_33_333, contact_forces);
    SumAllContactForces(contactlist_33_666, contact_forces);
    SumAllContactForces(contactlist_33_33, contact_forces);

    SumAllContactForces(contactlist_66_3, contact_forces);
    SumAllContactForces(contactlist_66_6, contact_forces);
    SumAllContactForces(contactlist_66_333, contact_forces);
    SumAllContactForces(contactlist_66_666, contact_forces);
    SumAllContactForces(contactlist_66_33, contact_forces);
    SumAllContactForces(contactlist_66_66, contact_forces);
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

template <class Tcont>
void _ReportAllContacts(std::list<Tcont*>& contactlist, ChContactContainer::ReportContactCallback* mcallback) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        bool proceed = mcallback->OnReportContact(
            (*itercontact)->GetContactP1(), (*itercontact)->GetContactP2(), (*itercontact)->GetContactPlane(),
            (*itercontact)->GetContactDistance(), (*itercontact)->GetEffectiveCurvatureRadius(),
            (*itercontact)->GetContactForce(), VNULL, (*itercontact)->GetObjA(), (*itercontact)->GetObjB());
        if (!proceed)
            break;
        ++itercontact;
    }
}

void ChContactContainerSMC::ReportAllContacts(std::shared_ptr<ReportContactCallback> callback) {
    _ReportAllContacts(contactlist_3_3, callback.get());

    _ReportAllContacts(contactlist_6_3, callback.get());
    _ReportAllContacts(contactlist_6_6, callback.get());

    _ReportAllContacts(contactlist_333_3, callback.get());
    _ReportAllContacts(contactlist_333_6, callback.get());
    _ReportAllContacts(contactlist_333_333, callback.get());

    _ReportAllContacts(contactlist_666_3, callback.get());
    _ReportAllContacts(contactlist_666_6, callback.get());
    _ReportAllContacts(contactlist_666_333, callback.get());
    _ReportAllContacts(contactlist_666_666, callback.get());

    _ReportAllContacts(contactlist_33_3, callback.get());
    _ReportAllContacts(contactlist_33_6, callback.get());
    _ReportAllContacts(contactlist_33_333, callback.get());
    _ReportAllContacts(contactlist_33_666, callback.get());
    _ReportAllContacts(contactlist_33_33, callback.get());

    _ReportAllContacts(contactlist_66_3, callback.get());
    _ReportAllContacts(contactlist_66_6, callback.get());
    _ReportAllContacts(contactlist_66_333, callback.get());
    _ReportAllContacts(contactlist_66_666, callback.get());
    _ReportAllContacts(contactlist_66_33, callback.get());
    _ReportAllContacts(contactlist_66_66, callback.get());
}

// STATE INTERFACE

template <class Tcont>
void _IntLoadResidual_F(std::list<Tcont*>& contactlist, ChVectorDynamic<>& R, const double c) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContIntLoadResidual_F(R, c);
        ++itercontact;
    }
}

void ChContactContainerSMC::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    _IntLoadResidual_F(contactlist_3_3, R, c);

    _IntLoadResidual_F(contactlist_6_3, R, c);
    _IntLoadResidual_F(contactlist_6_6, R, c);

    _IntLoadResidual_F(contactlist_333_3, R, c);
    _IntLoadResidual_F(contactlist_333_6, R, c);
    _IntLoadResidual_F(contactlist_333_333, R, c);

    _IntLoadResidual_F(contactlist_666_3, R, c);
    _IntLoadResidual_F(contactlist_666_6, R, c);
    _IntLoadResidual_F(contactlist_666_333, R, c);
    _IntLoadResidual_F(contactlist_666_666, R, c);

    _IntLoadResidual_F(contactlist_33_3, R, c);
    _IntLoadResidual_F(contactlist_33_6, R, c);
    _IntLoadResidual_F(contactlist_33_333, R, c);
    _IntLoadResidual_F(contactlist_33_666, R, c);
    _IntLoadResidual_F(contactlist_33_33, R, c);

    _IntLoadResidual_F(contactlist_66_3, R, c);
    _IntLoadResidual_F(contactlist_66_6, R, c);
    _IntLoadResidual_F(contactlist_66_333, R, c);
    _IntLoadResidual_F(contactlist_66_666, R, c);
    _IntLoadResidual_F(contactlist_66_33, R, c);
    _IntLoadResidual_F(contactlist_66_66, R, c);
}

template <class Tcont>
void _KRMmatricesLoad(std::list<Tcont*> contactlist, double Kfactor, double Rfactor) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContKRMmatricesLoad(Kfactor, Rfactor);
        ++itercontact;
    }
}

void ChContactContainerSMC::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    _KRMmatricesLoad(contactlist_3_3, Kfactor, Rfactor);

    _KRMmatricesLoad(contactlist_6_3, Kfactor, Rfactor);
    _KRMmatricesLoad(contactlist_6_6, Kfactor, Rfactor);

    _KRMmatricesLoad(contactlist_333_3, Kfactor, Rfactor);
    _KRMmatricesLoad(contactlist_333_6, Kfactor, Rfactor);
    _KRMmatricesLoad(contactlist_333_333, Kfactor, Rfactor);

    _KRMmatricesLoad(contactlist_666_3, Kfactor, Rfactor);
    _KRMmatricesLoad(contactlist_666_6, Kfactor, Rfactor);
    _KRMmatricesLoad(contactlist_666_333, Kfactor, Rfactor);
    _KRMmatricesLoad(contactlist_666_666, Kfactor, Rfactor);

    _KRMmatricesLoad(contactlist_33_3, Kfactor, Rfactor);
    _KRMmatricesLoad(contactlist_33_6, Kfactor, Rfactor);
    _KRMmatricesLoad(contactlist_33_333, Kfactor, Rfactor);
    _KRMmatricesLoad(contactlist_33_666, Kfactor, Rfactor);
    _KRMmatricesLoad(contactlist_33_33, Kfactor, Rfactor);

    _KRMmatricesLoad(contactlist_66_3, Kfactor, Rfactor);
    _KRMmatricesLoad(contactlist_66_6, Kfactor, Rfactor);
    _KRMmatricesLoad(contactlist_66_333, Kfactor, Rfactor);
    _KRMmatricesLoad(contactlist_66_666, Kfactor, Rfactor);
    _KRMmatricesLoad(contactlist_66_33, Kfactor, Rfactor);
    _KRMmatricesLoad(contactlist_66_66, Kfactor, Rfactor);
}

template <class Tcont>
void _InjectKRMmatrices(std::list<Tcont*> contactlist, ChSystemDescriptor& descriptor) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContInjectKRMmatrices(descriptor);
        ++itercontact;
    }
}

void ChContactContainerSMC::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    _InjectKRMmatrices(contactlist_3_3, descriptor);

    _InjectKRMmatrices(contactlist_6_3, descriptor);
    _InjectKRMmatrices(contactlist_6_6, descriptor);

    _InjectKRMmatrices(contactlist_333_3, descriptor);
    _InjectKRMmatrices(contactlist_333_6, descriptor);
    _InjectKRMmatrices(contactlist_333_333, descriptor);

    _InjectKRMmatrices(contactlist_666_3, descriptor);
    _InjectKRMmatrices(contactlist_666_6, descriptor);
    _InjectKRMmatrices(contactlist_666_333, descriptor);
    _InjectKRMmatrices(contactlist_666_666, descriptor);

    _InjectKRMmatrices(contactlist_33_3, descriptor);
    _InjectKRMmatrices(contactlist_33_6, descriptor);
    _InjectKRMmatrices(contactlist_33_333, descriptor);
    _InjectKRMmatrices(contactlist_33_666, descriptor);
    _InjectKRMmatrices(contactlist_33_33, descriptor);

    _InjectKRMmatrices(contactlist_66_3, descriptor);
    _InjectKRMmatrices(contactlist_66_6, descriptor);
    _InjectKRMmatrices(contactlist_66_333, descriptor);
    _InjectKRMmatrices(contactlist_66_666, descriptor);
    _InjectKRMmatrices(contactlist_66_33, descriptor);
    _InjectKRMmatrices(contactlist_66_66, descriptor);
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
