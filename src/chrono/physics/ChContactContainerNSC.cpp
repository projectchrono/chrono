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
#include "chrono/solver/ChConstraintTwoTuplesContactN.h"

namespace chrono {

using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChContactContainerNSC)

ChContactContainerNSC::ChContactContainerNSC()
    : n_added_6_6(0),
      n_added_6_3(0),
      n_added_3_3(0),
      n_added_333_3(0),
      n_added_333_6(0),
      n_added_333_333(0),
      n_added_666_3(0),
      n_added_666_6(0),
      n_added_666_333(0),
      n_added_666_666(0),
      n_added_6_6_rolling(0) {}

ChContactContainerNSC::ChContactContainerNSC(const ChContactContainerNSC& other) : ChContactContainer(other) {
    n_added_6_6 = 0;
    n_added_6_3 = 0;
    n_added_3_3 = 0;
    n_added_333_3 = 0;
    n_added_333_6 = 0;
    n_added_333_333 = 0;
    n_added_666_3 = 0;
    n_added_666_6 = 0;
    n_added_666_333 = 0;
    n_added_666_666 = 0;
    n_added_6_6_rolling = 0;
}

ChContactContainerNSC::~ChContactContainerNSC() {
    RemoveAllContacts();
}

void ChContactContainerNSC::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class, basically doing nothing :)
    ChContactContainer::Update(mytime, update_assets);
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

void ChContactContainerNSC::RemoveAllContacts() {
    _RemoveAllContacts(contactlist_6_6, lastcontact_6_6, n_added_6_6);
    _RemoveAllContacts(contactlist_6_3, lastcontact_6_3, n_added_6_3);
    _RemoveAllContacts(contactlist_3_3, lastcontact_3_3, n_added_3_3);
    _RemoveAllContacts(contactlist_333_3, lastcontact_333_3, n_added_333_3);
    _RemoveAllContacts(contactlist_333_6, lastcontact_333_6, n_added_333_6);
    _RemoveAllContacts(contactlist_333_333, lastcontact_333_333, n_added_333_333);
    _RemoveAllContacts(contactlist_666_3, lastcontact_666_3, n_added_666_3);
    _RemoveAllContacts(contactlist_666_6, lastcontact_666_6, n_added_666_6);
    _RemoveAllContacts(contactlist_666_333, lastcontact_666_333, n_added_666_333);
    _RemoveAllContacts(contactlist_666_666, lastcontact_666_666, n_added_666_666);
    _RemoveAllContacts(contactlist_6_6_rolling, lastcontact_6_6_rolling, n_added_6_6_rolling);
}

void ChContactContainerNSC::BeginAddContact() {
    lastcontact_6_6 = contactlist_6_6.begin();
    n_added_6_6 = 0;

    lastcontact_6_3 = contactlist_6_3.begin();
    n_added_6_3 = 0;

    lastcontact_3_3 = contactlist_3_3.begin();
    n_added_3_3 = 0;

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

    lastcontact_6_6_rolling = contactlist_6_6_rolling.begin();
    n_added_6_6_rolling = 0;
}

void ChContactContainerNSC::EndAddContact() {
    // remove contacts that are beyond last contact
    while (lastcontact_6_6 != contactlist_6_6.end()) {
        delete (*lastcontact_6_6);
        lastcontact_6_6 = contactlist_6_6.erase(lastcontact_6_6);
    }
    while (lastcontact_6_3 != contactlist_6_3.end()) {
        delete (*lastcontact_6_3);
        lastcontact_6_3 = contactlist_6_3.erase(lastcontact_6_3);
    }
    while (lastcontact_3_3 != contactlist_3_3.end()) {
        delete (*lastcontact_3_3);
        lastcontact_3_3 = contactlist_3_3.erase(lastcontact_3_3);
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

    while (lastcontact_6_6_rolling != contactlist_6_6_rolling.end()) {
        delete (*lastcontact_6_6_rolling);
        lastcontact_6_6_rolling = contactlist_6_6_rolling.erase(lastcontact_6_6_rolling);
    }
}

template <class Tcont, class Titer, class Ta, class Tb>
void _OptimalContactInsert(std::list<Tcont*>& contactlist,          ///< contact list
                           Titer& lastcontact,                      ///< last contact acquired
                           int& n_added,                            ///< number of contact inserted
                           ChContactContainer* mcontainer,          ///< contact container
                           Ta* objA,                                ///< collidable object A
                           Tb* objB,                                ///< collidable object B
                           const collision::ChCollisionInfo& cinfo  ///< collision informations
) {
    if (lastcontact != contactlist.end()) {
        // reuse old contacts
        (*lastcontact)->Reset(objA, objB, cinfo);
        lastcontact++;

    } else {
        // add new contact
        Tcont* mc = new Tcont(mcontainer, objA, objB, cinfo);
        contactlist.push_back(mc);
        lastcontact = contactlist.end();
    }
    n_added++;
}

void ChContactContainerNSC::AddContact(const collision::ChCollisionInfo& mcontact) {
    assert(mcontact.modelA->GetContactable());
    assert(mcontact.modelB->GetContactable());

    auto contactableA = mcontact.modelA->GetContactable();
    auto contactableB = mcontact.modelB->GetContactable();

    // Bail out if any of the two contactable objects is not contact-active:
    bool inactiveA = !contactableA->IsContactActive();
    bool inactiveB = !contactableB->IsContactActive();
    if (inactiveA && inactiveB)
        return;

    // Check if both collision models use NSC ('non-smooth dynamics') materials.
    // If not NSC vs NSC, just bailout (ex it could be that this was a SMC vs SMC contact)
    if (contactableA->GetMaterialSurface()->GetContactMethod() != ChMaterialSurface::NSC ||
        contactableB->GetMaterialSurface()->GetContactMethod() != ChMaterialSurface::NSC)
        return;

    auto mmatA = std::static_pointer_cast<ChMaterialSurfaceNSC>(contactableA->GetMaterialSurface());
    auto mmatB = std::static_pointer_cast<ChMaterialSurfaceNSC>(contactableB->GetMaterialSurface());

    // CREATE THE CONTACTS
    //
    // Switch among the various cases of contacts: i.e. between a 6-dof variable and another 6-dof variable,
    // or 6 vs 3, etc.
    // These cases are made distinct to exploit the optimization coming from templates and static data sizes
    // in contact types.
    //
    // Notes:
    // 1. this was formerly implemented using dynamic casting and introduced a performance bottleneck.
    // 2. use a switch only for the outer level (nested switch negatively affects performance)

    switch (contactableA->GetContactableType()) {
        case ChContactable::CONTACTABLE_3: {
            auto mmboA = static_cast<ChContactable_1vars<3>*>(contactableA);
            if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_3) {
                auto mmboB = static_cast<ChContactable_1vars<3>*>(contactableB);
                // 3_3
                _OptimalContactInsert(contactlist_3_3, lastcontact_3_3, n_added_3_3, this, mmboA, mmboB, mcontact);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_6) {
                auto mmboB = static_cast<ChContactable_1vars<6>*>(contactableB);
                // 3_6 -> 6_3
                collision::ChCollisionInfo swapped_contact(mcontact, true);
                _OptimalContactInsert(contactlist_6_3, lastcontact_6_3, n_added_6_3, this, mmboB, mmboA, swapped_contact);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_333) {
                auto mmboB = static_cast<ChContactable_3vars<3, 3, 3>*>(contactableB);
                // 3_333 -> 333_3
                collision::ChCollisionInfo swapped_contact(mcontact, true);
                _OptimalContactInsert(contactlist_333_3, lastcontact_333_3, n_added_333_3, this, mmboB, mmboA, swapped_contact);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_666) {
                auto mmboB = static_cast<ChContactable_3vars<6, 6, 6>*>(contactableB);
                // 3_666 -> 666_3
                collision::ChCollisionInfo swapped_contact(mcontact, true);
                _OptimalContactInsert(contactlist_666_3, lastcontact_666_3, n_added_666_3, this, mmboB, mmboA, swapped_contact);
            }
        } break;

        case ChContactable::CONTACTABLE_6: {
            auto mmboA = static_cast<ChContactable_1vars<6>*>(contactableA);
            if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_3) {
                auto mmboB = static_cast<ChContactable_1vars<3>*>(contactableB);
                // 6_3
                _OptimalContactInsert(contactlist_6_3, lastcontact_6_3, n_added_6_3, this, mmboA, mmboB, mcontact);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_6) {
                auto mmboB = static_cast<ChContactable_1vars<6>*>(contactableB);
                // 6_6    ***NOTE: for body-body one could have rolling friction: ***
                if ((mmatA->rolling_friction && mmatB->rolling_friction) ||
                    (mmatA->spinning_friction && mmatB->spinning_friction)) {
                    _OptimalContactInsert(contactlist_6_6_rolling, lastcontact_6_6_rolling, n_added_6_6_rolling, this, mmboA, mmboB, mcontact);
                } else {
                    _OptimalContactInsert(contactlist_6_6, lastcontact_6_6, n_added_6_6, this, mmboA, mmboB, mcontact);
                }
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_333) {
                auto mmboB = static_cast<ChContactable_3vars<3, 3, 3>*>(contactableB);
                // 6_333 -> 333_6
                collision::ChCollisionInfo swapped_contact(mcontact, true);
                _OptimalContactInsert(contactlist_333_6, lastcontact_333_6, n_added_333_6, this, mmboB, mmboA, swapped_contact);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_666) {
                auto mmboB = static_cast<ChContactable_3vars<6, 6, 6>*>(contactableB);
                // 6_666 -> 666_6
                collision::ChCollisionInfo swapped_contact(mcontact, true);
                _OptimalContactInsert(contactlist_666_6, lastcontact_666_6, n_added_666_6, this, mmboB, mmboA, swapped_contact);
            }
        } break;

        case ChContactable::CONTACTABLE_333: {
            auto mmboA = static_cast<ChContactable_3vars<3, 3, 3>*>(contactableA);
            if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_3) {
                auto mmboB = static_cast<ChContactable_1vars<3>*>(contactableB);
                // 333_3
                _OptimalContactInsert(contactlist_333_3, lastcontact_333_3, n_added_333_3, this, mmboA, mmboB, mcontact);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_6) {
                auto mmboB = static_cast<ChContactable_1vars<6>*>(contactableB);
                // 333_6
                _OptimalContactInsert(contactlist_333_6, lastcontact_333_6, n_added_333_6, this, mmboA, mmboB, mcontact);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_333) {
                auto mmboB = static_cast<ChContactable_3vars<3, 3, 3>*>(contactableB);
                // 333_333
                _OptimalContactInsert(contactlist_333_333, lastcontact_333_333, n_added_333_333, this, mmboA, mmboB, mcontact);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_666) {
                auto mmboB = static_cast<ChContactable_3vars<6, 6, 6>*>(contactableB);
                // 333_666 -> 666_333
                collision::ChCollisionInfo swapped_contact(mcontact, true);
                _OptimalContactInsert(contactlist_666_333, lastcontact_666_333, n_added_666_333, this, mmboB, mmboA, swapped_contact);
            }
        } break;

        case ChContactable::CONTACTABLE_666: {
            auto mmboA = static_cast<ChContactable_3vars<6, 6, 6>*>(contactableA);
            if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_3) {
                auto mmboB = static_cast<ChContactable_1vars<3>*>(contactableB);
                // 666_3
                _OptimalContactInsert(contactlist_666_3, lastcontact_666_3, n_added_666_3, this, mmboA, mmboB, mcontact);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_6) {
                auto mmboB = static_cast<ChContactable_1vars<6>*>(contactableB);
                // 666_6
                _OptimalContactInsert(contactlist_666_6, lastcontact_666_6, n_added_666_6, this, mmboA, mmboB, mcontact);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_333) {
                auto mmboB = static_cast<ChContactable_3vars<3, 3, 3>*>(contactableB);
                // 666_333
                _OptimalContactInsert(contactlist_666_333, lastcontact_666_333, n_added_666_333, this, mmboA, mmboB, mcontact);
            } else if (contactableB->GetContactableType() == ChContactable::CONTACTABLE_666) {
                auto mmboB = static_cast<ChContactable_3vars<6, 6, 6>*>(contactableB);
                // 666_666
                _OptimalContactInsert(contactlist_666_666, lastcontact_666_666, n_added_666_666, this, mmboA, mmboB, mcontact);
            }
        } break;

    }  // switch (contactableA->GetContactableType())

    // ***TODO*** Fallback to some dynamic-size allocated constraint for cases that were not trapped by the switch
}

void ChContactContainerNSC::ComputeContactForces() {
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
    SumAllContactForces(contactlist_6_6_rolling, contact_forces);
}

ChVector<> ChContactContainerNSC::GetContactableForce(ChContactable* contactable) {
    std::unordered_map<ChContactable*, ForceTorque>::const_iterator Iterator = contact_forces.find(contactable);
    if (Iterator != contact_forces.end()) {
        return Iterator->second.force;
    }
    return ChVector<>(0);
}

ChVector<> ChContactContainerNSC::GetContactableTorque(ChContactable* contactable) {
    std::unordered_map<ChContactable*, ForceTorque>::const_iterator Iterator = contact_forces.find(contactable);
    if (Iterator != contact_forces.end()) {
        return Iterator->second.torque;
    }
    return ChVector<>(0);
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

template <class Tcont>
void _ReportAllContactsRolling(std::list<Tcont*>& contactlist, ChContactContainer::ReportContactCallback* mcallback) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        bool proceed = mcallback->OnReportContact(
            (*itercontact)->GetContactP1(), (*itercontact)->GetContactP2(), (*itercontact)->GetContactPlane(),
            (*itercontact)->GetContactDistance(), (*itercontact)->GetEffectiveCurvatureRadius(),
            (*itercontact)->GetContactForce(), (*itercontact)->GetContactTorque(), (*itercontact)->GetObjA(),
            (*itercontact)->GetObjB());
        if (!proceed)
            break;
        ++itercontact;
    }
}

void ChContactContainerNSC::ReportAllContacts(ReportContactCallback* mcallback) {
    _ReportAllContacts(contactlist_6_6, mcallback);
    _ReportAllContacts(contactlist_6_3, mcallback);
    _ReportAllContacts(contactlist_3_3, mcallback);
    _ReportAllContacts(contactlist_333_3, mcallback);
    _ReportAllContacts(contactlist_333_6, mcallback);
    _ReportAllContacts(contactlist_333_333, mcallback);
    _ReportAllContacts(contactlist_666_3, mcallback);
    _ReportAllContacts(contactlist_666_6, mcallback);
    _ReportAllContacts(contactlist_666_333, mcallback);
    _ReportAllContacts(contactlist_666_666, mcallback);
    _ReportAllContactsRolling(contactlist_6_6_rolling, mcallback);
}

////////// STATE INTERFACE ////

template <class Tcont>
void _IntStateGatherReactions(unsigned int& coffset,
                              std::list<Tcont*>& contactlist,
                              const unsigned int off_L,
                              ChVectorDynamic<>& L,
                              const int stride) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContIntStateGatherReactions(off_L + coffset, L);
        coffset += stride;
        ++itercontact;
    }
}

void ChContactContainerNSC::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    unsigned int coffset = 0;
    _IntStateGatherReactions(coffset, contactlist_6_6, off_L, L, 3);
    _IntStateGatherReactions(coffset, contactlist_6_3, off_L, L, 3);
    _IntStateGatherReactions(coffset, contactlist_3_3, off_L, L, 3);
    _IntStateGatherReactions(coffset, contactlist_333_3, off_L, L, 3);
    _IntStateGatherReactions(coffset, contactlist_333_6, off_L, L, 3);
    _IntStateGatherReactions(coffset, contactlist_333_333, off_L, L, 3);
    _IntStateGatherReactions(coffset, contactlist_666_3, off_L, L, 3);
    _IntStateGatherReactions(coffset, contactlist_666_6, off_L, L, 3);
    _IntStateGatherReactions(coffset, contactlist_666_333, off_L, L, 3);
    _IntStateGatherReactions(coffset, contactlist_666_666, off_L, L, 3);
    _IntStateGatherReactions(coffset, contactlist_6_6_rolling, off_L, L, 6);
}

template <class Tcont>
void _IntStateScatterReactions(unsigned int& coffset,
                               std::list<Tcont*>& contactlist,
                               const unsigned int off_L,
                               const ChVectorDynamic<>& L,
                               const int stride) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContIntStateScatterReactions(off_L + coffset, L);
        coffset += stride;
        ++itercontact;
    }
}

void ChContactContainerNSC::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    unsigned int coffset = 0;
    _IntStateScatterReactions(coffset, contactlist_6_6, off_L, L, 3);
    _IntStateScatterReactions(coffset, contactlist_6_3, off_L, L, 3);
    _IntStateScatterReactions(coffset, contactlist_3_3, off_L, L, 3);
    _IntStateScatterReactions(coffset, contactlist_333_3, off_L, L, 3);
    _IntStateScatterReactions(coffset, contactlist_333_6, off_L, L, 3);
    _IntStateScatterReactions(coffset, contactlist_333_333, off_L, L, 3);
    _IntStateScatterReactions(coffset, contactlist_666_3, off_L, L, 3);
    _IntStateScatterReactions(coffset, contactlist_666_6, off_L, L, 3);
    _IntStateScatterReactions(coffset, contactlist_666_333, off_L, L, 3);
    _IntStateScatterReactions(coffset, contactlist_666_666, off_L, L, 3);
    _IntStateScatterReactions(coffset, contactlist_6_6_rolling, off_L, L, 6);
}

template <class Tcont>
void _IntLoadResidual_CqL(unsigned int& coffset,           ///< offset of the contacts
                          std::list<Tcont*>& contactlist,  ///< list of contacts
                          const unsigned int off_L,        ///< offset in L multipliers
                          ChVectorDynamic<>& R,            ///< result: the R residual, R += c*Cq'*L
                          const ChVectorDynamic<>& L,      ///< the L vector
                          const double c,                  ///< a scaling factor
                          const int stride                 ///< stride
) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContIntLoadResidual_CqL(off_L + coffset, R, L, c);
        coffset += stride;
        ++itercontact;
    }
}

void ChContactContainerNSC::IntLoadResidual_CqL(const unsigned int off_L,
                                                ChVectorDynamic<>& R,
                                                const ChVectorDynamic<>& L,
                                                const double c) {
    unsigned int coffset = 0;
    _IntLoadResidual_CqL(coffset, contactlist_6_6, off_L, R, L, c, 3);
    _IntLoadResidual_CqL(coffset, contactlist_6_3, off_L, R, L, c, 3);
    _IntLoadResidual_CqL(coffset, contactlist_3_3, off_L, R, L, c, 3);
    _IntLoadResidual_CqL(coffset, contactlist_333_3, off_L, R, L, c, 3);
    _IntLoadResidual_CqL(coffset, contactlist_333_6, off_L, R, L, c, 3);
    _IntLoadResidual_CqL(coffset, contactlist_333_333, off_L, R, L, c, 3);
    _IntLoadResidual_CqL(coffset, contactlist_666_3, off_L, R, L, c, 3);
    _IntLoadResidual_CqL(coffset, contactlist_666_6, off_L, R, L, c, 3);
    _IntLoadResidual_CqL(coffset, contactlist_666_333, off_L, R, L, c, 3);
    _IntLoadResidual_CqL(coffset, contactlist_666_666, off_L, R, L, c, 3);
    _IntLoadResidual_CqL(coffset, contactlist_6_6_rolling, off_L, R, L, c, 6);
}

template <class Tcont>
void _IntLoadConstraint_C(unsigned int& coffset,           ///< contact offset
                          std::list<Tcont*>& contactlist,  ///< contact list
                          const unsigned int off,          ///< offset in Qc residual
                          ChVectorDynamic<>& Qc,           ///< result: the Qc residual, Qc += c*C
                          const double c,                  ///< a scaling factor
                          bool do_clamp,                   ///< apply clamping to c*C?
                          double recovery_clamp,           ///< value for min/max clamping of c*C
                          const int stride                 ///< stride
) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContIntLoadConstraint_C(off + coffset, Qc, c, do_clamp, recovery_clamp);
        coffset += stride;
        ++itercontact;
    }
}

void ChContactContainerNSC::IntLoadConstraint_C(const unsigned int off,
                                                ChVectorDynamic<>& Qc,
                                                const double c,
                                                bool do_clamp,
                                                double recovery_clamp) {
    unsigned int coffset = 0;
    _IntLoadConstraint_C(coffset, contactlist_6_6, off, Qc, c, do_clamp, recovery_clamp, 3);
    _IntLoadConstraint_C(coffset, contactlist_6_3, off, Qc, c, do_clamp, recovery_clamp, 3);
    _IntLoadConstraint_C(coffset, contactlist_3_3, off, Qc, c, do_clamp, recovery_clamp, 3);
    _IntLoadConstraint_C(coffset, contactlist_333_3, off, Qc, c, do_clamp, recovery_clamp, 3);
    _IntLoadConstraint_C(coffset, contactlist_333_6, off, Qc, c, do_clamp, recovery_clamp, 3);
    _IntLoadConstraint_C(coffset, contactlist_333_333, off, Qc, c, do_clamp, recovery_clamp, 3);
    _IntLoadConstraint_C(coffset, contactlist_666_3, off, Qc, c, do_clamp, recovery_clamp, 3);
    _IntLoadConstraint_C(coffset, contactlist_666_6, off, Qc, c, do_clamp, recovery_clamp, 3);
    _IntLoadConstraint_C(coffset, contactlist_666_333, off, Qc, c, do_clamp, recovery_clamp, 3);
    _IntLoadConstraint_C(coffset, contactlist_666_666, off, Qc, c, do_clamp, recovery_clamp, 3);
    _IntLoadConstraint_C(coffset, contactlist_6_6_rolling, off, Qc, c, do_clamp, recovery_clamp, 6);
}

template <class Tcont>
void _IntToDescriptor(unsigned int& coffset,
                      std::list<Tcont*>& contactlist,
                      const unsigned int off_v,
                      const ChStateDelta& v,
                      const ChVectorDynamic<>& R,
                      const unsigned int off_L,
                      const ChVectorDynamic<>& L,
                      const ChVectorDynamic<>& Qc,
                      const int stride) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContIntToDescriptor(off_L + coffset, L, Qc);
        coffset += stride;
        ++itercontact;
    }
}

void ChContactContainerNSC::IntToDescriptor(const unsigned int off_v,
                                            const ChStateDelta& v,
                                            const ChVectorDynamic<>& R,
                                            const unsigned int off_L,
                                            const ChVectorDynamic<>& L,
                                            const ChVectorDynamic<>& Qc) {
    unsigned int coffset = 0;
    _IntToDescriptor(coffset, contactlist_6_6, off_v, v, R, off_L, L, Qc, 3);
    _IntToDescriptor(coffset, contactlist_6_3, off_v, v, R, off_L, L, Qc, 3);
    _IntToDescriptor(coffset, contactlist_3_3, off_v, v, R, off_L, L, Qc, 3);
    _IntToDescriptor(coffset, contactlist_333_3, off_v, v, R, off_L, L, Qc, 3);
    _IntToDescriptor(coffset, contactlist_333_6, off_v, v, R, off_L, L, Qc, 3);
    _IntToDescriptor(coffset, contactlist_333_333, off_v, v, R, off_L, L, Qc, 3);
    _IntToDescriptor(coffset, contactlist_666_3, off_v, v, R, off_L, L, Qc, 3);
    _IntToDescriptor(coffset, contactlist_666_6, off_v, v, R, off_L, L, Qc, 3);
    _IntToDescriptor(coffset, contactlist_666_333, off_v, v, R, off_L, L, Qc, 3);
    _IntToDescriptor(coffset, contactlist_666_666, off_v, v, R, off_L, L, Qc, 3);
    _IntToDescriptor(coffset, contactlist_6_6_rolling, off_v, v, R, off_L, L, Qc, 6);
}

template <class Tcont>
void _IntFromDescriptor(unsigned int& coffset,
                        std::list<Tcont*>& contactlist,
                        const unsigned int off_v,
                        ChStateDelta& v,
                        const unsigned int off_L,
                        ChVectorDynamic<>& L,
                        const int stride) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContIntFromDescriptor(off_L + coffset, L);
        coffset += stride;
        ++itercontact;
    }
}

void ChContactContainerNSC::IntFromDescriptor(const unsigned int off_v,
                                              ChStateDelta& v,
                                              const unsigned int off_L,
                                              ChVectorDynamic<>& L) {
    unsigned int coffset = 0;
    _IntFromDescriptor(coffset, contactlist_6_6, off_v, v, off_L, L, 3);
    _IntFromDescriptor(coffset, contactlist_6_3, off_v, v, off_L, L, 3);
    _IntFromDescriptor(coffset, contactlist_3_3, off_v, v, off_L, L, 3);
    _IntFromDescriptor(coffset, contactlist_333_3, off_v, v, off_L, L, 3);
    _IntFromDescriptor(coffset, contactlist_333_6, off_v, v, off_L, L, 3);
    _IntFromDescriptor(coffset, contactlist_333_333, off_v, v, off_L, L, 3);
    _IntFromDescriptor(coffset, contactlist_666_3, off_v, v, off_L, L, 3);
    _IntFromDescriptor(coffset, contactlist_666_6, off_v, v, off_L, L, 3);
    _IntFromDescriptor(coffset, contactlist_666_333, off_v, v, off_L, L, 3);
    _IntFromDescriptor(coffset, contactlist_666_666, off_v, v, off_L, L, 3);
    _IntFromDescriptor(coffset, contactlist_6_6_rolling, off_v, v, off_L, L, 6);
}

// SOLVER INTERFACES

template <class Tcont>
void _InjectConstraints(std::list<Tcont*>& contactlist, ChSystemDescriptor& mdescriptor) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->InjectConstraints(mdescriptor);
        ++itercontact;
    }
}

void ChContactContainerNSC::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    _InjectConstraints(contactlist_6_6, mdescriptor);
    _InjectConstraints(contactlist_6_3, mdescriptor);
    _InjectConstraints(contactlist_3_3, mdescriptor);
    _InjectConstraints(contactlist_333_3, mdescriptor);
    _InjectConstraints(contactlist_333_6, mdescriptor);
    _InjectConstraints(contactlist_333_333, mdescriptor);
    _InjectConstraints(contactlist_666_3, mdescriptor);
    _InjectConstraints(contactlist_666_6, mdescriptor);
    _InjectConstraints(contactlist_666_333, mdescriptor);
    _InjectConstraints(contactlist_666_666, mdescriptor);
    _InjectConstraints(contactlist_6_6_rolling, mdescriptor);
}

template <class Tcont>
void _ConstraintsBiReset(std::list<Tcont*>& contactlist) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ConstraintsBiReset();
        ++itercontact;
    }
}

void ChContactContainerNSC::ConstraintsBiReset() {
    _ConstraintsBiReset(contactlist_6_6);
    _ConstraintsBiReset(contactlist_6_3);
    _ConstraintsBiReset(contactlist_3_3);
    _ConstraintsBiReset(contactlist_333_3);
    _ConstraintsBiReset(contactlist_333_6);
    _ConstraintsBiReset(contactlist_333_333);
    _ConstraintsBiReset(contactlist_666_3);
    _ConstraintsBiReset(contactlist_666_6);
    _ConstraintsBiReset(contactlist_666_333);
    _ConstraintsBiReset(contactlist_666_666);
    _ConstraintsBiReset(contactlist_6_6_rolling);
}

template <class Tcont>
void _ConstraintsBiLoad_C(std::list<Tcont*>& contactlist, double factor, double recovery_clamp, bool do_clamp) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
        ++itercontact;
    }
}

void ChContactContainerNSC::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    _ConstraintsBiLoad_C(contactlist_6_6, factor, recovery_clamp, do_clamp);
    _ConstraintsBiLoad_C(contactlist_6_3, factor, recovery_clamp, do_clamp);
    _ConstraintsBiLoad_C(contactlist_3_3, factor, recovery_clamp, do_clamp);
    _ConstraintsBiLoad_C(contactlist_333_3, factor, recovery_clamp, do_clamp);
    _ConstraintsBiLoad_C(contactlist_333_6, factor, recovery_clamp, do_clamp);
    _ConstraintsBiLoad_C(contactlist_333_333, factor, recovery_clamp, do_clamp);
    _ConstraintsBiLoad_C(contactlist_666_3, factor, recovery_clamp, do_clamp);
    _ConstraintsBiLoad_C(contactlist_666_6, factor, recovery_clamp, do_clamp);
    _ConstraintsBiLoad_C(contactlist_666_333, factor, recovery_clamp, do_clamp);
    _ConstraintsBiLoad_C(contactlist_666_666, factor, recovery_clamp, do_clamp);
    _ConstraintsBiLoad_C(contactlist_6_6_rolling, factor, recovery_clamp, do_clamp);
}

void ChContactContainerNSC::ConstraintsLoadJacobians() {
    // already loaded when contact objects are created
}

template <class Tcont>
void _ConstraintsFetch_react(std::list<Tcont*>& contactlist, double factor) {
    // From constraints to react vector:
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ConstraintsFetch_react(factor);
        ++itercontact;
    }
}

void ChContactContainerNSC::ConstraintsFetch_react(double factor) {
    _ConstraintsFetch_react(contactlist_6_6, factor);
    _ConstraintsFetch_react(contactlist_6_3, factor);
    _ConstraintsFetch_react(contactlist_3_3, factor);
    _ConstraintsFetch_react(contactlist_333_3, factor);
    _ConstraintsFetch_react(contactlist_333_6, factor);
    _ConstraintsFetch_react(contactlist_333_333, factor);
    _ConstraintsFetch_react(contactlist_666_3, factor);
    _ConstraintsFetch_react(contactlist_666_6, factor);
    _ConstraintsFetch_react(contactlist_666_333, factor);
    _ConstraintsFetch_react(contactlist_666_666, factor);
    _ConstraintsFetch_react(contactlist_6_6_rolling, factor);
}

void ChContactContainerNSC::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChContactContainerNSC>();
    // serialize parent class
    ChContactContainer::ArchiveOUT(marchive);
    // serialize all member data:
    // NO SERIALIZATION of contact list because assume it is volatile and generated when needed
}

/// Method to allow de serialization of transient data from archives.
void ChContactContainerNSC::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChContactContainerNSC>();
    // deserialize parent class
    ChContactContainer::ArchiveIN(marchive);
    // stream in all member data:
    RemoveAllContacts();
    // NO SERIALIZATION of contact list because assume it is volatile and generated when needed
}
}  // end namespace chrono
