// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChContactContainerDVI.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/solver/ChConstraintTwoTuplesContactN.h"

namespace chrono {

using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChContactContainerDVI)

ChContactContainerDVI::ChContactContainerDVI()
    : n_added_6_6(0), n_added_6_3(0), n_added_3_3(0), n_added_6_6_rolling(0) {}

ChContactContainerDVI::ChContactContainerDVI(const ChContactContainerDVI& other) : ChContactContainerBase(other) {
    n_added_6_6 = 0;
    n_added_6_3 = 0;
    n_added_3_3 = 0;
    n_added_6_6_rolling = 0;
}

ChContactContainerDVI::~ChContactContainerDVI() {
    RemoveAllContacts();
}

void ChContactContainerDVI::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class, basically doing nothing :)
    ChContactContainerBase::Update(mytime, update_assets);
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

void ChContactContainerDVI::RemoveAllContacts() {
    _RemoveAllContacts(contactlist_6_6, lastcontact_6_6, n_added_6_6);
    _RemoveAllContacts(contactlist_6_3, lastcontact_6_3, n_added_6_3);
    _RemoveAllContacts(contactlist_3_3, lastcontact_3_3, n_added_3_3);
    _RemoveAllContacts(contactlist_6_6_rolling, lastcontact_6_6_rolling, n_added_6_6_rolling);
}

void ChContactContainerDVI::BeginAddContact() {
    lastcontact_6_6 = contactlist_6_6.begin();
    n_added_6_6 = 0;

    lastcontact_6_3 = contactlist_6_3.begin();
    n_added_6_3 = 0;

    lastcontact_3_3 = contactlist_3_3.begin();
    n_added_3_3 = 0;

    lastcontact_6_6_rolling = contactlist_6_6_rolling.begin();
    n_added_6_6_rolling = 0;
}

void ChContactContainerDVI::EndAddContact() {
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
    while (lastcontact_6_6_rolling != contactlist_6_6_rolling.end()) {
        delete (*lastcontact_6_6_rolling);
        lastcontact_6_6_rolling = contactlist_6_6_rolling.erase(lastcontact_6_6_rolling);
    }
}

template <class Tcont, class Titer, class Ta, class Tb>
void _OptimalContactInsert(std::list<Tcont*>& contactlist,
                           Titer& lastcontact,
                           int& n_added,
                           ChContactContainerBase* mcontainer,
                           Ta* objA,  ///< collidable object A
                           Tb* objB,  ///< collidable object B
                           const collision::ChCollisionInfo& cinfo) {
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

void ChContactContainerDVI::AddContact(const collision::ChCollisionInfo& mcontact) {
    assert(mcontact.modelA->GetContactable());
    assert(mcontact.modelB->GetContactable());

    // See if both collision models use DVI i.e. 'nonsmooth dynamics' material
    // of type ChMaterialSurface, trying to downcast from ChMaterialSurfaceBase.
    // If not DVI vs DVI, just bailout (ex it could be that this was a DEM vs DEM contact)

    auto mmatA =
        std::dynamic_pointer_cast<ChMaterialSurface>(mcontact.modelA->GetContactable()->GetMaterialSurfaceBase());
    auto mmatB =
        std::dynamic_pointer_cast<ChMaterialSurface>(mcontact.modelB->GetContactable()->GetMaterialSurfaceBase());

    if (!mmatA || !mmatB)
        return;

    // Bail out if any of the two contactable objects is
    // not contact-active:

    bool inactiveA = !mcontact.modelA->GetContactable()->IsContactActive();
    bool inactiveB = !mcontact.modelB->GetContactable()->IsContactActive();

    if ((inactiveA && inactiveB))
        return;

    // CREATE THE CONTACTS
    //
    // Switch among the various cases of contacts: i.e. between a 6-dof variable and another 6-dof variable,
    // or 6 vs 3, etc.
    // These cases are made distinct to exploit the optimization coming from templates and static data sizes
    // in contact types.

    if (ChContactable_1vars<6>* mmboA = dynamic_cast<ChContactable_1vars<6>*>(mcontact.modelA->GetContactable())) {
        // 6_6
        if (ChContactable_1vars<6>* mmboB = dynamic_cast<ChContactable_1vars<6>*>(mcontact.modelB->GetContactable())) {
            if ((mmatA->rolling_friction && mmatB->rolling_friction) ||
                (mmatA->spinning_friction && mmatB->spinning_friction)) {
                _OptimalContactInsert(contactlist_6_6_rolling, lastcontact_6_6_rolling, n_added_6_6_rolling, this,
                                      mmboA, mmboB, mcontact);
            } else {
                _OptimalContactInsert(contactlist_6_6, lastcontact_6_6, n_added_6_6, this, mmboA, mmboB, mcontact);
            }
            return;
        }
        // 6_3
        if (ChContactable_1vars<3>* mmboB = dynamic_cast<ChContactable_1vars<3>*>(mcontact.modelB->GetContactable())) {
            _OptimalContactInsert(contactlist_6_3, lastcontact_6_3, n_added_6_3, this, mmboA, mmboB, mcontact);
            return;
        }
    }

    if (ChContactable_1vars<3>* mmboA = dynamic_cast<ChContactable_1vars<3>*>(mcontact.modelA->GetContactable())) {
        // 3_6 -> 6_3
        if (ChContactable_1vars<6>* mmboB = dynamic_cast<ChContactable_1vars<6>*>(mcontact.modelB->GetContactable())) {
            collision::ChCollisionInfo swapped_contact(mcontact, true);
            _OptimalContactInsert(contactlist_6_3, lastcontact_6_3, n_added_6_3, this, mmboB, mmboA, swapped_contact);
            return;
        }
        // 3_3
        if (ChContactable_1vars<3>* mmboB = dynamic_cast<ChContactable_1vars<3>*>(mcontact.modelB->GetContactable())) {
            _OptimalContactInsert(contactlist_3_3, lastcontact_3_3, n_added_3_3, this, mmboA, mmboB, mcontact);
            return;
        }
    }

    // ***TODO*** Fallback to some dynamic-size allocated constraint for cases that were not trapped by the switch
}

void ChContactContainerDVI::ComputeContactForces() {
    contact_forces.clear();
    SumAllContactForces(contactlist_6_6, contact_forces);
    SumAllContactForces(contactlist_6_3, contact_forces);
}

template <class Tcont>
void _ReportAllContacts(std::list<Tcont*>& contactlist, ChReportContactCallback* mcallback) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        bool proceed = mcallback->ReportContactCallback(
            (*itercontact)->GetContactP1(), (*itercontact)->GetContactP2(), (*itercontact)->GetContactPlane(),
            (*itercontact)->GetContactDistance(), (*itercontact)->GetContactForce(),
            VNULL,  // no react torques
            (*itercontact)->GetObjA(), (*itercontact)->GetObjB());
        if (!proceed)
            break;
        ++itercontact;
    }
}

template <class Tcont>
void _ReportAllContactsRolling(std::list<Tcont*>& contactlist, ChReportContactCallback* mcallback) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        bool proceed = mcallback->ReportContactCallback(
            (*itercontact)->GetContactP1(), (*itercontact)->GetContactP2(), (*itercontact)->GetContactPlane(),
            (*itercontact)->GetContactDistance(), (*itercontact)->GetContactForce(), (*itercontact)->GetContactTorque(),
            (*itercontact)->GetObjA(), (*itercontact)->GetObjB());
        if (!proceed)
            break;
        ++itercontact;
    }
}

void ChContactContainerDVI::ReportAllContacts(ChReportContactCallback* mcallback) {
    _ReportAllContacts(contactlist_6_6, mcallback);
    _ReportAllContacts(contactlist_6_3, mcallback);
    _ReportAllContacts(contactlist_3_3, mcallback);
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

void ChContactContainerDVI::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    unsigned int coffset = 0;
    _IntStateGatherReactions(coffset, contactlist_6_6, off_L, L, 3);
    _IntStateGatherReactions(coffset, contactlist_6_3, off_L, L, 3);
    _IntStateGatherReactions(coffset, contactlist_3_3, off_L, L, 3);
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

void ChContactContainerDVI::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    unsigned int coffset = 0;
    _IntStateScatterReactions(coffset, contactlist_6_6, off_L, L, 3);
    _IntStateScatterReactions(coffset, contactlist_6_3, off_L, L, 3);
    _IntStateScatterReactions(coffset, contactlist_3_3, off_L, L, 3);
    _IntStateScatterReactions(coffset, contactlist_6_6_rolling, off_L, L, 6);
}

template <class Tcont>
void _IntLoadResidual_CqL(unsigned int& coffset,
                          std::list<Tcont*>& contactlist,
                          const unsigned int off_L,    ///< offset in L multipliers
                          ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                          const ChVectorDynamic<>& L,  ///< the L vector
                          const double c,              ///< a scaling factor
                          const int stride) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContIntLoadResidual_CqL(off_L + coffset, R, L, c);
        coffset += stride;
        ++itercontact;
    }
}

void ChContactContainerDVI::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                                ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                                const ChVectorDynamic<>& L,  ///< the L vector
                                                const double c               ///< a scaling factor
                                                ) {
    unsigned int coffset = 0;
    _IntLoadResidual_CqL(coffset, contactlist_6_6, off_L, R, L, c, 3);
    _IntLoadResidual_CqL(coffset, contactlist_6_3, off_L, R, L, c, 3);
    _IntLoadResidual_CqL(coffset, contactlist_3_3, off_L, R, L, c, 3);
    _IntLoadResidual_CqL(coffset, contactlist_6_6_rolling, off_L, R, L, c, 6);
}

template <class Tcont>
void _IntLoadConstraint_C(unsigned int& coffset,
                          std::list<Tcont*>& contactlist,
                          const unsigned int off,  ///< offset in Qc residual
                          ChVectorDynamic<>& Qc,   ///< result: the Qc residual, Qc += c*C
                          const double c,          ///< a scaling factor
                          bool do_clamp,           ///< apply clamping to c*C?
                          double recovery_clamp,   ///< value for min/max clamping of c*C
                          const int stride) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContIntLoadConstraint_C(off + coffset, Qc, c, do_clamp, recovery_clamp);
        coffset += stride;
        ++itercontact;
    }
}

void ChContactContainerDVI::IntLoadConstraint_C(const unsigned int off,  ///< offset in Qc residual
                                                ChVectorDynamic<>& Qc,   ///< result: the Qc residual, Qc += c*C
                                                const double c,          ///< a scaling factor
                                                bool do_clamp,           ///< apply clamping to c*C?
                                                double recovery_clamp    ///< value for min/max clamping of c*C
                                                ) {
    unsigned int coffset = 0;
    _IntLoadConstraint_C(coffset, contactlist_6_6, off, Qc, c, do_clamp, recovery_clamp, 3);
    _IntLoadConstraint_C(coffset, contactlist_6_3, off, Qc, c, do_clamp, recovery_clamp, 3);
    _IntLoadConstraint_C(coffset, contactlist_3_3, off, Qc, c, do_clamp, recovery_clamp, 3);
    _IntLoadConstraint_C(coffset, contactlist_6_6_rolling, off, Qc, c, do_clamp, recovery_clamp, 6);
}

template <class Tcont>
void _IntToDescriptor(unsigned int& coffset,
                      std::list<Tcont*>& contactlist,
                      const unsigned int off_v,  ///< offset in v, R
                      const ChStateDelta& v,
                      const ChVectorDynamic<>& R,
                      const unsigned int off_L,  ///< offset in L, Qc
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

void ChContactContainerDVI::IntToDescriptor(const unsigned int off_v,  ///< offset in v, R
                                            const ChStateDelta& v,
                                            const ChVectorDynamic<>& R,
                                            const unsigned int off_L,  ///< offset in L, Qc
                                            const ChVectorDynamic<>& L,
                                            const ChVectorDynamic<>& Qc) {
    unsigned int coffset = 0;
    _IntToDescriptor(coffset, contactlist_6_6, off_v, v, R, off_L, L, Qc, 3);
    _IntToDescriptor(coffset, contactlist_6_3, off_v, v, R, off_L, L, Qc, 3);
    _IntToDescriptor(coffset, contactlist_3_3, off_v, v, R, off_L, L, Qc, 3);
    _IntToDescriptor(coffset, contactlist_6_6_rolling, off_v, v, R, off_L, L, Qc, 6);
}

template <class Tcont>
void _IntFromDescriptor(unsigned int& coffset,
                        std::list<Tcont*>& contactlist,
                        const unsigned int off_v,  ///< offset in v
                        ChStateDelta& v,
                        const unsigned int off_L,  ///< offset in L
                        ChVectorDynamic<>& L,
                        const int stride) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContIntFromDescriptor(off_L + coffset, L);
        coffset += stride;
        ++itercontact;
    }
}

void ChContactContainerDVI::IntFromDescriptor(const unsigned int off_v,  ///< offset in v
                                              ChStateDelta& v,
                                              const unsigned int off_L,  ///< offset in L
                                              ChVectorDynamic<>& L) {
    unsigned int coffset = 0;
    _IntFromDescriptor(coffset, contactlist_6_6, off_v, v, off_L, L, 3);
    _IntFromDescriptor(coffset, contactlist_6_3, off_v, v, off_L, L, 3);
    _IntFromDescriptor(coffset, contactlist_3_3, off_v, v, off_L, L, 3);
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

void ChContactContainerDVI::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    _InjectConstraints(contactlist_6_6, mdescriptor);
    _InjectConstraints(contactlist_6_3, mdescriptor);
    _InjectConstraints(contactlist_3_3, mdescriptor);
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

void ChContactContainerDVI::ConstraintsBiReset() {
    _ConstraintsBiReset(contactlist_6_6);
    _ConstraintsBiReset(contactlist_6_3);
    _ConstraintsBiReset(contactlist_3_3);
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

void ChContactContainerDVI::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    _ConstraintsBiLoad_C(contactlist_6_6, factor, recovery_clamp, do_clamp);
    _ConstraintsBiLoad_C(contactlist_6_3, factor, recovery_clamp, do_clamp);
    _ConstraintsBiLoad_C(contactlist_3_3, factor, recovery_clamp, do_clamp);
    _ConstraintsBiLoad_C(contactlist_6_6_rolling, factor, recovery_clamp, do_clamp);
}

void ChContactContainerDVI::ConstraintsLoadJacobians() {
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

void ChContactContainerDVI::ConstraintsFetch_react(double factor) {
    _ConstraintsFetch_react(contactlist_6_6, factor);
    _ConstraintsFetch_react(contactlist_6_3, factor);
    _ConstraintsFetch_react(contactlist_3_3, factor);
    _ConstraintsFetch_react(contactlist_6_6_rolling, factor);
}

void ChContactContainerDVI::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChContactContainerDVI>();
    // serialize parent class
    ChContactContainerBase::ArchiveOUT(marchive);
    // serialize all member data:
    // NO SERIALIZATION of contact list because assume it is volatile and generated when needed
}

/// Method to allow de serialization of transient data from archives.
void ChContactContainerDVI::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChContactContainerDVI>();
    // deserialize parent class
    ChContactContainerBase::ArchiveIN(marchive);
    // stream in all member data:
    RemoveAllContacts();
    // NO SERIALIZATION of contact list because assume it is volatile and generated when needed
}
}  // end namespace chrono
