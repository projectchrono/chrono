//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//


#include "physics/ChContactContainerDVI.h"
#include "physics/ChSystem.h"
#include "physics/ChBody.h"
#include "physics/ChParticlesClones.h"
#include "lcp/ChLcpConstraintTwoTuplesContactN.h"
#include "collision/ChCModelBulletBody.h"
#include "collision/ChCModelBulletParticle.h"

namespace chrono {

using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChContactContainerDVI> a_registration_ChContactContainerDVI;


ChContactContainerDVI::ChContactContainerDVI() {
    contactlist_6_6.clear();
    n_added_6_6 = 0;

    contactlist_6_3.clear();
    n_added_6_3 = 0;

    contactlist_3_3.clear();
    n_added_3_3 = 0;

    //contactlist_roll.clear();
    //n_added_roll = 0;
}

ChContactContainerDVI::~ChContactContainerDVI() {

    RemoveAllContacts();
}

void ChContactContainerDVI::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class, basically doing nothing :)
    ChContactContainerBase::Update(mytime, update_assets);
}


template <class Tcont, class Titer>
void _RemoveAllContacts(std::list<Tcont*>contactlist, Titer& lastcontact, int& n_added) {
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
    //**TODO*** cont. roll.
}

void ChContactContainerDVI::BeginAddContact() {
    lastcontact_6_6 = contactlist_6_6.begin();
    n_added_6_6 = 0;

    lastcontact_6_3 = contactlist_6_3.begin();
    n_added_6_3 = 0;

    lastcontact_3_3 = contactlist_3_3.begin();
    n_added_3_3 = 0;

    //lastcontact_roll = contactlist_roll.begin();
    //n_added_roll = 0;
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

    //while (lastcontact_roll != contactlist_roll.end()) {
    //    delete (*lastcontact_roll);
    //    lastcontact_roll = contactlist_roll.erase(lastcontact_roll);
    //}
}



template <class Tcont, class Titer, class Ta, class Tb>
void _OptimalContactInsert(
    std::list<Tcont*> contactlist, 
    Titer& lastcontact, 
    int& n_added,
    ChContactContainerBase* mcontainer,
    Ta* objA,  ///< collidable object A
    Tb* objB,  ///< collidable object B
    const collision::ChCollisionInfo& cinfo
    )
{
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

    ChSharedPtr<ChMaterialSurface> mmatA( mcontact.modelA->GetContactable()->GetMaterialSurfaceBase().DynamicCastTo<ChMaterialSurface>() );
    ChSharedPtr<ChMaterialSurface> mmatB( mcontact.modelB->GetContactable()->GetMaterialSurfaceBase().DynamicCastTo<ChMaterialSurface>() );

    if (mmatA.IsNull() || mmatB.IsNull())
        return;

    // Bail out if any of the two contactable objects is 
    // not contact-active:

    bool inactiveA = !mcontact.modelA->GetContactable()->IsContactActive();
    bool inactiveB = !mcontact.modelA->GetContactable()->IsContactActive();

    if ((inactiveA && inactiveB))
        return;

    // CREATE THE CONTACTS
    //
    // Switch among the various cases of contacts: i.e. between a 6-dof variable and another 6-dof variable, 
    // or 6 vs 3, etc.
    // These cases are made distinct to exploit the optimization coming from templates and static data sizes 
    // in contact types.

    if (    ChContactable_1vars<6>* mmboA = dynamic_cast<ChContactable_1vars<6>*>(mcontact.modelA->GetContactable())) {
        // 6_6
        if (ChContactable_1vars<6>* mmboB = dynamic_cast<ChContactable_1vars<6>*>(mcontact.modelB->GetContactable())) {
            _OptimalContactInsert(contactlist_6_6, lastcontact_6_6, n_added_6_6, this, mmboA, mmboB, mcontact);
        }
        // 6_3
        if (ChContactable_1vars<3>* mmboB = dynamic_cast<ChContactable_1vars<3>*>(mcontact.modelB->GetContactable())) {
            _OptimalContactInsert(contactlist_6_3, lastcontact_6_3, n_added_6_3, this, mmboA, mmboB, mcontact);
        }
    }

    if (    ChContactable_1vars<3>* mmboA = dynamic_cast<ChContactable_1vars<3>*>(mcontact.modelA->GetContactable())) {
        // 3_6 -> 6_3
        if (ChContactable_1vars<6>* mmboB = dynamic_cast<ChContactable_1vars<6>*>(mcontact.modelB->GetContactable())) {
            collision::ChCollisionInfo scontact = mcontact; scontact.SwapModels();
            _OptimalContactInsert(contactlist_6_3, lastcontact_6_3, n_added_6_3, this, mmboB, mmboA, mcontact);
        }
        // 3_3
        if (ChContactable_1vars<3>* mmboB = dynamic_cast<ChContactable_1vars<3>*>(mcontact.modelB->GetContactable())) {
            _OptimalContactInsert(contactlist_3_3, lastcontact_3_3, n_added_3_3, this, mmboA, mmboB, mcontact);
        }
    }

    // ***TODO*** Fallback to some dynamic-size allocated constraint for cases that were not trapped by the switch
}


template <class Tcont>
void _ReportAllContacts(std::list<Tcont*>contactlist, ChReportContactCallback2* mcallback) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        bool proceed =
            mcallback->ReportContactCallback2((*itercontact)->GetContactP1(), (*itercontact)->GetContactP2(),
                                             *(*itercontact)->GetContactPlane(), (*itercontact)->GetContactDistance(),
                                             (*itercontact)->GetContactForce(),
                                             VNULL,  // no react torques
                                             (*itercontact)->GetObjA(), (*itercontact)->GetObjB());
        if (!proceed)
            break;
        ++itercontact;
    }
}

void ChContactContainerDVI::ReportAllContacts2(ChReportContactCallback2* mcallback) {
    
    _ReportAllContacts(contactlist_6_6, mcallback);
    _ReportAllContacts(contactlist_6_3, mcallback);
    _ReportAllContacts(contactlist_3_3, mcallback);
    //***TODO*** rolling cont. 
}

////////// STATE INTERFACE ////

template <class Tcont>
void _IntStateGatherReactions(std::list<Tcont*>contactlist, const unsigned int off_L, ChVectorDynamic<>& L) {
    int coffset = 0;
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContIntStateGatherReactions(off_L + coffset, L);
        coffset += 3;  // NOTE THAT coffset = 6 for rolling contacts!!!! to be templated ...
        ++itercontact;
    }
}

void ChContactContainerDVI::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    _IntStateGatherReactions(contactlist_6_6, off_L,L);
    _IntStateGatherReactions(contactlist_6_3, off_L,L);
    _IntStateGatherReactions(contactlist_3_3, off_L,L);
    //***TODO*** rolling cont. NOTE THAT coffset = 6 for rolling contacts!!!!
}

template <class Tcont>
void _IntStateScatterReactions(std::list<Tcont*>contactlist, const unsigned int off_L, const ChVectorDynamic<>& L) {
    int coffset = 0;
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContIntStateScatterReactions(off_L + coffset, L);
        coffset += 3;
        ++itercontact;
    }
}

void ChContactContainerDVI::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    _IntStateScatterReactions(contactlist_6_6, off_L, L);
    _IntStateScatterReactions(contactlist_6_3, off_L, L);
    _IntStateScatterReactions(contactlist_3_3, off_L, L);
    //***TODO*** rolling cont. NOTE THAT coffset = 6 for rolling contacts!!!!
}


template <class Tcont>
void _IntLoadResidual_CqL(std::list<Tcont*>contactlist, 
                                             const unsigned int off_L,    ///< offset in L multipliers
                                             ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                             const ChVectorDynamic<>& L,  ///< the L vector
                                             const double c               ///< a scaling factor
                                             )
{
    int coffset = 0;
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContIntLoadResidual_CqL(off_L + coffset, R, L, c);
        coffset += 3;
        ++itercontact;
    }
}

void ChContactContainerDVI::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                             ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                             const ChVectorDynamic<>& L,  ///< the L vector
                                             const double c               ///< a scaling factor
                                             ) {
    _IntLoadResidual_CqL(contactlist_6_6, off_L, R, L, c );
    _IntLoadResidual_CqL(contactlist_6_3, off_L, R, L, c );
    _IntLoadResidual_CqL(contactlist_3_3, off_L, R, L, c );
    //***TODO*** rolling cont. NOTE THAT coffset = 6 for rolling contacts!!!!
}


template <class Tcont>
void _IntLoadConstraint_C(std::list<Tcont*>contactlist, 
                                             const unsigned int off,  ///< offset in Qc residual
                                             ChVectorDynamic<>& Qc,   ///< result: the Qc residual, Qc += c*C
                                             const double c,          ///< a scaling factor
                                             bool do_clamp,           ///< apply clamping to c*C?
                                             double recovery_clamp    ///< value for min/max clamping of c*C
                                             ) {
    int coffset = 0;
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContIntLoadConstraint_C(off + coffset, Qc, c, do_clamp, recovery_clamp);
        coffset += 3;
        ++itercontact;
    }
}

void ChContactContainerDVI::IntLoadConstraint_C(const unsigned int off,  ///< offset in Qc residual
                                             ChVectorDynamic<>& Qc,   ///< result: the Qc residual, Qc += c*C
                                             const double c,          ///< a scaling factor
                                             bool do_clamp,           ///< apply clamping to c*C?
                                             double recovery_clamp    ///< value for min/max clamping of c*C
                                             ) {
    _IntLoadConstraint_C(contactlist_6_6, off, Qc, c, do_clamp, recovery_clamp);
    _IntLoadConstraint_C(contactlist_6_3, off, Qc, c, do_clamp, recovery_clamp);
    _IntLoadConstraint_C(contactlist_3_3, off, Qc, c, do_clamp, recovery_clamp);
    //***TODO*** rolling cont. NOTE THAT coffset = 6 for rolling contacts!!!!
}



template <class Tcont>
void _IntToLCP(std::list<Tcont*>contactlist,
                                  const unsigned int off_v,  ///< offset in v, R
                                  const ChStateDelta& v,
                                  const ChVectorDynamic<>& R,
                                  const unsigned int off_L,  ///< offset in L, Qc
                                  const ChVectorDynamic<>& L,
                                  const ChVectorDynamic<>& Qc) {
    int coffset = 0;
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContIntToLCP(off_L + coffset, L, Qc);
        coffset += 3;
        ++itercontact;
    }
}

void ChContactContainerDVI::IntToLCP(const unsigned int off_v,  ///< offset in v, R
                                  const ChStateDelta& v,
                                  const ChVectorDynamic<>& R,
                                  const unsigned int off_L,  ///< offset in L, Qc
                                  const ChVectorDynamic<>& L,
                                  const ChVectorDynamic<>& Qc) {
    _IntToLCP(contactlist_6_6, off_v, v, R, off_L, L, Qc);
    _IntToLCP(contactlist_6_3, off_v, v, R, off_L, L, Qc);
    _IntToLCP(contactlist_3_3, off_v, v, R, off_L, L, Qc);
    //***TODO*** rolling cont. NOTE THAT coffset = 6 for rolling contacts!!!!
}



template <class Tcont>
void _IntFromLCP(std::list<Tcont*>contactlist,
                                    const unsigned int off_v,  ///< offset in v
                                    ChStateDelta& v,
                                    const unsigned int off_L,  ///< offset in L
                                    ChVectorDynamic<>& L) {
    int coffset = 0;
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ContIntFromLCP(off_L + coffset, L);
        coffset += 3;
        ++itercontact;
    }
}

void ChContactContainerDVI::IntFromLCP(const unsigned int off_v,  ///< offset in v
                                    ChStateDelta& v,
                                    const unsigned int off_L,  ///< offset in L
                                    ChVectorDynamic<>& L) {
    _IntFromLCP(contactlist_6_6, off_v, v, off_L, L);
    _IntFromLCP(contactlist_6_3, off_v, v, off_L, L);
    _IntFromLCP(contactlist_3_3, off_v, v, off_L, L);
    //***TODO*** rolling cont. NOTE THAT coffset = 6 for rolling contacts!!!!
}

////////// LCP INTERFACES ////

template <class Tcont>
void _InjectConstraints(std::list<Tcont*>contactlist, ChLcpSystemDescriptor& mdescriptor) {
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->InjectConstraints(mdescriptor);
        ++itercontact;
    }
}

void ChContactContainerDVI::InjectConstraints(ChLcpSystemDescriptor& mdescriptor) {
    _InjectConstraints(contactlist_6_6, mdescriptor);
    _InjectConstraints(contactlist_6_3, mdescriptor);
    _InjectConstraints(contactlist_3_3, mdescriptor);
    //***TODO*** rolling cont.
}



template <class Tcont>
void _ConstraintsBiReset(std::list<Tcont*>contactlist) {
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
}


template <class Tcont>
void _ConstraintsBiLoad_C(std::list<Tcont*>contactlist, double factor, double recovery_clamp, bool do_clamp) {
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
}


void ChContactContainerDVI::ConstraintsLoadJacobians() {
    // already loaded when ChContact objects are created
}


template <class Tcont>
void _ConstraintsFetch_react(std::list<Tcont*>contactlist, double factor) {
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
}



// Following functions are for exploiting the contact persistence

template <class Tcont>
void _ConstraintsLiLoadSuggestedSpeedSolution(std::list<Tcont*>contactlist) {
    // Fetch the last computed impulsive reactions from the persistent contact manifold (could
    // be used for warm starting the CCP speed solver):
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ConstraintsLiLoadSuggestedSpeedSolution();
        ++itercontact;
    }
}

void ChContactContainerDVI::ConstraintsLiLoadSuggestedSpeedSolution() {
   _ConstraintsLiLoadSuggestedSpeedSolution(contactlist_6_6);
   _ConstraintsLiLoadSuggestedSpeedSolution(contactlist_6_3);
   _ConstraintsLiLoadSuggestedSpeedSolution(contactlist_3_3);
}


template <class Tcont>
void _ConstraintsLiLoadSuggestedPositionSolution(std::list<Tcont*>contactlist) {
    // Fetch the last computed 'positional' reactions from the persistent contact manifold (could
    // be used for warm starting the CCP position stabilization solver):
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ConstraintsLiLoadSuggestedPositionSolution();
        ++itercontact;
    }
}

void ChContactContainerDVI::ConstraintsLiLoadSuggestedPositionSolution() {
    _ConstraintsLiLoadSuggestedPositionSolution(contactlist_6_6);
    _ConstraintsLiLoadSuggestedPositionSolution(contactlist_6_3);
    _ConstraintsLiLoadSuggestedPositionSolution(contactlist_3_3);
}


template <class Tcont>
void _ConstraintsLiFetchSuggestedSpeedSolution(std::list<Tcont*>contactlist) {
    // Store the last computed reactions into the persistent contact manifold (might
    // be used for warm starting CCP the speed solver):
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ConstraintsLiFetchSuggestedSpeedSolution();
        ++itercontact;
    }
}

void ChContactContainerDVI::ConstraintsLiFetchSuggestedSpeedSolution() {
    _ConstraintsLiFetchSuggestedSpeedSolution(contactlist_6_6);
    _ConstraintsLiFetchSuggestedSpeedSolution(contactlist_6_3);
    _ConstraintsLiFetchSuggestedSpeedSolution(contactlist_3_3);
}


template <class Tcont>
void _ConstraintsLiFetchSuggestedPositionSolution(std::list<Tcont*>contactlist) {
    // Store the last computed 'positional' reactions into the persistent contact manifold (might
    // be used for warm starting the CCP position stabilization solver):
    typename std::list<Tcont*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        (*itercontact)->ConstraintsLiFetchSuggestedPositionSolution();
        ++itercontact;
    }
}

void ChContactContainerDVI::ConstraintsLiFetchSuggestedPositionSolution() {
    _ConstraintsLiFetchSuggestedPositionSolution(contactlist_6_6);
    _ConstraintsLiFetchSuggestedPositionSolution(contactlist_6_3);
    _ConstraintsLiFetchSuggestedPositionSolution(contactlist_3_3);
}

}  // END_OF_NAMESPACE____
