//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//***OBSOLETE***

#include "physics/ChContactContainerDEM__old.h"
#include "physics/ChSystem.h"
#include "physics/ChIndexedNodes.h"
#include "physics/ChBody.h"

#include "collision/ChCModelBulletBody.h"

namespace chrono {

using namespace collision;
using namespace geometry;

//***OBSOLETE***


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChContactContainerDEM__old> a_registration_ChContactContainerDEM__old;

ChContactContainerDEM__old::ChContactContainerDEM__old() {
    contactlist.clear();
    n_added = 0;
}

ChContactContainerDEM__old::~ChContactContainerDEM__old() {
    std::list<ChContactDEM__old*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        delete (*itercontact);
        (*itercontact) = 0;
        ++itercontact;
    }

    contactlist.clear();
}

void ChContactContainerDEM__old::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class, basically doing nothing :)
    ChContactContainerBase::Update(mytime, update_assets);
}

//// STATE BOOKKEEPING FUNCTIONS

void ChContactContainerDEM__old::IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                              ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                              const double c           ///< a scaling factor
                                              ) {
    ChContactDEM__old* cntct;

    std::list<ChContactDEM__old*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        cntct = (ChContactDEM__old*)(*itercontact);
        if ((cntct->GetContactPenetration() > 0)) {
            (*itercontact)->DemIntLoadResidual_F(R, c);
        }
        ++itercontact;
    }
}

////////// LCP INTERFACES ////

void ChContactContainerDEM__old::ConstraintsFbLoadForces(double factor) {
    ChContactDEM__old* cntct;

    std::list<ChContactDEM__old*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        cntct = (ChContactDEM__old*)(*itercontact);
        if ((cntct->GetContactPenetration() > 0)) {
            (*itercontact)->ConstraintsFbLoadForces(factor);
        }
        ++itercontact;
    }
}

void ChContactContainerDEM__old::RemoveAllContacts() {
    std::list<ChContactDEM__old*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        delete (*itercontact);
        (*itercontact) = 0;
        ++itercontact;
    }
    contactlist.clear();

    lastcontact = contactlist.begin();
    n_added = 0;
}

void ChContactContainerDEM__old::BeginAddContact() {
    lastcontact = contactlist.begin();
    n_added = 0;
}

void ChContactContainerDEM__old::EndAddContact() {
    // remove contacts that are beyond last contact
    while (lastcontact != contactlist.end()) {
        delete (*lastcontact);
        lastcontact = contactlist.erase(lastcontact);
    }
}

void ChContactContainerDEM__old::AddContact(const collision::ChCollisionInfo& mcontact) {
    // Do nothing if the shapes are separated
    if (mcontact.distance >= 0)
        return;

    // Return now if not expected contact models or no associated bodies.
    ChModelBulletBody* mmboA = dynamic_cast<ChModelBulletBody*>(mcontact.modelA);
    ChModelBulletBody* mmboB = dynamic_cast<ChModelBulletBody*>(mcontact.modelB);
    if (!mmboA || !mmboB)
        return;

    if (!mmboA->GetBody() || !mmboB->GetBody())
        return;

    // Return now if both bodies are inactive.
    if (!mmboA->GetBody()->IsActive() && !mmboB->GetBody()->IsActive())
        return;

    // Reuse an existing contact or create a new one
    if (lastcontact != contactlist.end()) {
        // reuse old contacts
        (*lastcontact)->Reset(mmboA, mmboB, mcontact);
        lastcontact++;
    } else {
        // add new contact
        ChContactDEM__old* mc = new ChContactDEM__old(mmboA, mmboB, mcontact);
        contactlist.push_back(mc);
        lastcontact = contactlist.end();
    }

    n_added++;
}

void ChContactContainerDEM__old::ReportAllContacts(ChReportContactCallback* mcallback) {
    std::list<ChContactDEM__old*>::iterator itercontact = contactlist.begin();
    while (itercontact != contactlist.end()) {
        bool proceed = mcallback->ReportContactCallback(
            (*itercontact)->GetContactP1(), (*itercontact)->GetContactP2(), (*itercontact)->GetContactPlane(),
            (*itercontact)->GetContactPenetration(), 0.0, (*itercontact)->GetContactForceLocal(),
            VNULL,  // no react torques
            (*itercontact)->GetModel1(), (*itercontact)->GetModel2());

        if (!proceed)
            break;

        ++itercontact;
    }
}

}  // END_OF_NAMESPACE____
