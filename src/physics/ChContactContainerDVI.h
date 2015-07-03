//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCONTACTCONTAINERDVI_H
#define CHCONTACTCONTAINERDVI_H


#include "physics/ChContactContainerBase.h"
#include "physics/ChContactable.h"
#include "physics/ChContactDVI.h"
//#include "physics/ChContactRolling.h"
#include <list>

namespace chrono {

///
/// Class representing a container of many contacts,
/// implemented as a typical linked list of ChContactDVI
/// objects (that is, contacts between two ChContactable objects).
/// This is the default contact container used in most
/// cases.
///

class ChApi ChContactContainerDVI : public ChContactContainerBase {
    CH_RTTI(ChContactContainerDVI, ChContactContainerBase);

  public:
    typedef ChContactDVI< ChContactable_1vars<6>, ChContactable_1vars<6> > ChContactDVI_6_6;
    typedef ChContactDVI< ChContactable_1vars<6>, ChContactable_1vars<3> > ChContactDVI_6_3;
    typedef ChContactDVI< ChContactable_1vars<3>, ChContactable_1vars<3> > ChContactDVI_3_3;

  protected:
    //
    // DATA
    //

    std::list< ChContactDVI_6_6* > contactlist_6_6;
    std::list< ChContactDVI_6_3* > contactlist_6_3;
    std::list< ChContactDVI_3_3* > contactlist_3_3;

    int n_added_6_6;
    int n_added_6_3;
    int n_added_3_3;

    std::list<ChContactDVI_6_6*>::iterator lastcontact_6_6;
    std::list<ChContactDVI_6_3*>::iterator lastcontact_6_3;
    std::list<ChContactDVI_3_3*>::iterator lastcontact_3_3;

  public:
    //
    // CONSTRUCTORS
    //

    ChContactContainerDVI();

    virtual ~ChContactContainerDVI();

    //
    // FUNCTIONS
    //
    /// Tell the number of added contacts
    virtual int GetNcontacts() { return n_added_6_6 + n_added_6_3 + n_added_3_3;} // TODO + n_added_roll; }

    /// Return the contact List
    //virtual std::list<ChContact*>& GetContactList() { return contactlist; }

    /// Remove (delete) all contained contact data.
    virtual void RemoveAllContacts();

    /// The collision system will call BeginAddContact() before adding
    /// all contacts (for example with AddContact() or similar). Instead of
    /// simply deleting all list of the previous contacts, this optimized implementation
    /// rewinds the link iterator to begin and tries to reuse previous contact objects
    /// until possible, to avoid too much allocation/deallocation.
    virtual void BeginAddContact();

    /// Add a contact between two frames.
    virtual void AddContact(const collision::ChCollisionInfo& mcontact);

    /// The collision system will call BeginAddContact() after adding
    /// all contacts (for example with AddContact() or similar). This optimized version
    /// purges the end of the list of contacts that were not reused (if any).
    virtual void EndAddContact();

    /// Scans all the contacts and for each contact exacutes the ReportContactCallback()
    /// function of the user object inherited from ChReportContactCallback.
    /// Child classes of ChContactContainerBase should try to implement this (although
    /// in some highly-optimized cases as in ChContactContainerGPU it could be impossible to
    /// report all contacts).
    virtual void ReportAllContacts2(ChReportContactCallback2* mcallback);

    /// Tell the number of scalar bilateral constraints (actually, friction
    /// constraints aren't exactly as unilaterals, but count them too)
    virtual int GetDOC_d() { return (GetNcontacts() * 3) ;} // TODO  + (n_added_roll * 6); }

    /// In detail, it computes jacobians, violations, etc. and stores
    /// results in inner structures of contacts.
    virtual void Update(double mtime, bool update_assets = true);

    //
    // STATE FUNCTIONS
    //

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L);
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L);
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c);
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp);
    virtual void IntToLCP(const unsigned int off_v,
                          const ChStateDelta& v,
                          const ChVectorDynamic<>& R,
                          const unsigned int off_L,
                          const ChVectorDynamic<>& L,
                          const ChVectorDynamic<>& Qc);
    virtual void IntFromLCP(const unsigned int off_v, ChStateDelta& v, const unsigned int off_L, ChVectorDynamic<>& L);

    //
    // LCP INTERFACE
    //

    virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);
    virtual void ConstraintsBiReset();
    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false);
    // virtual void ConstraintsBiLoad_Ct(double factor=1.) {};
    virtual void ConstraintsLoadJacobians();
    virtual void ConstraintsLiLoadSuggestedSpeedSolution();
    virtual void ConstraintsLiLoadSuggestedPositionSolution();
    virtual void ConstraintsLiFetchSuggestedSpeedSolution();
    virtual void ConstraintsLiFetchSuggestedPositionSolution();
    virtual void ConstraintsFetch_react(double factor = 1.);
};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____

#endif
