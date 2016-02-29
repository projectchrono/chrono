//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCONTACTCONTAINERDEM_H
#define CHCONTACTCONTAINERDEM_H

#include <algorithm>
#include <cmath>
#include <list>

#include "chrono/physics/ChContactContainerBase.h"
#include "chrono/physics/ChContactDEM.h"
#include "chrono/physics/ChContactable.h"

namespace chrono {

/// Class representing a container of many penalty contacts.
/// This is implemented as a typical linked list of ChContactDEM objects
/// (that is, contacts between two ChContactable objects).
class ChApi ChContactContainerDEM : public ChContactContainerBase {
    CH_RTTI(ChContactContainerDEM, ChContactContainerBase);

  public:
    typedef ChContactDEM< ChContactable_1vars<6>, ChContactable_1vars<6> > ChContactDEM_6_6;
    typedef ChContactDEM< ChContactable_1vars<6>, ChContactable_1vars<3> > ChContactDEM_6_3;
    typedef ChContactDEM< ChContactable_1vars<3>, ChContactable_1vars<3> > ChContactDEM_3_3;
    typedef ChContactDEM< ChContactable_3vars<3,3,3>, ChContactable_1vars<6> > ChContactDEM_333_6;
    typedef ChContactDEM< ChContactable_3vars<3,3,3>, ChContactable_1vars<3> > ChContactDEM_333_3;
    typedef ChContactDEM< ChContactable_3vars<3,3,3>, ChContactable_3vars<3,3,3> > ChContactDEM_333_333;

  protected:
    //
    // DATA
    //

    std::list< ChContactDEM_6_6* > contactlist_6_6;
    std::list< ChContactDEM_6_3* > contactlist_6_3;
    std::list< ChContactDEM_3_3* > contactlist_3_3;
    std::list< ChContactDEM_333_6* > contactlist_333_6;
    std::list< ChContactDEM_333_3* > contactlist_333_3;
    std::list< ChContactDEM_333_333* > contactlist_333_333;

    int n_added_6_6;
    int n_added_6_3;
    int n_added_3_3;
    int n_added_333_6;
    int n_added_333_3;
    int n_added_333_333;

    std::list<ChContactDEM_6_6*>::iterator lastcontact_6_6;
    std::list<ChContactDEM_6_3*>::iterator lastcontact_6_3;
    std::list<ChContactDEM_3_3*>::iterator lastcontact_3_3;
    std::list<ChContactDEM_333_6*>::iterator lastcontact_333_6;
    std::list<ChContactDEM_333_3*>::iterator lastcontact_333_3;
    std::list<ChContactDEM_333_333*>::iterator lastcontact_333_333;

  public:
    //
    // CONSTRUCTORS
    //

    ChContactContainerDEM();

    virtual ~ChContactContainerDEM();

    //
    // FUNCTIONS
    //
    /// Tell the number of added contacts
    virtual int GetNcontacts() {
        return n_added_6_6 + n_added_6_3 + n_added_3_3 + n_added_333_6 + n_added_333_3 + n_added_333_333;
    }

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

    /// Scans all the contacts and for each contact executes the ReportContactCallback()
    /// function of the user object inherited from ChReportContactCallback.
    virtual void ReportAllContacts(ChReportContactCallback* mcallback) override;

    /// In detail, it computes jacobians, violations, etc. and stores
    /// results in inner structures of contacts.
    virtual void Update(double mtime, bool update_assets = true);

    //
    // STATE FUNCTIONS
    //

    // Override/implement interfaces for global state vectors (see ChPhysicsItem)
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) override;
    virtual void InjectKRMmatrices(ChLcpSystemDescriptor& mdescriptor) override;

    //
    // LCP INTERFACE
    //

    virtual void ConstraintsFbLoadForces(double factor);

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive) {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChContactContainerBase::ArchiveOUT(marchive);
        // serialize all member data:
        // NO SERIALIZATION of contact list because assume it is volatile and generated when needed
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChContactContainerBase::ArchiveIN(marchive);
        // stream in all member data:
        RemoveAllContacts();
        // NO SERIALIZATION of contact list because assume it is volatile and generated when needed
    }
};

}  // END_OF_NAMESPACE____

#endif
