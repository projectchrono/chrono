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

#ifndef CHCONTACTCONTAINERDVI_H
#define CHCONTACTCONTAINERDVI_H

#include <list>

#include "chrono/physics/ChContactContainerBase.h"
#include "chrono/physics/ChContactDVI.h"
#include "chrono/physics/ChContactDVIrolling.h"
#include "chrono/physics/ChContactable.h"

namespace chrono {

/// Class representing a container of many complementarity contacts.
/// This is implemented as a typical linked list of ChContactDVI objects
/// (that is, contacts between two ChContactable objects, with 3 reactions).
/// It might also contain ChContactDVIrolling objects (extended versions of ChContactDVI,
/// with 6 reactions, that account also for rolling and spinning resistance), but also
/// for '6dof vs 6dof' contactables.
/// This is the default contact container used in most cases.
class ChApi ChContactContainerDVI : public ChContactContainerBase {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChContactContainerDVI)

  public:
    typedef ChContactDVI<ChContactable_1vars<6>, ChContactable_1vars<6> > ChContactDVI_6_6;
    typedef ChContactDVI<ChContactable_1vars<6>, ChContactable_1vars<3> > ChContactDVI_6_3;
    typedef ChContactDVI<ChContactable_1vars<3>, ChContactable_1vars<3> > ChContactDVI_3_3;
    typedef ChContactDVIrolling<ChContactable_1vars<6>, ChContactable_1vars<6> > ChContactDVIrolling_6_6;

  protected:
    std::list<ChContactDVI_6_6*> contactlist_6_6;
    std::list<ChContactDVI_6_3*> contactlist_6_3;
    std::list<ChContactDVI_3_3*> contactlist_3_3;
    std::list<ChContactDVIrolling_6_6*> contactlist_6_6_rolling;

    int n_added_6_6;
    int n_added_6_3;
    int n_added_3_3;
    int n_added_6_6_rolling;

    std::list<ChContactDVI_6_6*>::iterator lastcontact_6_6;
    std::list<ChContactDVI_6_3*>::iterator lastcontact_6_3;
    std::list<ChContactDVI_3_3*>::iterator lastcontact_3_3;
    std::list<ChContactDVIrolling_6_6*>::iterator lastcontact_6_6_rolling;

  public:
    ChContactContainerDVI();
    ChContactContainerDVI(const ChContactContainerDVI& other);
    virtual ~ChContactContainerDVI();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChContactContainerDVI* Clone() const override { return new ChContactContainerDVI(*this); }

    /// Tell the number of added contacts
    virtual int GetNcontacts() const override { return n_added_6_6 + n_added_6_3 + n_added_3_3 + n_added_6_6_rolling; }

    /// Remove (delete) all contained contact data.
    virtual void RemoveAllContacts() override;

    /// The collision system will call BeginAddContact() before adding
    /// all contacts (for example with AddContact() or similar). Instead of
    /// simply deleting all list of the previous contacts, this optimized implementation
    /// rewinds the link iterator to begin and tries to reuse previous contact objects
    /// until possible, to avoid too much allocation/deallocation.
    virtual void BeginAddContact() override;

    /// Add a contact between two frames.
    virtual void AddContact(const collision::ChCollisionInfo& mcontact) override;

    /// The collision system will call BeginAddContact() after adding
    /// all contacts (for example with AddContact() or similar). This optimized version
    /// purges the end of the list of contacts that were not reused (if any).
    virtual void EndAddContact() override;

    /// Scans all the contacts and for each contact executes the ReportContactCallback()
    /// function of the user object inherited from ChReportContactCallback.
    virtual void ReportAllContacts(ChReportContactCallback* mcallback) override;

    /// Tell the number of scalar bilateral constraints (actually, friction
    /// constraints aren't exactly as unilaterals, but count them too)
    virtual int GetDOC_d() override {
        return 3 * (n_added_6_6 + n_added_6_3 + n_added_3_3) + 6 * (n_added_6_6_rolling);
    }

    /// In detail, it computes jacobians, violations, etc. and stores
    /// results in inner structures of contacts.
    virtual void Update(double mtime, bool update_assets = true) override;

    /// Compute contact forces on all contactable objects in this container.
    virtual void ComputeContactForces() override;

    //
    // STATE FUNCTIONS
    //

    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp) override;
    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) override;

    //
    // SOLVER INTERFACE
    //

    virtual void InjectConstraints(ChSystemDescriptor& mdescriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsLoadJacobians() override;
    virtual void ConstraintsFetch_react(double factor = 1) override;

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChContactContainerDVI,0)

}  // end namespace chrono

#endif
