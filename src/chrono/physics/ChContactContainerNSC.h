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

#ifndef CH_CONTACTCONTAINER_NSC_H
#define CH_CONTACTCONTAINER_NSC_H

#include <list>

#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChContactNSC.h"
#include "chrono/physics/ChContactNSCrolling.h"
#include "chrono/physics/ChContactable.h"

namespace chrono {

/// Class representing a container of many non-smooth contacts.
/// Implemented using linked lists of ChContactNSC objects (that is, contacts between two ChContactable objects, with 3
/// reactions). It might also contain ChContactNSCrolling objects (extended versions of ChContactNSC, with 6 reactions,
/// that account also for rolling and spinning resistance), but also for '6dof vs 6dof' contactables.
class ChApi ChContactContainerNSC : public ChContactContainer {
  public:
    typedef ChContactNSC<ChContactable_1vars<6>, ChContactable_1vars<6> > ChContactNSC_6_6;
    typedef ChContactNSC<ChContactable_1vars<6>, ChContactable_1vars<3> > ChContactNSC_6_3;
    typedef ChContactNSC<ChContactable_1vars<3>, ChContactable_1vars<3> > ChContactNSC_3_3;
    typedef ChContactNSC<ChContactable_3vars<3, 3, 3>, ChContactable_1vars<3> > ChContactNSC_333_3;
    typedef ChContactNSC<ChContactable_3vars<3, 3, 3>, ChContactable_1vars<6> > ChContactNSC_333_6;
    typedef ChContactNSC<ChContactable_3vars<3, 3, 3>, ChContactable_3vars<3, 3, 3> > ChContactNSC_333_333;
    typedef ChContactNSC<ChContactable_3vars<6, 6, 6>, ChContactable_1vars<3> > ChContactNSC_666_3;
    typedef ChContactNSC<ChContactable_3vars<6, 6, 6>, ChContactable_1vars<6> > ChContactNSC_666_6;
    typedef ChContactNSC<ChContactable_3vars<6, 6, 6>, ChContactable_3vars<3, 3, 3> > ChContactNSC_666_333;
    typedef ChContactNSC<ChContactable_3vars<6, 6, 6>, ChContactable_3vars<6, 6, 6> > ChContactNSC_666_666;

    typedef ChContactNSCrolling<ChContactable_1vars<6>, ChContactable_1vars<6> > ChContactNSCrolling_6_6;

  protected:
    std::list<ChContactNSC_6_6*> contactlist_6_6;
    std::list<ChContactNSC_6_3*> contactlist_6_3;
    std::list<ChContactNSC_3_3*> contactlist_3_3;
    std::list<ChContactNSC_333_3*> contactlist_333_3;
    std::list<ChContactNSC_333_6*> contactlist_333_6;
    std::list<ChContactNSC_333_333*> contactlist_333_333;
    std::list<ChContactNSC_666_3*> contactlist_666_3;
    std::list<ChContactNSC_666_6*> contactlist_666_6;
    std::list<ChContactNSC_666_333*> contactlist_666_333;
    std::list<ChContactNSC_666_666*> contactlist_666_666;

    std::list<ChContactNSCrolling_6_6*> contactlist_6_6_rolling;

    int n_added_6_6;
    int n_added_6_3;
    int n_added_3_3;
    int n_added_333_3;
    int n_added_333_6;
    int n_added_333_333;
    int n_added_666_3;
    int n_added_666_6;
    int n_added_666_333;
    int n_added_666_666;
    int n_added_6_6_rolling;

    std::list<ChContactNSC_6_6*>::iterator lastcontact_6_6;
    std::list<ChContactNSC_6_3*>::iterator lastcontact_6_3;
    std::list<ChContactNSC_3_3*>::iterator lastcontact_3_3;
    std::list<ChContactNSC_333_3*>::iterator lastcontact_333_3;
    std::list<ChContactNSC_333_6*>::iterator lastcontact_333_6;
    std::list<ChContactNSC_333_333*>::iterator lastcontact_333_333;
    std::list<ChContactNSC_666_3*>::iterator lastcontact_666_3;
    std::list<ChContactNSC_666_6*>::iterator lastcontact_666_6;
    std::list<ChContactNSC_666_333*>::iterator lastcontact_666_333;
    std::list<ChContactNSC_666_666*>::iterator lastcontact_666_666;

    std::list<ChContactNSCrolling_6_6*>::iterator lastcontact_6_6_rolling;

    std::unordered_map<ChContactable*, ForceTorque> contact_forces;

  public:
    ChContactContainerNSC();
    ChContactContainerNSC(const ChContactContainerNSC& other);
    virtual ~ChContactContainerNSC();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChContactContainerNSC* Clone() const override { return new ChContactContainerNSC(*this); }

    /// Report the number of added contacts.
    virtual int GetNcontacts() const override {
        return n_added_3_3 + n_added_6_3 + n_added_6_6 + n_added_333_3 + n_added_333_6 + n_added_333_333 +
               n_added_666_3 + n_added_666_6 + n_added_666_333 + n_added_666_666 + n_added_6_6_rolling;
    }

    /// Remove (delete) all contained contact data.
    virtual void RemoveAllContacts() override;

    /// The collision system will call BeginAddContact() before adding all contacts (for example with AddContact() or
    /// similar). Instead of simply deleting all list of the previous contacts, this optimized implementation rewinds
    /// the link iterator to begin and tries to reuse previous contact objects until possible, to avoid too much
    /// allocation/deallocation.
    virtual void BeginAddContact() override;

    /// Add a contact between two frames.
    virtual void AddContact(const collision::ChCollisionInfo& mcontact) override;

    /// The collision system will call BeginAddContact() after adding all contacts (for example with AddContact() or
    /// similar). This optimized version purges the end of the list of contacts that were not reused (if any).
    virtual void EndAddContact() override;

    /// Scan all the contacts and for each contact executes the OnReportContact() function of the provided callback
    /// object.
    virtual void ReportAllContacts(ReportContactCallback* mcallback) override;

    /// Report the number of scalar unilateral constraints.
    /// Note: friction constraints aren't exactly unilaterals, but they are still counted.
    virtual int GetDOC_d() override {
        return 3 * (n_added_3_3 + n_added_6_3 + n_added_6_6 + n_added_333_3 + n_added_333_6 + n_added_333_333 +
                    n_added_666_3 + n_added_666_6 + n_added_666_333 + n_added_666_666) +
               6 * (n_added_6_6_rolling);
    }

    /// Update state of this contact container: compute jacobians, violations, etc.
    /// and store results in inner structures of contacts.
    virtual void Update(double mtime, bool update_assets = true) override;

    /// Compute contact forces on all contactable objects in this container.
    /// This function caches contact forces in a map.
    virtual void ComputeContactForces() override;

    /// Return the resultant contact force acting on the specified contactable object.
    virtual ChVector<> GetContactableForce(ChContactable* contactable) override;

    /// Return the resultant contact torque acting on the specified contactable object.
    virtual ChVector<> GetContactableTorque(ChContactable* contactable) override;

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

CH_CLASS_VERSION(ChContactContainerNSC, 0)

}  // end namespace chrono

#endif
