// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================

#pragma once

#include <list>

#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChContactTuple.h"
#include "chrono_parallel/ChApiParallel.h"
#include "chrono_parallel/ChDataManager.h"

namespace chrono {

/// @addtogroup parallel_collision
/// @{

/// Class representing a container of many contacts, implemented as a linked list of contact tuples.
/// This container is used only for reporting geometric information about the contact pairs and is therefore suitable
/// for both NSC and SMC systems. Currently, only contacts between rigid bodies are considered.
class CH_PARALLEL_API ChContactContainerParallel : public ChContactContainer {
  public:
    typedef ChContactTuple<ChContactable_1vars<6>, ChContactable_1vars<6> > ChContact_6_6;

    ChContactContainerParallel(ChParallelDataManager* dc);
    ChContactContainerParallel(const ChContactContainerParallel& other);
    ~ChContactContainerParallel();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChContactContainerParallel* Clone() const override { return new ChContactContainerParallel(*this); }

    virtual int GetNcontacts() const override { return data_manager->num_rigid_contacts; }

    virtual void RemoveAllContacts() override;
    virtual void BeginAddContact() override;
    virtual void AddContact(const collision::ChCollisionInfo& mcontact) override;
    virtual void EndAddContact() override;

    /// Specify a callback object to be used each time a contact point is added to the container.
    virtual void RegisterAddContactCallback(AddContactCallback* mcallback) override {
        data_manager->add_contact_callback = mcallback;
    }

    /// Get the callback object to be used each time a contact point is added to the container.
    virtual AddContactCallback* GetAddContactCallback() override { return data_manager->add_contact_callback; }

    /// Scan all the contacts and for each contact executes the OnReportContact()
    /// function of the provided callback object.
    /// Note: currently, the contact reaction force and torque are not set (always zero).
    virtual void ReportAllContacts(ReportContactCallback* callback) override;

    /// Compute contact forces on all contactable objects in this container.
    /// Note that this function must be explicitly called by the user at each time where
    /// calls to GetContactableForce or ContactableTorque are made.
    virtual void ComputeContactForces() override;

    /// Return the resultant contact force acting on the specified contactable object.
    /// Note that ComputeContactForces must be called prior to calling this function
    /// at any time where reporting of contact forces is desired.
    virtual ChVector<> GetContactableForce(ChContactable* contactable) override;

    /// Return the resultant contact torque acting on the specified contactable object.
    /// Note that ComputeContactForces must be called prior to calling this function
    /// at any time where reporting of contact torques is desired.
    virtual ChVector<> GetContactableTorque(ChContactable* contactable) override;

    /// Return the list of contacts between rigid bodies
    const std::list<ChContact_6_6*>& GetContactList() const { return contactlist_6_6; }

    ChParallelDataManager* data_manager;

  private:
    int n_added_6_6;
    std::list<ChContact_6_6*> contactlist_6_6;
    std::list<ChContact_6_6*>::iterator lastcontact_6_6;
};

/// @} parallel_colision

}  // end namespace chrono
