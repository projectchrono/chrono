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
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================

#pragma once

#include <list>

#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChContact.h"
#include "chrono/collision/multicore/ChCollisionData.h"
#include "chrono_multicore/ChApiMulticore.h"
#include "chrono_multicore/ChDataManager.h"

namespace chrono {

/// @addtogroup multicore_collision
/// @{

/// Class representing a container of many contacts, implemented as a linked list of contact tuples.
class CH_MULTICORE_API ChContactContainerMulticore : public ChContactContainer {
  public:
    ChContactContainerMulticore(ChMulticoreDataManager* dc);
    ChContactContainerMulticore(const ChContactContainerMulticore& other);
    virtual ~ChContactContainerMulticore();

    virtual unsigned int GetNumContacts() const override { return data_manager->cd_data->num_rigid_contacts; }

    virtual void RemoveAllContacts() override;
    virtual void BeginAddContact() override;
    virtual void EndAddContact() override;

    /// Specify a callback object to be used each time a contact point is added to the container.
    virtual void RegisterAddContactCallback(std::shared_ptr<AddContactCallback> mcallback) override {
        data_manager->add_contact_callback = mcallback;
    }

    /// Get the callback object to be used each time a contact point is added to the container.
    virtual std::shared_ptr<AddContactCallback> GetAddContactCallback() override {
        return data_manager->add_contact_callback;
    }

    /// Scan all the contacts and for each contact executes the OnReportContact()
    /// function of the provided callback object.
    /// Note: currently, the contact reaction force and torque are not set (always zero).
    virtual void ReportAllContacts(std::shared_ptr<ReportContactCallback> callback) override;

    /// Compute contact forces on all contactable objects in this container.
    /// Note that this function must be explicitly called by the user at each time where
    /// calls to GetContactableForce or ContactableTorque are made.
    virtual void ComputeContactForces() override;

    /// Return the resultant contact force acting on the specified contactable object.
    /// Note that ComputeContactForces must be called prior to calling this function
    /// at any time where reporting of contact forces is desired.
    virtual ChVector3d GetContactableForce(ChContactable* contactable) override;

    /// Return the resultant contact torque acting on the specified contactable object.
    /// Note that ComputeContactForces must be called prior to calling this function
    /// at any time where reporting of contact torques is desired.
    virtual ChVector3d GetContactableTorque(ChContactable* contactable) override;

    /// Return the list of contacts between rigid bodies
    const std::list<ChContact*>& GetContactList() const { return contacts; }

    /// Process the contact between the two specified collision shapes on the two specified bodies
    /// (compute composite material properties and load in global data structure).
    virtual void AddContact(int index, int b1, int s1, int b2, int s2) = 0;

    ChMulticoreDataManager* data_manager;

  protected:
    int n_added;
    std::list<ChContact*> contacts;
    std::list<ChContact*>::iterator last_contact;

    using ChContactContainer::AddContact;

    friend class ChSystemMulticoreNSC;
    friend class ChSystemMulticoreSMC;
};

/// @} multicore_colision

}  // end namespace chrono
