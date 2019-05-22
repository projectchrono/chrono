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

#ifndef CH_CONTACT_CONTAINER_H
#define CH_CONTACT_CONTAINER_H

#include <list>
#include <unordered_map>

#include "chrono/collision/ChCCollisionInfo.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/physics/ChMaterialSurface.h"

namespace chrono {

/// Class representing a container of many contacts.
class ChApi ChContactContainer : public ChPhysicsItem {
  public:
    ChContactContainer() : add_contact_callback(nullptr), report_contact_callback(nullptr) {}
    ChContactContainer(const ChContactContainer& other);
    virtual ~ChContactContainer() {}

    /// Get the number of added contacts.
    virtual int GetNcontacts() const = 0;

    /// Remove (delete) all contained contact data.
    virtual void RemoveAllContacts() = 0;

    /// The collision system will call BeginAddContact() before adding all contacts (for example with AddContact() or
    /// similar). By default it deletes all previous contacts. More efficient implementations might reuse contacts if
    /// possible.
    virtual void BeginAddContact() { RemoveAllContacts(); }

    /// Add a contact between two models, storing it into this container.
    virtual void AddContact(const collision::ChCollisionInfo& mcontact) = 0;

    /// The collision system will call EndAddContact() after adding all contacts (for example with AddContact() or
    /// similar).
    virtual void EndAddContact() {}

    /// Class to be used as a callback interface for some user defined action to be taken
    /// each time a contact is added to the container.
    /// It can be used to modify the composite material properties for the contact pair.
    class ChApi AddContactCallback {
      public:
        virtual ~AddContactCallback() {}

        /// Callback used to process contact points being added to the container.
        /// A derived user-provided callback class must implement this. The provided
        /// composite material should be downcast to the appropriate type.
        virtual void OnAddContact(
            const collision::ChCollisionInfo& contactinfo,  ///< information about the collision pair
            ChMaterialComposite* const material             ///< composite material can be modified
            ) = 0;
    };

    /// Specify a callback object to be used each time a contact point is added to the container.
    /// Note that derived classes may not support this. If supported, the OnAddContact() method
    /// of the provided callback object will be called for each contact pair to allow modifying the
    /// composite material properties.
    virtual void RegisterAddContactCallback(AddContactCallback* mcallback) { add_contact_callback = mcallback; }

    /// Get the callback object to be used each time a contact point is added to the container.
    virtual AddContactCallback* GetAddContactCallback() { return add_contact_callback; }

    /// Class to be used as a callback interface for some user defined action to be taken
    /// for each contact (already added to the container, maybe with already computed forces).
    /// It can be used to report or post-process contacts.
    class ChApi ReportContactCallback {
      public:
        virtual ~ReportContactCallback() {}

        /// Callback used to report contact points already added to the container.
        /// If it returns false, the contact scanning will be stopped.
        virtual bool OnReportContact(
            const ChVector<>& pA,             ///< contact pA
            const ChVector<>& pB,             ///< contact pB
            const ChMatrix33<>& plane_coord,  ///< contact plane coordsystem (A column 'X' is contact normal)
            const double& distance,           ///< contact distance
            const double& eff_radius,         ///< effective radius of curvature at contact
            const ChVector<>& react_forces,   ///< react.forces (if already computed). In coordsystem 'plane_coord'
            const ChVector<>& react_torques,  ///< react.torques, if rolling friction (if already computed).
            ChContactable* contactobjA,  ///< model A (note: some containers may not support it and could be nullptr)
            ChContactable* contactobjB   ///< model B (note: some containers may not support it and could be nullptr)
            ) = 0;
    };

    /// Scan all the contacts and for each contact executes the OnReportContact() function of the provided callback
    /// object.
    virtual void ReportAllContacts(ReportContactCallback* mcallback) {}

    /// Compute contact forces on all contactable objects in this container.
    virtual void ComputeContactForces() {}

    /// Return the resultant contact force acting on the specified contactable object.
    virtual ChVector<> GetContactableForce(ChContactable* contactable) = 0;

    /// Return the resultant contact torque acting on the specified contactable object.
    virtual ChVector<> GetContactableTorque(ChContactable* contactable) = 0;

    /// Method for serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method for de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);

  protected:
    struct ForceTorque {
        ChVector<> force;
        ChVector<> torque;
    };

    AddContactCallback* add_contact_callback;
    ReportContactCallback* report_contact_callback;

    /// Utility function to accumulate contact forces from a specified list of contacts.
    /// This function is templated by the contact type (assumed to be derived from ChContactTuple).
    /// Contact forces are accumulated in a map keyed by the contactable objects.
    /// Derived ChContactContainer classes can use this utility (processing their various lists
    /// of contacts) to cache information used for reporting through GetContactableForce and
    /// GetContactableTorque.
    template <class Tcont>
    void SumAllContactForces(std::list<Tcont*>& contactlist,
                             std::unordered_map<ChContactable*, ForceTorque>& contactforces) {
        for (auto contact = contactlist.begin(); contact != contactlist.end(); ++contact) {
            // Extract information for current contact (expressed in global frame)
            ChMatrix33<> A = (*contact)->GetContactPlane();
            ChVector<> force_loc = (*contact)->GetContactForce();
            ChVector<> force = A.Matr_x_Vect(force_loc);
            ChVector<> p1 = (*contact)->GetContactP1();
            ChVector<> p2 = (*contact)->GetContactP2();

            // Calculate contact torque for first object (expressed in global frame).
            // Recall that -force is applied to the first object.
            ChVector<> torque1(0);
            if (ChBody* body = dynamic_cast<ChBody*>((*contact)->GetObjA())) {
                torque1 = Vcross(p1 - body->GetPos(), -force);
            }

            // If there is already an entry for the first object, accumulate.
            // Otherwise, insert a new entry.
            auto entry1 = contactforces.find((*contact)->GetObjA());
            if (entry1 != contactforces.end()) {
                entry1->second.force -= force;
                entry1->second.torque += torque1;
            } else {
                ForceTorque ft{-force, torque1};
                contactforces.insert(std::make_pair((*contact)->GetObjA(), ft));
            }

            // Calculate contact torque for second object (expressed in global frame).
            // Recall that +force is applied to the second object.
            ChVector<> torque2(0);
            if (ChBody* body = dynamic_cast<ChBody*>((*contact)->GetObjB())) {
                torque2 = Vcross(p2 - body->GetPos(), force);
            }

            // If there is already an entry for the first object, accumulate.
            // Otherwise, insert a new entry.
            auto entry2 = contactforces.find((*contact)->GetObjB());
            if (entry2 != contactforces.end()) {
                entry2->second.force += force;
                entry2->second.torque += torque2;
            } else {
                ForceTorque ft{force, torque2};
                contactforces.insert(std::make_pair((*contact)->GetObjB(), ft));
            }
        }
    }
};

CH_CLASS_VERSION(ChContactContainer, 0)

}  // end namespace chrono

#endif
