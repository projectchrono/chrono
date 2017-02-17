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

#ifndef CHCONTACTCONTAINERBASE_H
#define CHCONTACTCONTAINERBASE_H

#include <list>
#include <unordered_map>

#include "chrono/collision/ChCCollisionInfo.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChContactable.h"
#include "chrono/physics/ChMaterialCouple.h"

namespace chrono {

/// Class to be used as a callback interface for some user defined
/// action to be taken each time a contact is added to the container.
/// It can be used to modify the friction value (as default it is the
/// average of the friction of the two bodies).
/// The user should implement an inherited class and
/// implement a custom ContactCallback() function.
class ChApi ChAddContactCallback {
  public:
    virtual ~ChAddContactCallback() {}

    /// Callback used to report contact points being added to the container.
    /// This must be implemented by a child class of ChAddContactCallback
    virtual void ContactCallback(
        const collision::ChCollisionInfo& mcontactinfo,  ///< get info about contact (cannot change it)
        ChMaterialCouple& material                       ///< you can modify this!
        ) = 0;
};

/// Class to be used as a callback interface for some user defined
/// action to be taken for each contact (already added to the container,
/// maybe with already computed forces).
/// The user should implement an inherited class and
/// implement a custom ReportContactCallback() function.
class ChApi ChReportContactCallback {
  public:
    virtual ~ChReportContactCallback() {}

    /// Callback, used to report contact points already added to the container.
    /// This must be implemented by a child class of ChReportContactCallback.
    /// If returns false, the contact scanning will be stopped.
    virtual bool ReportContactCallback(
        const ChVector<>& pA,             ///< get contact pA
        const ChVector<>& pB,             ///< get contact pB
        const ChMatrix33<>& plane_coord,  ///< get contact plane coordsystem (A column 'X' is contact normal)
        const double& distance,           ///< get contact distance
        const ChVector<>& react_forces,   ///< get react.forces (if already computed). In coordsystem 'plane_coord'
        const ChVector<>& react_torques,  ///< get react.torques, if rolling friction (if already computed).
        ChContactable* contactobjA,       ///< get model A (note: some containers may not support it and could be zero!)
        ChContactable* contactobjB        ///< get model B (note: some containers may not support it and could be zero!)
        ) = 0;
};

/// Class representing a container of many contacts.
/// There might be implementations of this interface in form of plain CPU linked lists of contact objects,
/// or highly optimized GPU buffers, etc. This is only the basic interface with the features that are in common.
/// Struct to store resultant contact force/torque applied on rigid body
class ChApi ChContactContainerBase : public ChPhysicsItem {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChContactContainerBase)

  public:
    ChContactContainerBase() : add_contact_callback(NULL), report_contact_callback(NULL) {}
    ChContactContainerBase(const ChContactContainerBase& other);
    virtual ~ChContactContainerBase() {}

    /// Get the number of added contacts. To be implemented by child classes.
    virtual int GetNcontacts() const = 0;

    /// Remove (delete) all contained contact data. To be implemented by child classes.
    virtual void RemoveAllContacts() = 0;

    /// The collision system will call BeginAddContact() before adding
    /// all contacts (for example with AddContact() or similar). By default
    /// it deletes all previous contacts. Custom more efficient implementations
    /// might reuse contacts if possible.
    virtual void BeginAddContact() { RemoveAllContacts(); }

    /// Add a contact between two models, storing it into this container.
    /// To be implemented by child classes.
    /// Some specialized child classes (ex. one that uses GPU buffers)
    /// could implement also other more efficient functions to add many contacts
    /// in a batch (so that, for example, a special GPU collision system can exploit it);
    /// yet most collision system might still fall back to this function if no other
    /// specialized add-functions are found.
    virtual void AddContact(const collision::ChCollisionInfo& mcontact) = 0;

    /// The collision system will call EndAddContact() after adding
    /// all contacts (for example with AddContact() or similar). By default
    /// it does nothing.
    virtual void EndAddContact() {}

    /// Sets a callback to be used each time a contact point is
    /// added to the container. Note that not all child classes can
    /// support this function in all circumstances (example, the GPU container
    /// won't launch the callback for all its points because of performance optimization)
    void SetAddContactCallback(ChAddContactCallback* mcallback) { add_contact_callback = mcallback; }

    /// Gets the callback to be used each time a contact point is added to the container.
    ChAddContactCallback* GetAddContactCallback() { return add_contact_callback; }

    /// Scans all the contacts and for each contact executes the ReportContactCallback()
    /// function of the user object inherited from ChReportContactCallback.
    /// Child classes of ChContactContainerBase should try to implement this.
    virtual void ReportAllContacts(ChReportContactCallback* mcallback) {}

    /// Compute contact forces on all contactable objects in this container.
    /// If implemented by a derived class, these forces must be stored in the hash table
    /// contact_forces (with key a pointer to ChContactable and value a ForceTorque structure).
    virtual void ComputeContactForces() {}

    /// Return the resultant contact force acting on the specified contactable object.
    ChVector<> GetContactableForce(ChContactable* contactable);

    /// Return the resultant contact torque acting on the specified contactable object.
    ChVector<> GetContactableTorque(ChContactable* contactable);

    /// Method for serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method for de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);

  protected:
    struct ForceTorque {
        ChVector<> force;
        ChVector<> torque;
    };

    std::unordered_map<ChContactable*, ForceTorque> contact_forces;
    ChAddContactCallback* add_contact_callback;
    ChReportContactCallback* report_contact_callback;

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

CH_CLASS_VERSION(ChContactContainerBase,0)

}  // end namespace chrono

#endif
