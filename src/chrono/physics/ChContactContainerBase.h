//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCONTACTCONTAINERBASE_H
#define CHCONTACTCONTAINERBASE_H

#include "chrono/collision/ChCCollisionInfo.h"
#include "chrono/physics/ChMaterialCouple.h"
#include "chrono/physics/ChPhysicsItem.h"

namespace chrono {

/// Class to be used as a callback interface for some user defined
/// action to be taken each time a contact is added to the container.
/// It can be used to modify the friction value (as default it is the
/// average of the friction of the two bodies).
/// The user should implement an inherited class and
/// implement a custom ContactCallback() function.
class ChApi ChAddContactCallback {
  public:
    /// Callback, used to report contact points being added to the container.
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
class ChApi ChContactContainerBase : public ChPhysicsItem {
    CH_RTTI(ChContactContainerBase, ChPhysicsItem);

  protected:
    //
    // DATA
    //

    ChAddContactCallback* add_contact_callback;
    ChReportContactCallback* report_contact_callback;

  public:
    //
    // CONSTRUCTORS
    //

    ChContactContainerBase() {
        add_contact_callback = 0;
        report_contact_callback = 0;
    }

    virtual ~ChContactContainerBase() {}

    //
    // FUNCTIONS
    //

    /// Tell the number of added contacts. To be implemented by child classes.
    virtual int GetNcontacts() = 0;

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

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive) {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChPhysicsItem::ArchiveOUT(marchive);
        // serialize all member data:
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChPhysicsItem::ArchiveIN(marchive);
        // stream in all member data:
    }
};

}  // END_OF_NAMESPACE____

#endif
