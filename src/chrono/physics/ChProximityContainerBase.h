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

#ifndef CHPROXIMITYCONTAINERBASE_H
#define CHPROXIMITYCONTAINERBASE_H

///////////////////////////////////////////////////
//
//   ChProximityContainerBase.h
//
//   Class for container of many contacts
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChPhysicsItem.h"
#include "physics/ChMaterialCouple.h"
#include "collision/ChCCollisionInfo.h"

namespace chrono {

/// Class to be used as a callback interface for some user defined
/// action to be taken each time a proximity info is added to the container.

class ChApi ChAddProximityCallback {
  public:
    /// Callback, used to report proximity pairs being added to the container.
    /// This must be implemented by a child class of ChAddProximityCallback
    virtual void ProximityCallback(const collision::ChCollisionModel& modA,  ///< get contact model 1
                                   const collision::ChCollisionModel& modB   ///< get contact model 2
                                   ) = 0;
};

/// Class to be used as a callback interface for some user defined
/// action to be taken for each proximity pair (already added to the container).
/// The user should implement an inherited class and
/// implement a custom ReportProximityCallback() function.

class ChApi ChReportProximityCallback {
  public:
    /// Callback, used to report contact points already added to the container.
    /// This must be implemented by a child class of ChReportContactCallback.
    /// If returns false, the contact scanning will be stopped.
    virtual bool ReportProximityCallback(
        collision::ChCollisionModel*
            modA,  ///< get model A (note: some containers may not support it and could be zero!)
        collision::ChCollisionModel*
            modB  ///< get model B (note: some containers may not support it and could be zero!)
        ) = 0;
};

///
/// Class representing the interface for containers of proximity pairs,
/// that is pairs of collision models that have been obtained from the
/// broadphase collision.
/// There might be implementations of this interface
/// in form of plain CPU linked lists of objects (ex. springs or similar
/// forcefields for cloth simulation etc.) or highly optimized GPU buffers,
/// etc. etc.
/// This is only the basic interface with the features that are in common.
///

class ChApi ChProximityContainerBase : public ChPhysicsItem {
    CH_RTTI(ChProximityContainerBase, ChPhysicsItem);

  protected:
    //
    // DATA
    //

    ChAddProximityCallback* add_proximity_callback;
    ChReportProximityCallback* report_proximity_callback;

  public:
    //
    // CONSTRUCTORS
    //

    ChProximityContainerBase() {
        add_proximity_callback = 0;
        report_proximity_callback = 0;
    };

    virtual ~ChProximityContainerBase(){};

    //
    // FUNCTIONS
    //

    /// Tell the number of added proximity pairs. To be implemented by child classes.
    virtual int GetNproximities() = 0;

    /// Remove (delete) all contained proximity pairs. To be implemented by child classes.
    virtual void RemoveAllProximities() = 0;

    /// The collision system will call BeginAddProximity() before adding
    /// all proximity pairs (for example with AddProximity() or similar). By default
    /// it deletes all previous contacts. Custom more efficient implementations
    /// might reuse contacts if possible.
    virtual void BeginAddProximities() { RemoveAllProximities(); }

    /// Add a proximity pair between two collision models, storing it into this container.
    /// To be implemented by child classes.
    /// Some specialized child classes (ex. one that uses GPU buffers)
    /// could implement also other more efficient functions to add many proximity pairs
    /// in a batch (so that, for example, a special GPU collision system can exploit it);
    /// yet most collision system might still fall back to this function if no other
    /// specialized add-functions are found.
    virtual void AddProximity(collision::ChCollisionModel* modA,  ///< get contact model 1
                              collision::ChCollisionModel* modB   ///< get contact model 2
                              ) = 0;

    /// The collision system will call this after adding
    /// all pairs (for example with AddProximity() or similar). By default
    /// it does nothing.
    virtual void EndAddProximities(){};

    /// Sets a callback to be used each time a proximity pair is
    /// added to the container. Note that not all child classes can
    /// support this function in all circumstances (example, the GPU container
    /// won't launch the callback for all its points because of performance optimization)
    void SetAddProximityCallback(ChAddProximityCallback* mcallback) { add_proximity_callback = mcallback; }

    /// Scans all the contacts and for each proximity pair executes the ReportProximityCallback()
    /// function of the user object inherited from ChReportContactCallback.
    /// Child classes of ChContactContainerBase should try to implement this.
    virtual void ReportAllProximities(ChReportProximityCallback* mcallback) = 0;


    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChPhysicsItem::ArchiveOUT(marchive);
        // serialize all member data:
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChPhysicsItem::ArchiveIN(marchive);
        // stream in all member data:
    }
};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____

#endif
