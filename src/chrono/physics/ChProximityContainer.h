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

#ifndef CH_PROXIMITY_CONTAINER_H
#define CH_PROXIMITY_CONTAINER_H

#include "chrono/collision/ChCollisionInfo.h"
#include "chrono/physics/ChPhysicsItem.h"

namespace chrono {

/// Class representing the interface for containers of proximity pairs,
/// that is pairs of collision models that have been obtained from the
/// broadphase collision.
/// There might be implementations of this interface
/// in form of plain CPU linked lists of objects (ex. springs or similar
/// forcefields for cloth simulation etc.) or highly optimized GPU buffers,
/// etc. etc.
/// This is only the basic interface with the features that are in common.
class ChApi ChProximityContainer : public ChPhysicsItem {
  public:
    ChProximityContainer() : add_proximity_callback(nullptr), report_proximity_callback(nullptr) {}
    ChProximityContainer(const ChProximityContainer& other);
    virtual ~ChProximityContainer() {}

    /// Tell the number of added proximity pairs. To be implemented by child classes.
    virtual int GetNproximities() const = 0;

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
    virtual void EndAddProximities() {}

    /// Class to be used as a callback interface for some user defined
    /// action to be taken each time a proximity info is added to the container.
    class ChApi AddProximityCallback {
      public:
        virtual ~AddProximityCallback() {}

        /// Callback used to process proximity pairs being added to the container.
        /// A derived user-provided callback class must implement this.
        virtual void OnAddProximity(const collision::ChCollisionModel& modA,  ///< contact model 1
                                    const collision::ChCollisionModel& modB   ///< contact model 2
                                    ) = 0;
    };

    /// Specify a callback object to be used each time a proximity pair is
    /// added to the container. Note that not all derived classes can support this.
    /// If supported, the OnAddProximity() method of the provided callback object
    /// will be called for each proximity pair as it is created.
    void RegisterAddProximityCallback(AddProximityCallback* mcallback) { add_proximity_callback = mcallback; }

    /// Class to be used as a callback interface for some user defined action to be taken
    /// for each proximity pair (already added to the container).
    /// It can be used to report or post-process proximity pairs.
    class ChApi ReportProximityCallback {
      public:
        virtual ~ReportProximityCallback() {}

        /// Callback used to report contact points already added to the container.
        /// If it returns false, the contact scanning will be stopped.
        virtual bool OnReportProximity(
            collision::ChCollisionModel* modA,  ///< model A (could be nullptr, if the container does not support it)
            collision::ChCollisionModel* modB   ///< model B (could be nullptr, if the container does not support it)
            ) = 0;
    };

    /// Scans all the proximity pairs and, for each pair, executes the OnReportProximity()
    /// function of the provided callback object.
    /// Derived classes of ChProximityContainer should try to implement this.
    virtual void ReportAllProximities(ReportProximityCallback* mcallback) = 0;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive);

  protected:
    AddProximityCallback* add_proximity_callback;
    ReportProximityCallback* report_proximity_callback;
};

}  // end namespace chrono

#endif
