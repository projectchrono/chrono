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

#ifndef CHINDEXEDNODES_H
#define CHINDEXEDNODES_H

#include "chrono/physics/ChNodeBase.h"

namespace chrono {

/// Interface class for clusters of points that can
/// be accessed with an index.
/// Must be inherited by children classes.

class ChApi ChIndexedNodes : public ChPhysicsItem {
  public:
    ChIndexedNodes() {}
    ChIndexedNodes(const ChIndexedNodes& other) : ChPhysicsItem(other) {}
    virtual ~ChIndexedNodes() {}

    /// Get the number of nodes
    virtual unsigned int GetNumNodes() const = 0;

    /// Access the N-th node
    virtual std::shared_ptr<ChNodeBase> GetNode(unsigned int n) const = 0;

    /// Add a new node to the particle cluster, passing a
    /// vector as initial position.
    //virtual void AddNode(ChVector3d initial_state) = 0;

    /// Resize the node cluster. Also clear the state of
    /// previously created particles, if any.
    //virtual void ResizeNnodes(int newsize) = 0;

    /// Number of coordinates of the node cluster
    //virtual unsigned int GetNumCoordsPosLevel() {return 3*GetNumNodes();}

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

CH_CLASS_VERSION(ChIndexedNodes, 0)

}  // end namespace chrono

#endif
