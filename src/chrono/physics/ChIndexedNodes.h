//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHINDEXEDNODES_H
#define CHINDEXEDNODES_H

//////////////////////////////////////////////////
//
//   ChIndexedNodes.h
//
//   Interface class for clusters of points that can
//   be accessed with an index.
//   Must be inherited by children classes.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChNodeBase.h"

namespace chrono {

/// Interface class for clusters of points that can
/// be accessed with an index.
/// Must be inherited by children classes.

class ChApi ChIndexedNodes : public ChPhysicsItem {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChIndexedNodes, ChPhysicsItem);

  private:
    //
    // DATA
    //

  public:
    //
    // CONSTRUCTORS
    //

    /// Build a cluster of nodes
    ChIndexedNodes();

    /// Destructor
    virtual ~ChIndexedNodes();

    //
    // FUNCTIONS
    //

    /// Get the number of nodes
    virtual unsigned int GetNnodes() = 0;

    /// Access the N-th node
    virtual std::shared_ptr<ChNodeBase> GetNode(unsigned int n) = 0;

    /// Add a new node to the particle cluster, passing a
    /// vector as initial position.
    //	virtual void AddNode(ChVector<double> initial_state) =0;

    /// Resize the node cluster. Also clear the state of
    /// previously created particles, if any.
    //	virtual void ResizeNnodes(int newsize) =0;

    /// Number of coordinates of the node cluster
    //	virtual int GetDOF  ()   {return 3*GetNnodes();}

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);
};

}  // END_OF_NAMESPACE____

#endif
