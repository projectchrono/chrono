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

#ifndef CHINDEXEDPARTICLES_H
#define CHINDEXEDPARTICLES_H

//////////////////////////////////////////////////
//
//   ChIndexedParticles.h
//
//   Interface class for clusters of 'particles' that can
//   be accessed with an index. Particles have 6 DOF.
//   Must be inherited by children classes.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <math.h>

#include "core/ChFrameMoving.h"
#include "physics/ChPhysicsItem.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)

class ChSystem;

/// Base class for a single particle to be used in ChIndexedParticles containers.
/// It is an item that has 6 degrees of freedom, like a moving frame.

class ChApi ChParticleBase : public ChFrameMoving<double> {
  public:
    ChParticleBase();
    virtual ~ChParticleBase();

    ChParticleBase(const ChParticleBase& other);             // Copy constructor
    ChParticleBase& operator=(const ChParticleBase& other);  // Assignment operator

    // Access the 'LCP variables' of the node
    virtual ChLcpVariables& Variables() = 0;
};

/// Interface class for clusters of particles that can
/// be accessed with an index.
/// Must be inherited by children classes.

class ChApi ChIndexedParticles : public ChPhysicsItem {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChIndexedParticles, ChPhysicsItem);

  private:
    //
    // DATA
    //

  public:
    //
    // CONSTRUCTORS
    //

    /// Build a cluster of particles.
    /// By default the cluster will contain 0 particles.
    ChIndexedParticles();

    /// Destructor
    ~ChIndexedParticles();

    //
    // FUNCTIONS
    //

    /// Get the number of particles
    virtual size_t GetNparticles() const = 0;

    /// Access the N-th particle
    virtual ChParticleBase& GetParticle(unsigned int n) = 0;

    /// Resize the particle cluster. Also clear the state of
    /// previously created particles, if any.
    virtual void ResizeNparticles(int newsize) = 0;

    /// Add a new particle to the particle cluster, passing a
    /// coordinate system as initial state.
    virtual void AddParticle(ChCoordsys<double> initial_state = CSYSNORM) = 0;

    /// Number of coordinates of the particle cluster, x7 because with quaternions for rotation
    virtual int GetDOF() { return 7 * (int)GetNparticles(); }
    /// Number of coordinates of the particle cluster, x6 because derivatives es. angular vel.
    virtual int GetDOF_w() { return 6 * (int)GetNparticles(); }

    /// Get the master coordinate system for the assets (this will return the
    /// main coordinate system of the rigid body)
    virtual ChFrame<> GetAssetsFrame(unsigned int nclone = 0) {
        ChFrame<> res;
        res = GetParticle(nclone);
        return (res);
    }
    virtual unsigned int GetAssetsFrameNclones() { return (unsigned int)GetNparticles(); }

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
