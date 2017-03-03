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

#ifndef CHINDEXEDPARTICLES_H
#define CHINDEXEDPARTICLES_H

#include <cmath>

#include "chrono/core/ChFrameMoving.h"
#include "chrono/physics/ChPhysicsItem.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)

class ChSystem;

/// Base class for a single particle to be used in ChIndexedParticles containers.
/// It is an item that has 6 degrees of freedom, like a moving frame.

class ChApi ChParticleBase : public ChFrameMoving<double> {
  public:
    ChParticleBase() {}
    ChParticleBase(const ChParticleBase& other) : ChFrameMoving<double>(other) {}
    virtual ~ChParticleBase() {}

    ChParticleBase& operator=(const ChParticleBase& other);

    // Access the variables of the node
    virtual ChVariables& Variables() = 0;
};

/// Interface class for clusters of particles that can
/// be accessed with an index.
/// Must be inherited by children classes.

class ChApi ChIndexedParticles : public ChPhysicsItem {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChIndexedParticles)

  public:
    ChIndexedParticles() {}
    ChIndexedParticles(const ChIndexedParticles& other) : ChPhysicsItem(other) {}
    virtual ~ChIndexedParticles() {}

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
    virtual int GetDOF() override { return 7 * (int)GetNparticles(); }
    /// Number of coordinates of the particle cluster, x6 because derivatives es. angular vel.
    virtual int GetDOF_w() override { return 6 * (int)GetNparticles(); }

    /// Get the master coordinate system for the assets (this will return the
    /// main coordinate system of the rigid body)
    virtual ChFrame<> GetAssetsFrame(unsigned int nclone = 0) override;

    virtual unsigned int GetAssetsFrameNclones() override { return (unsigned int)GetNparticles(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChIndexedParticles,0)

}  // end namespace chrono

#endif
