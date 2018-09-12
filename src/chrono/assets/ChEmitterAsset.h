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

#ifndef CHEMITTERASSET_H
#define CHEMITTERASSET_H

#include "chrono/assets/ChAsset.h"
#include "chrono/core/ChFrame.h"
#include "chrono/particlefactory/ChParticleEmitter.h"

namespace chrono {

/// Class that attaches a ChParticleEmitter to a physics item (most often that item is a ChBody).
/// The emitter can then move together with the body.
class ChApi ChEmitterAsset : public ChAsset {
  protected:
    particlefactory::ChParticleEmitter memitter;
    double last_t;

  public:
    ChEmitterAsset() { last_t = 0; }

    virtual ~ChEmitterAsset() {}

    /// Access to the emitter.
    particlefactory::ChParticleEmitter& Emitter() { return this->memitter; }

    /// Updates the embedded emitter. If a dt is passed, it creates the particles.
    /// No need to call this by the user, it is called automatically by the asset owner (ie. the body).
    virtual void Update(ChPhysicsItem* updater, const ChCoordsys<>& coords) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChEmitterAsset, 0)

}  // end namespace chrono

#endif
