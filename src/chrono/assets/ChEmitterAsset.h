//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHEMITTERASSET_H
#define CHEMITTERASSET_H


#include "assets/ChAsset.h"
#include "core/ChFrame.h"
#include "particlefactory/ChParticleEmitter.h"

namespace chrono {

/// Class that attachs a ChParticleEmitter to a physics item (most
/// often that item is a ChBody).
/// The emitter can move together with the body, then.

class ChApi ChEmitterAsset : public ChAsset {
    // Chrono RTTI, needed for serialization
    CH_RTTI(ChEmitterAsset, ChAsset);

  protected:
    //
    // DATA
    //
    particlefactory::ChParticleEmitter memitter;

    double last_t;

  public:
    //
    // CONSTRUCTORS
    //

    ChEmitterAsset() { last_t =0; };

    virtual ~ChEmitterAsset(){};

    //
    // FUNCTIONS
    //


    /// Access to the emitter.
    particlefactory::ChParticleEmitter& Emitter() { return this->memitter; }


    /// Updates the embedded emitter. If a dt is passed, it creates the particles.
    /// No need to call this by the user, it is called automatically by the asset owner (ie. the body).
    virtual void Update(ChPhysicsItem* updater, const ChCoordsys<>& coords);


    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChAsset::ArchiveOUT(marchive);
        // serialize all member data:
       // marchive << CHNVP(memitter); //***TODO***
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
        // deserialize parent class
        ChAsset::ArchiveIN(marchive);
        // stream in all member data:
       // marchive >> CHNVP(memitter); //***TODO***
    }
};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____

#endif
