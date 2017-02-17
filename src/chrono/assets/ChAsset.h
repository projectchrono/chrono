//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHASSET_H
#define CHASSET_H

#include "chrono/core/ChCoordsys.h"
#include "chrono/serialization/ChArchive.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

// Forward
class ChPhysicsItem;


/// Classes for adding user data (such as rendering
/// shapes, reference to files) to ChPhysicsItem objects.
/// User can inherit his classes for custom assets from
/// this class.
class ChApi ChAsset {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChAsset)

  protected:
    //
    // DATA
    //

  public:
    //
    // CONSTRUCTORS
    //

    ChAsset(){};

    virtual ~ChAsset(){};

    //
    // FUNCTIONS
    //

        /// This is called by the owner, i.e. a ChPhysicsItem. Note that 
        /// the ChAssets can be shared between owners, so an asset might receive
        /// different updates from different 'updater's each with different 'coords'.
    virtual void Update(ChPhysicsItem* updater, const ChCoordsys<>& coords){};

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite<ChAsset>();
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead<ChAsset>();
    }
};

CH_CLASS_VERSION(ChAsset,0)


/// @} chrono_assets

}  // end namespace chrono

#endif
