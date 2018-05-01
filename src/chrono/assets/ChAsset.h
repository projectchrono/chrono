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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHASSET_H
#define CHASSET_H

#include "chrono/core/ChCoordsys.h"
#include "chrono/serialization/ChArchive.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

class ChPhysicsItem;

/// Classes for adding user data (such as rendering shapes, reference to files) to ChPhysicsItem objects.
class ChApi ChAsset {
  public:
    ChAsset() {}
    virtual ~ChAsset() {}

    /// This is called by the owner, i.e. a ChPhysicsItem. Note that
    /// the ChAssets can be shared between owners, so an asset might receive
    /// different updates from different 'updater's each with different 'coords'.
    virtual void Update(ChPhysicsItem* updater, const ChCoordsys<>& coords){};

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);
};

CH_CLASS_VERSION(ChAsset, 0)

/// @} chrono_assets

}  // end namespace chrono

#endif
