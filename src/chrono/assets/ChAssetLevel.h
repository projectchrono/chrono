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

#ifndef CHASSETLEVEL_H
#define CHASSETLEVEL_H

#include "chrono/assets/ChAsset.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChMatrixDynamic.h"

namespace chrono {

/// Base class for grouping assets in a level. The
/// level is like a 'subdirectory'. A level can contain
/// assets; amnog these, also further levels, etc. (but please
/// avoid circular loops!)
/// A level can have custom rotation and translation respect
/// its parent level.

class ChApi ChAssetLevel : public ChAsset {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChAssetLevel)

  protected:
    //
    // DATA
    //
    ChFrame<> levelframe;

    std::vector<std::shared_ptr<ChAsset> > assets;

  public:
    //
    // CONSTRUCTORS
    //

    ChAssetLevel() : levelframe(CSYSNORM){};

    virtual ~ChAssetLevel(){};

    //
    // FUNCTIONS
    //

    /// Access the coordinate sytem information of the level, for setting/getting its position
    /// and rotation respect to its parent.
    ChFrame<>& GetFrame() { return levelframe; }

    /// Access to the list of children assets.
    std::vector<std::shared_ptr<ChAsset> >& GetAssets() { return this->assets; }

    /// Get the Nth asset in list
    std::shared_ptr<ChAsset> GetAssetN(unsigned int num) {
        if (num < assets.size())
            return assets[num];
        else
            return std::shared_ptr<ChAsset>();
    }

    /// Add an asset
    void AddAsset(std::shared_ptr<ChAsset> masset) { this->assets.push_back(masset); }

    /// Updates all children assets, if any. Overrides default behaviour that does nothing.
    /// Note that when calls Update() on children assets, their 'coords' will be the result
    /// of concatenating this frame csys and 'coords'.
    virtual void Update(ChPhysicsItem* updater, const ChCoordsys<>& coords);


    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite<ChAssetLevel>();
        // serialize parent class
        ChAsset::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(levelframe);
        marchive << CHNVP(assets);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead<ChAssetLevel>();
        // deserialize parent class
        ChAsset::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(levelframe);
        marchive >> CHNVP(assets);
    }
};

CH_CLASS_VERSION(ChAssetLevel,0)

}  // end namespace chrono

#endif
