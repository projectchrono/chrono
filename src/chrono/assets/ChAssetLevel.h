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

#ifndef CHASSETLEVEL_H
#define CHASSETLEVEL_H

#include "chrono/assets/ChAsset.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChMatrixDynamic.h"

namespace chrono {

/// Class for grouping assets in a level. The level is like a 'subdirectory'.
/// A level can contain assets, as well as further levels (but avoid circular loops!)
/// A level can have custom rotation and translation respect its parent level.
class ChApi ChAssetLevel : public ChAsset {
  public:
    ChAssetLevel() : levelframe(CSYSNORM) {}

    virtual ~ChAssetLevel() {}

    /// Access the coordinate system information of the level, for setting/getting its position
    /// and rotation respect to its parent.
    ChFrame<>& GetFrame() { return levelframe; }

    /// Access to the list of children assets.
    std::vector<std::shared_ptr<ChAsset>>& GetAssets() { return assets; }

    /// Get the Nth asset in list
    std::shared_ptr<ChAsset> GetAssetN(unsigned int num);

    /// Add an asset to this level.
    void AddAsset(std::shared_ptr<ChAsset> masset) { this->assets.push_back(masset); }

    /// Updates all children assets, if any.
    /// Note that when calling Update() on children assets, their 'coords' will be the result
    /// of concatenating this frame csys and 'coords'.
    virtual void Update(ChPhysicsItem* updater, const ChCoordsys<>& coords) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  protected:
    ChFrame<> levelframe;

    std::vector<std::shared_ptr<ChAsset>> assets;
};

CH_CLASS_VERSION(ChAssetLevel, 0)

}  // end namespace chrono

#endif
