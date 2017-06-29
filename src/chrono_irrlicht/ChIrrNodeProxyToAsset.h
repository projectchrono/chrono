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

#ifndef CHIRRNODEPROXYTOASSET_H
#define CHIRRNODEPROXYTOASSET_H

#include <irrlicht.h>

#include "chrono_irrlicht/ChApiIrr.h"

#define ESNT_CHIRRNODEPROXYTOASSET 1202

namespace chrono {
namespace irrlicht {

/// @addtogroup irrlicht_module
/// @{

/// Class for proxy to ChAsset, it is a node with mesh in Irrlicht system
/// and a shared pointer to the ChAsset to whom it corresponds.
/// Example: (with ascii art, with --> shared pointer, ...> raw pointer)
///
///   CHRONO side                  IRRLICHT side
///     ChBody  <......................._
///        ChIrrNodeAsset  -------->  ChIrrNode
///        ChBoxShape  <--------------   ChIrrNodeProxyToAsset
///                                           IMeshSceneNode
///        ChSphereShape  <------------  ChIrrNodeProxyToAsset
///                                           IMeshSceneNode
class ChApiIrr ChIrrNodeProxyToAsset : public irr::scene::ISceneNode {
  private:
    irr::core::aabbox3d<irr::f32> Box;

    std::shared_ptr<ChAsset> visualization_asset;

    bool do_update;

  public:
    /// Constructor
    ChIrrNodeProxyToAsset(
        std::shared_ptr<ChAsset> myvisualization,  ///< pointer to the ChronoEngine visualization asset
        irr::scene::ISceneNode* parent           ///< the parent node in Irrlicht hierarchy
    );

    /// Destructor.
    ~ChIrrNodeProxyToAsset() {}

    // The following two functions must be defined here since they are abstract
    // in the base class.
    virtual void render() {}
    virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const { return Box; }

    ISceneNode* clone(ISceneNode* newParent, irr::scene::ISceneManager* newManager);

    //
    // CHRONO::ENGINE SPECIFIC
    //

    /// Returns a reference to the shared pointer which references the ChAsset to
    /// whom this is a proxy.
    std::shared_ptr<ChAsset>& GetVisualizationAsset() { return visualization_asset; }

    /// Returns true if the node must recompute the mesh for each time that an
    /// Update is called.
    virtual bool IsUpdateEnabled() const { return do_update; }

    /// Set if the node must recompute the mesh for each time that an Update is
    /// called.
    virtual void SetUpdateEnabled(const bool mup) { do_update = mup; }

    /// Updates the child mesh to reflect the ChAsset.
    virtual void Update();

    virtual irr::scene::ESCENE_NODE_TYPE getType() const {
        return (irr::scene::ESCENE_NODE_TYPE)ESNT_CHIRRNODEPROXYTOASSET;
    }
};

/// @} irrlicht_module

}  // end namespace irrrlicht
}  // end namespace chrono

#endif
