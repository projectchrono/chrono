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

#include "chrono/assets/ChAsset.h"
#include "chrono/assets/ChGlyphs.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/assets/ChPathShape.h"
#include "chrono/assets/ChSurfaceShape.h"

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
  public:
    ChIrrNodeProxyToAsset(std::shared_ptr<ChAsset> asset,  ///< Chrono visualization asset
                          irr::scene::ISceneNode* parent   ///< parent node in Irrlicht hierarchy
    );

    ~ChIrrNodeProxyToAsset() {}

    virtual void render() {}
    virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const { return Box; }

    ISceneNode* clone(ISceneNode* newParent, irr::scene::ISceneManager* newManager);

    /// Get the associated visualization asset. 
    std::shared_ptr<ChAsset>& GetVisualizationAsset() { return visualization_asset; }

    /// Update to reflect possible changes in the associated asset.
    virtual void Update();

    virtual irr::scene::ESCENE_NODE_TYPE getType() const {
        return (irr::scene::ESCENE_NODE_TYPE)ESNT_CHIRRNODEPROXYTOASSET;
    }

  private:
    irr::core::aabbox3d<irr::f32> Box;             ///< bounding box
    std::shared_ptr<ChAsset> visualization_asset;  ///< associated visualization asset
    bool initial_update;                           ///< flag forcing a first update

    void UpdateTriangleMesh(std::shared_ptr<ChTriangleMeshShape> trianglemesh);
    void UpdateTriangleMeshFixedConnectivity(std::shared_ptr<ChTriangleMeshShape> trianglemesh);
    void UpdateGlyphs(std::shared_ptr<ChGlyphs> glyphs);
    void UpdateSurface(std::shared_ptr<ChSurfaceShape> surface);
    void UpdateLine(std::shared_ptr<geometry::ChLine> line, unsigned int nvertexes);
};

/// @} irrlicht_module

}  // namespace irrlicht
}  // end namespace chrono

#endif
