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

#ifndef CHIRRASSETCONVERTER_H
#define CHIRRASSETCONVERTER_H

#include <string>
#include <irrlicht.h>

#include "chrono/assets/ChCamera.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChObjShapeFile.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChVisualization.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChAssetLevel.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChGlyphs.h"
#include "chrono/assets/ChPathShape.h"
#include "chrono/assets/ChLineShape.h"

#include "chrono_irrlicht/ChApiIrr.h"
#include "chrono_irrlicht/ChIrrNodeAsset.h"

namespace chrono {
namespace irrlicht {

/// @addtogroup irrlicht_module
/// @{

/// Class with static functions which allow creation of Irrlicht frequent
/// 'scene nodes' like lights, camera, sky box etc. with very simple statements.
class ChApiIrr ChIrrAssetConverter {
  public:
    // shared meshes
    irr::scene::IAnimatedMesh* sphereMesh;
    irr::scene::IMesh* cubeMesh;
    irr::scene::IMesh* cylinderMesh;

    irr::scene::ISceneManager* scenemanager;
    irr::IrrlichtDevice* mdevice;

    ChIrrAppInterface* minterface;

    ChCamera* mcamera;
    bool camera_found_in_assets;

    /// Constructor
    ChIrrAssetConverter(ChIrrAppInterface& ainterface);

    /// Destructor
    ~ChIrrAssetConverter();

    /// Returns the proxy to the ChIrrNode, by scanning all assets.
    /// Note, check for the returned pointer, just in case a proxy has not been added.
    std::shared_ptr<ChIrrNodeAsset> GetIrrNodeAsset(std::shared_ptr<ChPhysicsItem> mitem);

    /// Shortcut to add and bind a ChIrrNodeAsset to an item, if it has not been
    /// added previously.
    void Bind(std::shared_ptr<ChPhysicsItem> mitem);

    /// Shortcut to add and bind a ChIrrNodeAsset to all items in a ChSystem.
    /// If it has been already added, the existing ChIrrNodeAsset is used.
    /// NOTE. If you want a finer control on which item has an Irrlicht proxy,
    /// and which other does not need it, just use Bind() on a per-item basis..
    /// NOTE. This conversion should be done only if needed (e.g. at the beginning
    /// of an animation), i.e. not too often, for performance reasons.
    void BindAll();

    /// This function sets up the Irrlicht nodes corresponding to the geometric
    /// assets that are found in the ChPhysicsItem 'mitem'. For example, if one
    /// has added a ChSphereShape and a ChBoxShape to the assets of a ChBody, and
    /// a ChIrrNodeAsset too, this Update() function will prepare a ISceneNode in
    /// Irrlicht (precisely, a ChIrrNode node) and it will fill it with a
    /// spherical triangle mesh, and a box triangle mesh.
    /// NOTE. This must be done after the ChIrrNodeAsset has been created and
    /// bound, for example via Bind().
    /// NOTE. This conversion should be done only if needed (e.g. at the beginning
    /// of an animation or when a shape changes), i.e. not too often, for
    /// performance reasons.
    void Update(std::shared_ptr<ChPhysicsItem> mitem);

    /// For all items in a ChSystem, this function sets up the Irrlicht nodes
    /// corresponding to the geometric assets that have been added to the items.
    /// NOTE. This must be done after the ChIrrNodeAsset has been created and
    /// bound, for example via Bind().
    /// NOTE. This conversion should be done only if needed (e.g. at the beginning
    /// of an animation), i.e. not too often, for performance reasons.
    void UpdateAll();

    /// Clean all Irrlicht stuff that has been put in the ChIrrNode in a previous
    /// Update or PopulateIrrlicht operation.
    void CleanIrrlicht(std::shared_ptr<ChPhysicsItem> mitem);

  private:
    void PopulateIrrlicht(std::shared_ptr<ChPhysicsItem> mitem);

    void mflipSurfacesOnX(irr::scene::IMesh* mesh) const;

    void _recursePopulateIrrlicht(std::vector<std::shared_ptr<ChAsset> >& assetlist,
                                  ChFrame<> parentframe,
                                  irr::scene::ISceneNode* mnode);

    void BindAllContentsOfAssembly(const ChAssembly* massy, std::unordered_set<const ChAssembly*>& mtrace);
    void UpdateAllContentsOfAssembly(const ChAssembly* massy, std::unordered_set<const ChAssembly*>& mtrace);
};

/// @} irrlicht_module

}  // end namespace irrlicht
}  // end namespace chrono

#endif
