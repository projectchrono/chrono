//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHIRRASSETCONVERTER_H
#define CHIRRASSETCONVERTER_H

#include <string>
#include <irrlicht.h>

#include "assets/ChCamera.h"
#include "assets/ChBoxShape.h"
#include "assets/ChCylinderShape.h"
#include "assets/ChSphereShape.h"
#include "assets/ChObjShapeFile.h"
#include "assets/ChTexture.h"
#include "assets/ChVisualization.h"
#include "assets/ChColorAsset.h"
#include "assets/ChAssetLevel.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChGlyphs.h"

#include "unit_IRRLICHT/ChApiIrr.h"
#include "unit_IRRLICHT/ChBodySceneNodeTools.h"
#include "unit_IRRLICHT/ChIrrNodeAsset.h"


namespace irr {
namespace scene {


/// Class with static functions which allow creation of Irrlicht frequent
/// 'scene nodes' like lights, camera, sky box etc. with very simple statements.

class ChApiIrr ChIrrAssetConverter
{
public:

  // shared meshes
  IAnimatedMesh*      sphereMesh;
  IMesh*              cubeMesh;
  IMesh*              cylinderMesh;

  ISceneManager*      scenemanager;
  IrrlichtDevice*     mdevice;

  ChIrrAppInterface*  minterface;

  chrono::ChCamera*   mcamera;
  bool                camera_found_in_assets;


  /// Constructor
  ChIrrAssetConverter(ChIrrAppInterface& ainterface);

  /// Destructor
  ~ChIrrAssetConverter();

  /// Set the directory where the cube, cylinder, etc. primitives are stored as
  /// .obj  files (by default, it is "../data/"). Must be set _before_ creating
  /// the ChIrrApp or the ChIrrAssetConverter (it is a static method)
  static void SetDefaultObjectDir(std::string mdir) {irrlicht_default_obj_dir = mdir;}

  /// Returns the proxy to the ChIrrNode, by scanning all assets.
  /// Note, check for the returned pointer, using mnode.IsNull(), just in case a
  /// proxy has not been added.
  chrono::ChSharedPtr<chrono::ChIrrNodeAsset> 
    GetIrrNodeAsset(chrono::ChSharedPtr<chrono::ChPhysicsItem> mitem);

  /// Shortcut to add and bind a ChIrrNodeAsset to an item, if it has not been
  /// added previously.
  void Bind(chrono::ChSharedPtr<chrono::ChPhysicsItem> mitem);

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
  void Update(chrono::ChSharedPtr<chrono::ChPhysicsItem> mitem);

  /// For all items in a ChSystem, this function sets up the Irrlicht nodes 
  /// corresponding to the geometric assets that have been added to the items.
  /// NOTE. This must be done after the ChIrrNodeAsset has been created and
  /// bound, for example via Bind().
  /// NOTE. This conversion should be done only if needed (e.g. at the beginning
  /// of an animation), i.e. not too often, for performance reasons.
  void UpdateAll();

  /// Clean all Irrlicht stuff that has been put in the ChIrrNode in a previous
  /// Update or PopulateIrrlicht operation.
  void CleanIrrlicht(chrono::ChSharedPtr<chrono::ChPhysicsItem> mitem);

private:

  void PopulateIrrlicht(chrono::ChSharedPtr<chrono::ChPhysicsItem> mitem);

  void mflipSurfacesOnX(scene::IMesh* mesh) const;

  void _recursePopulateIrrlicht( 
            std::vector< chrono::ChSharedPtr<chrono::ChAsset> >&  assetlist,
            chrono::ChFrame<>                                     parentframe,
            ISceneNode*                                           mnode);

};


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif

