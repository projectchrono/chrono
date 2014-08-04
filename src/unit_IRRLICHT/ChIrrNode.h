//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.


#ifndef CHIRRNODE_H
#define CHIRRNODE_H

#include <irrlicht.h>

#include "physics/ChSystem.h"
#include "physics/ChPhysicsItem.h"

#include "unit_IRRLICHT/ChApiIrr.h"
#include "unit_IRRLICHT/ChIrrNodeProxyToAsset.h"
#include "unit_IRRLICHT/ChIrrTools.h"


#define ESNT_CHIRRNODE 1201


namespace irr {
namespace scene {


/// Class for Irrlicht visualization. It will be managed by a ChIrrNodeAsset 
/// asset to be added among the ChBody assets. Such ChIrrNode can automatically
/// be populated with Irrlicht meshes, each inside a ChIrrNodeProxyToAsset, 
/// that will correspond to shapes that have been added as ChVisualization
/// assets in ChBody.
///
/// Example: (with ascii art, with --> shared pointer, ...> raw pointer)
///
///   CHRONO side                  IRRLICHT side
///     ChBody  <.......................
///        ChIrrNodeAsset  -------->  ChIrrNode
///        ChBoxShape  <--------------   ChIrrNodeProxyToAsset
///                                           IMeshSceneNode
///        ChSphereShape  <------------  ChIrrNodeProxyToAsset
///                                           IMeshSceneNode

class ChApiIrr ChIrrNode : public scene::ISceneNode
{

public:

  /// Build a scene node for the Irrlicht Engine.
  /// This scene node also has a pointer to a rigid body for the Chrono::Engine 
  /// multibody simulation.  To delete a ChIrrSceneNode node from an Irrlicht
  /// scene, use the remove() function from the Irrlicht side (it won't delete
  /// the C::E body, though), or better delete the corresponding ChIrrlichtObj
  /// asset from the C::E side, or delete the full C::E body.
  ChIrrNode(
    chrono::ChSharedPtr<chrono::ChPhysicsItem> mphysicsitem,  ///< pointer to the Chrono::Engine item (es. rigid body)
    ISceneNode*                                parent,        ///< the parent node in Irrlicht hierarchy
    ISceneManager*                             mgr,           ///< the Irrlicht scene manager 
    s32                                        id);           ///< the Irrlicht identifier

  /// Destructor.
  ~ChIrrNode() {}

  //
  // OVERRIDE/IMPLEMENT BASE IRRLICHT METHODS
  //

  virtual void OnRegisterSceneNode();

  // The following functions must be defined here since they are abstract in the
  // base class.
  virtual void render() {}
  virtual const core::aabbox3d<f32>& getBoundingBox() const {return Box;}
  virtual void setMaterialTexture (s32 textureLayer, video::ITexture *texture) {}

  ISceneNode* clone(ISceneNode*    newParent,
                    ISceneManager* newManager);

  /// This function is needed only when using the 'clones' feature, i.e. shapes
  /// in assets define a sample that must be cloned (ex in ChParticleClones)
  bool SetupClones();

  void OnAnimate(u32 timeMs);

  //
  // CHRONO::ENGINE SPECIFIC
  //

  /// Returns reference to the shared pointer which references the ChPhysicsItem
  /// (ex. a ChBody) wrapped by this scene node.
  chrono::ChSharedPtr<chrono::ChPhysicsItem> GetPhysicsItem();

  /// Returns true if the node is moved by Chrono::Engine simulation system.
  virtual bool IsChronoControlled() const {return ChronoControlled;}

  /// Set true if the node must be moved by Chrono::Engine simulation system
  /// (it follows a ChPhysicsItem that contains a ChIrrNodeAsset, proxy to this).
  virtual void SetChronoControlled(const bool &controlled) {ChronoControlled = controlled;}

  /// Updates the children Irr mesh nodes to reflect the ChVisualization assets,
  /// by spawning the Update() command to all children ChIrrNodeProxyToAsset
  virtual void UpdateAssetsProxies();

  virtual ESCENE_NODE_TYPE getType() const {return (ESCENE_NODE_TYPE)ESNT_CHIRRNODE;}

private:

  void _recurse_update_asset_proxies(ISceneNode* mnode);

  core::aabbox3d<f32>     Box;
  chrono::ChPhysicsItem*  physicsitem;
  bool                    ChronoControlled;
};


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif

