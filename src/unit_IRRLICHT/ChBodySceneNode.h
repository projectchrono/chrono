//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHBODYSCENENODE_H
#define CHBODYSCENENODE_H


// =============================================================================
//  ***OBSOLETE*** 
//  better: use the ChIrrNodeAsset approach
// =============================================================================


#include <irrlicht.h>

#include "physics/ChSystem.h"
#include "unit_IRRLICHT/ChApiIrr.h"


#define ESNT_CHBODY 1200


namespace irr {
namespace scene {


class ChApiIrr ChBodySceneNode : public scene::ISceneNode
{

public:

  /// Build a scene node for the Irrlicht Engine. This scene node is also a
  /// rigid body for the Chrono::Engine multibody simulation.
  /// As soon as created, the wrapped ChBody is also added to Chrono system.
  /// To delete a ChBodyScene node from an Irrlicht scene, use the remove() 
  /// function only! (it will also be removed from the Chrono system)
  ChBodySceneNode(
    chrono::ChSystem* msystem,  ///< pointer to the Chrono::Engine physical simulation system
    IAnimatedMesh*    mesh,     ///< a 3D mesh for representing the shape of the body
    ISceneNode*       parent,   ///< the parent node in Irrlicht hierarchy
    ISceneManager*    mgr,      ///< the Irrlicht scene manager 
    s32               id        ///< the Irrlicht identifier
    );

  ChBodySceneNode(
    chrono::ChSystem*         msystem,  ///< pointer to the Chrono::Engine physical simulation system
    IAnimatedMesh*            mesh,     ///< a 3D mesh for representing the shape of the body
    ISceneNode*               parent,   ///< the parent node in Irrlicht hierarchy
    ISceneManager*            mgr,      ///< the Irrlicht scene manager 
    s32                       id,       ///< the Irrlicht identifier
    const chrono::ChVector<>& offset    ///< offset between mesh and body COG
    );

  /// Destructor.
  /// Note: as this Irrlicht node is destructed, it also automatically removes 
  /// the wrapped ChBody from the ChronoEngine.
  ~ChBodySceneNode();

  //
  // OVERRIDE/IMPLEMENT BASE IRRLICHT METHODS
  //

  virtual void OnRegisterSceneNode();

  virtual void render();

  virtual const core::aabbox3d<f32>& getBoundingBox() const;

  virtual void setMaterialTexture(s32              textureLayer,
                                  video::ITexture* texture);

  virtual u32 getMaterialCount();

  virtual video::SMaterial& getMaterial(u32 i);

  void OnAnimate(u32 timeMs);

  virtual IShadowVolumeSceneNode*
    addShadowVolumeSceneNode(const IMesh* shadowMesh = 0,
                             s32          id = -1,
                             bool         zfailmethod = true,
                             f32          infinity = 10000.0f);

  //
  // CHRONO::ENGINE SPECIFIC
  //

  /// Returns reference to the shared pointer which references the
  /// rigid body wrapped by this scene node.
  chrono::ChSharedPtr<chrono::ChBody>& GetBody() {return *bodyp;}

  /// Returns true if the node is moved by Chrono::Engine simulation system.
  virtual bool IsChronoControlled() const { return ChronoControlled; }

  /// Set true if you want Chrono::Engine to include this body in simulation.
  virtual void SetChronoControlled(const bool &controlled) {ChronoControlled = controlled;}

  IAnimatedMeshSceneNode* GetChildMesh() {return child_mesh;}

  virtual ESCENE_NODE_TYPE getType() const {return (ESCENE_NODE_TYPE)ESNT_CHBODY;}

private:

  core::aabbox3d<f32>     Box;
  IAnimatedMeshSceneNode* child_mesh;

  // Chrono Engine specific data
  chrono::ChSharedPtr<chrono::ChBody>*  bodyp;
  bool                                  ChronoControlled;

  static int  body_identifier;

};


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif

