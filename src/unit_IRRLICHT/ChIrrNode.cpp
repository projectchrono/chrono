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
// File author: A.Tasora


#include "core/ChLog.h"
#include "unit_IRRLICHT/ChIrrNode.h"
#include "unit_IRRLICHT/ChIrrNodeProxyToAsset.h"


namespace irr {
namespace scene {


ChIrrNode::ChIrrNode(chrono::ChSharedPtr<chrono::ChPhysicsItem> mphysicsitem,
                     ISceneNode*                                parent,
                     ISceneManager*                             mgr,
                     s32                                        id)
: ISceneNode(parent, mgr, id),
  ChronoControlled(true)
{
#ifdef _DEBUG
  setDebugName("ChIrrNode");
#endif

  // Set the shared pointer to the owner ChBody
  physicsitem = mphysicsitem.get_ptr();

  // set an unique identifier
  //static_item_identifier++;
  //GetPhysicsItem()->SetIdentifier(static_item_identifier);
}


void ChIrrNode::OnRegisterSceneNode()
{
  if (IsVisible)
    SceneManager->registerNodeForRendering(this);

  ISceneNode::OnRegisterSceneNode();
}


ISceneNode* ChIrrNode::clone(ISceneNode*    newParent,
                             ISceneManager* newManager)
{
  //chrono::GetLog() << "Cloning!\n";
  if (!newParent)
    newParent = Parent;
  if (!newManager)
    newManager = SceneManager;

  chrono::ChSharedPtr<chrono::ChPhysicsItem> shphysicsitem(this->physicsitem);
  shphysicsitem->AddRef(); // trick because shphysicsitem ctor took raw this->physicsitem that is not a new() 

  ChIrrNode* nb = new ChIrrNode(shphysicsitem, newParent,
    newManager, this->ID);

  nb->cloneMembers(this, newManager);
  nb->Box = this->Box;
  nb->ChronoControlled = this->ChronoControlled;
  nb->physicsitem = this->physicsitem;

  if ( newParent )
    nb->drop();
  return nb;
}


bool ChIrrNode::SetupClones()
{
  unsigned int needed_clones = physicsitem->GetAssetsFrameNclones();

  if (needed_clones)
  {
    unsigned int actual_clones = this->getChildren().getSize();

    // At least one clone 'sample' must be present (created with
    // ChIrrAssetConverter::Update...); otherwise just bail out.
    if (actual_clones ==0)
      return false;

    // Compare actual number of children and number of clones:
    // - do nothing if equal
    // - clone the sample (the first node child of this node) if fewer children
    // - delete clones if more children
    if (needed_clones == actual_clones)
      return true;

    while (needed_clones > actual_clones)
    {
      core::list<ISceneNode*>::ConstIterator it = this->getChildren().begin();
      (*it)->clone(); 
      ++actual_clones;
    }

    while (needed_clones < actual_clones)
    {
      core::list<ISceneNode*>::ConstIterator it = this->getChildren().end();
      (*it)->remove(); 
      --actual_clones;
    }
  }

  return true;
}


void ChIrrNode::OnAnimate(u32 timeMs)
{
  if (IsVisible && ChronoControlled) {

    // reorient/reposition the scene node every frame
    if (physicsitem) {
      if (!physicsitem->GetAssetsFrameNclones()) {
        ChIrrTools::alignIrrlichtNodeToChronoCsys(this, physicsitem->GetAssetsFrame().GetCoord() );
      } else {
        // check that children clones are already as many as 
        // assets frame clones, and adjust it if not:
        if (SetupClones()) {
          // make each clone node match the corresponding asset frame :
          unsigned int nclones = physicsitem->GetAssetsFrameNclones();
          unsigned int iclone = 0;
          core::list<ISceneNode*>::ConstIterator it = this->getChildren().begin();
          for (; it != Children.end(); ++it) {
            ChIrrTools::alignIrrlichtNodeToChronoCsys((*it), physicsitem->GetAssetsFrame(iclone).GetCoord() );
            ++iclone;
          }
        }
      }

    }

  }

  ISceneNode::OnAnimate(timeMs);
}


chrono::ChSharedPtr<chrono::ChPhysicsItem> ChIrrNode::GetPhysicsItem() 
{
  chrono::ChSharedPtr<chrono::ChPhysicsItem> shphysicsitem(this->physicsitem);
  shphysicsitem->AddRef(); // trick because shphysicsitem ctor took raw this->physicsitem that is not a new() 
  return shphysicsitem;
}


void ChIrrNode::UpdateAssetsProxies()
{
  _recurse_update_asset_proxies(this);
}


void ChIrrNode::_recurse_update_asset_proxies(ISceneNode* mnode)
{
  ISceneNodeList::ConstIterator it = mnode->getChildren().begin();

  for (; it != mnode->getChildren().end(); ++it) {
    //if (ChIrrNodeProxyToAsset* mproxy = dynamic_cast<ChIrrNodeProxyToAsset*>(*it))
    if ((*it)->getType() == (ESCENE_NODE_TYPE)ESNT_CHIRRNODEPROXYTOASSET) {
      ChIrrNodeProxyToAsset* mproxy = (ChIrrNodeProxyToAsset*)(*it);
      mproxy->Update();
    }

    if (!(*it)->getChildren().empty())
      _recurse_update_asset_proxies((*it));
  }
}


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


