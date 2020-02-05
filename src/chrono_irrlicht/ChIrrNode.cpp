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

#include "chrono/core/ChLog.h"
#include "chrono_irrlicht/ChIrrNode.h"
#include "chrono_irrlicht/ChIrrNodeProxyToAsset.h"

namespace chrono {
namespace irrlicht {

using namespace irr;

ChIrrNode::ChIrrNode(std::weak_ptr<ChPhysicsItem> mphysicsitem,
                     scene::ISceneNode* parent,
                     scene::ISceneManager* mgr,
                     s32 id)
    : ISceneNode(parent, mgr, id), physicsitem(mphysicsitem), ChronoControlled(true) {
#ifdef _DEBUG
    setDebugName("ChIrrNode");
#endif

    // set an unique identifier
    // static_item_identifier++;
    // GetPhysicsItem()->SetIdentifier(static_item_identifier);
}

void ChIrrNode::OnRegisterSceneNode() {
    if (IsVisible)
        SceneManager->registerNodeForRendering(this);

    ISceneNode::OnRegisterSceneNode();
}

scene::ISceneNode* ChIrrNode::clone(scene::ISceneNode* newParent, scene::ISceneManager* newManager) {
    // GetLog() << "Cloning!\n";
    if (!newParent)
        newParent = Parent;
    if (!newManager)
        newManager = SceneManager;

    ChIrrNode* nb = new ChIrrNode(this->physicsitem, newParent, newManager, this->ID);

    nb->cloneMembers(this, newManager);
    nb->Box = this->Box;
    nb->ChronoControlled = this->ChronoControlled;
    nb->physicsitem = this->physicsitem;

    if (newParent)
        nb->drop();
    return nb;
}

bool ChIrrNode::SetupClones() {
    unsigned int needed_clones = physicsitem.lock()->GetAssetsFrameNclones();

    if (needed_clones) {
        unsigned int actual_clones = this->getChildren().getSize();

        // At least one clone 'sample' must be present (created with
        // ChIrrAssetConverter::Update...); otherwise just bail out.
        if (actual_clones == 0)
            return false;

        // Compare actual number of children and number of clones:
        // - do nothing if equal
        // - clone the sample (the first node child of this node) if fewer children
        // - delete clones if more children
        if (needed_clones == actual_clones)
            return true;

        while (needed_clones > actual_clones) {
            irr::core::list<ISceneNode*>::ConstIterator it = this->getChildren().begin();
            (*it)->clone();
            ++actual_clones;
        }

        while (needed_clones < actual_clones) {
            irr::core::list<ISceneNode*>::ConstIterator it = this->getChildren().end();
            (*it)->remove();
            --actual_clones;
        }
    }

    return true;
}

void ChIrrNode::OnAnimate(u32 timeMs) {
    if (IsVisible && ChronoControlled) {
        // reorient/reposition the scene node every frame
        if (!physicsitem.expired()) {
            if (!physicsitem.lock()->GetAssetsFrameNclones()) {
                ChIrrTools::alignIrrlichtNodeToChronoCsys(this, physicsitem.lock()->GetAssetsFrame().GetCoord());
            } else {
                // check that children clones are already as many as
                // assets frame clones, and adjust it if not:
                if (SetupClones()) {
                    // make each clone node match the corresponding asset frame :
                    unsigned int nclones = physicsitem.lock()->GetAssetsFrameNclones();
                    unsigned int iclone = 0;
                    irr::core::list<ISceneNode*>::ConstIterator it = this->getChildren().begin();
                    for (; it != Children.end(); ++it) {
                        ChIrrTools::alignIrrlichtNodeToChronoCsys(
                            (*it), physicsitem.lock()->GetAssetsFrame(iclone).GetCoord());
                        ++iclone;
                    }
                }
            }
        }
    }

    ISceneNode::OnAnimate(timeMs);
}

void ChIrrNode::UpdateAssetsProxies() {
    _recurse_update_asset_proxies(this);
}

void ChIrrNode::_recurse_update_asset_proxies(ISceneNode* mnode) {
    scene::ISceneNodeList::ConstIterator it = mnode->getChildren().begin();

    for (; it != mnode->getChildren().end(); ++it) {
        // if (ChIrrNodeProxyToAsset* mproxy = dynamic_cast<ChIrrNodeProxyToAsset*>(*it))
        if ((*it)->getType() == (scene::ESCENE_NODE_TYPE)ESNT_CHIRRNODEPROXYTOASSET) {
            ChIrrNodeProxyToAsset* mproxy = (ChIrrNodeProxyToAsset*)(*it);
            mproxy->Update();
        }

        if (!(*it)->getChildren().empty())
            _recurse_update_asset_proxies((*it));
    }
}

}  // end namespace irrlicht
}  // end namespace chrono
