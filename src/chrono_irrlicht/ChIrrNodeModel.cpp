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
#include "chrono_irrlicht/ChIrrNodeModel.h"
#include "chrono_irrlicht/ChIrrNodeShape.h"

namespace chrono {
namespace irrlicht {

using namespace irr;

ChIrrNodeModel::ChIrrNodeModel(std::weak_ptr<ChPhysicsItem> physicsitem,
                               scene::ISceneNode* parent,
                               scene::ISceneManager* mgr,
                               s32 id)
    : ISceneNode(parent, mgr, id), m_physicsitem(physicsitem) {
#ifdef _DEBUG
    setDebugName("ChIrrNodeModel");
#endif

    // set an unique identifier
    // static_item_identifier++;
    // GetPhysicsItem()->SetIdentifier(static_item_identifier);
}

void ChIrrNodeModel::OnRegisterSceneNode() {
    if (IsVisible)
        SceneManager->registerNodeForRendering(this);

    ISceneNode::OnRegisterSceneNode();
}

scene::ISceneNode* ChIrrNodeModel::clone(scene::ISceneNode* new_parent, scene::ISceneManager* new_manager) {
    // GetLog() << "Cloning!\n";
    if (!new_parent)
        new_parent = Parent;
    if (!new_manager)
        new_manager = SceneManager;

    ChIrrNodeModel* nb = new ChIrrNodeModel(m_physicsitem, new_parent, new_manager, this->ID);

    nb->cloneMembers(this, new_manager);
    nb->m_box = m_box;
    nb->m_physicsitem = m_physicsitem;

    if (new_parent)
        nb->drop();
    return nb;
}

scene::ESCENE_NODE_TYPE ChIrrNodeModel::getType() const {
    return (scene::ESCENE_NODE_TYPE)ESNT_CHIRRNODE_MODEL;
}

bool ChIrrNodeModel::SetupClones() {
    unsigned int needed_clones = m_physicsitem.lock()->GetNumVisualModelClones();

    if (needed_clones) {
        unsigned int actual_clones = this->getChildren().getSize();

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

void ChIrrNodeModel::OnAnimate(u32 timeMs) {
    if (m_physicsitem.expired())
        return;

    if (IsVisible) {
        // reorient/reposition the scene node
        if (!m_physicsitem.lock()->GetNumVisualModelClones()) {
            tools::alignIrrlichtNode(this, m_physicsitem.lock()->GetVisualModelFrame().GetCoord());
        } else {
            // check that children clones are already as many as
            // assets frame clones, and adjust it if not:
            if (SetupClones()) {
                // make each clone node match the corresponding asset frame :
                unsigned int iclone = 0;
                irr::core::list<ISceneNode*>::ConstIterator it = this->getChildren().begin();
                for (; it != Children.end(); ++it) {
                    tools::alignIrrlichtNode((*it), m_physicsitem.lock()->GetVisualModelFrame(iclone).GetCoord());
                    ++iclone;
                }
            }
        }
    }

    ISceneNode::OnAnimate(timeMs);
}

void ChIrrNodeModel::UpdateChildren() {
    UpdateChildren_recursive(this);
}

void ChIrrNodeModel::UpdateChildren_recursive(ISceneNode* node) {
    scene::ISceneNodeList::ConstIterator it = node->getChildren().begin();

    for (; it != node->getChildren().end(); ++it) {
        // if (ChIrrNodeShape* mproxy = dynamic_cast<ChIrrNodeShape*>(*it))
        if ((*it)->getType() == (scene::ESCENE_NODE_TYPE)ESNT_CHIRRNODE_SHAPE) {
            ChIrrNodeShape* mproxy = (ChIrrNodeShape*)(*it);
            mproxy->Update();
        }

        if (!(*it)->getChildren().empty())
            UpdateChildren_recursive((*it));
    }
}

}  // end namespace irrlicht
}  // end namespace chrono
