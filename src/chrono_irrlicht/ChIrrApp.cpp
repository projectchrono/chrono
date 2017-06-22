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

#include "chrono_irrlicht/ChIrrApp.h"

namespace chrono {
namespace irrlicht {

using namespace irr;

ChIrrApp::ChIrrApp(ChSystem* psystem,
                   const wchar_t* title,
                   core::dimension2d<u32> dimens,
                   bool do_fullscreen,
                   bool do_shadows,
                   bool do_antialias,
                   video::E_DRIVER_TYPE mydriver)
    : ChIrrAppInterface(psystem, title, dimens, do_fullscreen, do_shadows, do_antialias, mydriver) {
    mconverter = new ChIrrAssetConverter(*this);
}

ChIrrApp::~ChIrrApp() {
    if (mconverter)
        delete mconverter;
}

void ChIrrApp::AssetBind(std::shared_ptr<ChPhysicsItem> mitem) {
    GetAssetConverter()->Bind(mitem);
}

void ChIrrApp::AssetBindAll() {
    GetAssetConverter()->BindAll();
}

void ChIrrApp::AssetUpdate(std::shared_ptr<ChPhysicsItem> mitem) {
    GetAssetConverter()->Update(mitem);
}

void ChIrrApp::AssetUpdateAll() {
    GetAssetConverter()->UpdateAll();
}

void ChIrrApp::AddShadow(std::shared_ptr<ChPhysicsItem> mitem) {
    std::shared_ptr<ChIrrNodeAsset> myirrasset;
    myirrasset = GetAssetConverter()->GetIrrNodeAsset(mitem);
    if (myirrasset) {
        _recurse_add_shadow(myirrasset->GetIrrlichtNode());
    }
}

void ChIrrApp::AddShadowAll() {
    ChSystem::IteratorPhysicsItems miter = this->GetSystem()->IterBeginPhysicsItems();
    while (miter.HasItem()) {
        AddShadow(*miter);
        ++miter;
    }
}

void ChIrrApp::_recurse_add_shadow(scene::ISceneNode* mnode) {
    scene::ISceneNodeList::ConstIterator it = mnode->getChildren().begin();
    for (; it != mnode->getChildren().end(); ++it) {
        _recurse_add_shadow((*it));
    }
    // add shadow only to leaves
    if (mnode->getChildren().getSize() == 0)
        GetEffects()->addShadowToNode(mnode);
}

}  // end namespace irrlicht
}  // end namespace chrono
