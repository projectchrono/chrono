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

#ifndef CHIRRNODEASSET_H
#define CHIRRNODEASSET_H

#include <irrlicht.h>

#include "chrono/assets/ChAsset.h"

#include "chrono_irrlicht/ChApiIrr.h"
#include "chrono_irrlicht/ChIrrAppInterface.h"
#include "chrono_irrlicht/ChIrrNode.h"

namespace chrono {
namespace irrlicht {

/// @addtogroup irrlicht_module
/// @{

/// Class for adding Irrlicht visualization to a ChPhysicsItem.
/// This must be added as an 'asset' to the item (ex., a ChBody).
class ChApiIrr ChIrrNodeAsset : public ChAsset {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChIrrNodeAsset)

  protected:
    ChIrrNode* mnode;

  public:
    ChIrrNodeAsset() : mnode(0) {}

    virtual ~ChIrrNodeAsset() {
        // remove irrlicht node from scene manager
        this->UnBind();
    }

    irr::scene::ISceneNode* GetIrrlichtNode() { return mnode; }

    void Bind(std::shared_ptr<ChPhysicsItem> aitem, ChIrrAppInterface& aapp) {
        UnBind();

        mnode = new ChIrrNode(aitem, aapp.GetContainer(), aapp.GetSceneManager(), 0);

        // Grab() to avoid dangling pointer if irrlicht scene is deleted before this obj:
        //  ***NOT NEEDED! Already done in ctor OF ISceneNode when attaching to parent
        // mnode->grab();
    }

    void UnBind() {
        if (mnode) {
            mnode->remove();
            mnode->drop();
            mnode = 0;
        }
    }

    virtual void Update(ChPhysicsItem* updater, const ChCoordsys<>& coords) {
        // inherit parent
        ChAsset::Update(updater, coords);

        mnode->UpdateAssetsProxies();
    }
};

/// @} irrlicht_module

}  // end namespace irrlicht
}  // end namespace chrono

#endif
