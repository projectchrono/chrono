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

#ifndef CH_IRR_NODE_MODEL_H
#define CH_IRR_NODE_MODEL_H

#include <irrlicht.h>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChPhysicsItem.h"

#include "chrono_irrlicht/ChApiIrr.h"
#include "chrono_irrlicht/ChIrrNodeShape.h"
#include "chrono_irrlicht/ChIrrTools.h"

#define ESNT_CHIRRNODE_MODEL 1201

namespace chrono {
namespace irrlicht {

/// @addtogroup irrlicht_module
/// @{

/// Irrlicht scene node associated with the visual model of a physics item.
/// Such a node contains children nodes for each visual shape in the physics item's visual model.
class ChApiIrr ChIrrNodeModel : public irr::scene::ISceneNode {
  public:
    /// Construct an Irrlicht scene node for the visual model associated with the specified physics item.
    /// These nodes are all children of the top-level Irrlicht container node and they are each populate
    /// with children nodes associated to the various visual shapes in the model.
    /// A mapping between physics items and the Irrlicht scene nodes is managed by the Irrlicht visualization
    /// system object.
    ChIrrNodeModel(std::weak_ptr<ChPhysicsItem> physicsitem,  ///< pointer to the associated physics item
                   irr::scene::ISceneNode* parent,            ///< parent node in Irrlicht hierarchy
                   irr::scene::ISceneManager* mgr,            ///< Irrlicht scene manager
                   irr::s32 id                                ///< Irrlicht identifier
    );

    ~ChIrrNodeModel() {}

    /// Get the physics item associated with this scene node.
    std::weak_ptr<ChPhysicsItem> GetPhysicsItem() { return m_physicsitem; }

    /// Update the chidlren Irrlicht nodes associated with individual visual shapes.
    void UpdateChildren();

    /// Setup use of clones for visual models that use multiple instances of the same visual shape.
    bool SetupClones();

  private:
    void UpdateChildren_recursive(irr::scene::ISceneNode* node);

    virtual irr::scene::ESCENE_NODE_TYPE getType() const override;
    virtual void OnRegisterSceneNode() override;
    virtual void render() override {}
    virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const override { return m_box; }
    virtual ISceneNode* clone(ISceneNode* new_parent, irr::scene::ISceneManager* new_manager) override;
    virtual void OnAnimate(irr::u32 timeMs) override;

    irr::core::aabbox3d<irr::f32> m_box;
    std::weak_ptr<ChPhysicsItem> m_physicsitem;
};

/// @} irrlicht_module

}  // end namespace irrlicht
}  // end namespace chrono

#endif
