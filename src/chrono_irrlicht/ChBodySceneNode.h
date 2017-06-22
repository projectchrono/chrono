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

#ifndef CHBODYSCENENODE_H
#define CHBODYSCENENODE_H

// =============================================================================
//  ***OBSOLETE***
//  better: use the ChIrrNodeAsset approach
// =============================================================================

#include <irrlicht.h>

#include "chrono/physics/ChSystem.h"
#include "chrono_irrlicht/ChApiIrr.h"

#define ESNT_CHBODY 1200

namespace chrono {
namespace irrlicht {

/// @addtogroup irrlicht_module
/// @{

/// Irrlicht scene node for a body.
class ChApiIrr ChBodySceneNode : public irr::scene::ISceneNode {
  public:
    /// Build a scene node for the Irrlicht Engine. This scene node is also a
    /// rigid body for the Chrono multibody simulation.
    /// As soon as created, the wrapped ChBody is also added to Chrono system.
    /// To delete a ChBodyScene node from an Irrlicht scene, use the remove()
    /// function only! (it will also be removed from the Chrono system)
    ChBodySceneNode(ChSystem* msystem,                ///< pointer to the Chrono physical simulation system
                    irr::scene::IAnimatedMesh* mesh,  ///< a 3D mesh for representing the shape of the body
                    irr::scene::ISceneNode* parent,   ///< the parent node in Irrlicht hierarchy
                    irr::scene::ISceneManager* mgr,   ///< the Irrlicht scene manager
                    irr::s32 id                       ///< the Irrlicht identifier
                    );

    ChBodySceneNode(ChSystem* msystem,                ///< pointer to the Chrono physical simulation system
                    irr::scene::IAnimatedMesh* mesh,  ///< a 3D mesh for representing the shape of the body
                    irr::scene::ISceneNode* parent,   ///< the parent node in Irrlicht hierarchy
                    irr::scene::ISceneManager* mgr,   ///< the Irrlicht 	 manager
                    irr::s32 id,                      ///< the Irrlicht identifier
                    const ChVector<>& offset          ///< offset between mesh and body COG
                    );

    /// Destructor.
    /// Note: as this Irrlicht node is destructed, it also automatically removes
    /// the wrapped ChBody from the Chrono.
    ~ChBodySceneNode();

    //
    // OVERRIDE/IMPLEMENT BASE IRRLICHT METHODS
    //

    virtual void OnRegisterSceneNode();

    virtual void render();

    virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const;

    virtual void setMaterialTexture(irr::s32 textureLayer, irr::video::ITexture* texture);

    virtual irr::u32 getMaterialCount();

    virtual irr::video::SMaterial& getMaterial(irr::u32 i);

    void OnAnimate(irr::u32 timeMs);

    virtual irr::scene::IShadowVolumeSceneNode* addShadowVolumeSceneNode(const irr::scene::IMesh* shadowMesh = 0,
                                                                         irr::s32 id = -1,
                                                                         bool zfailmethod = true,
                                                                         irr::f32 infinity = 10000.0f);

    //
    // CHRONO::ENGINE SPECIFIC
    //

    /// Returns reference to the shared pointer which references the
    /// rigid body wrapped by this scene node.
    std::shared_ptr<ChBody>& GetBody() { return *bodyp; }

    /// Returns true if the node is moved by Chrono simulation system.
    virtual bool IsChronoControlled() const { return ChronoControlled; }

    /// Set true if you want Chrono to include this body in simulation.
    virtual void SetChronoControlled(const bool& controlled) { ChronoControlled = controlled; }

    irr::scene::IAnimatedMeshSceneNode* GetChildMesh() { return child_mesh; }

    virtual irr::scene::ESCENE_NODE_TYPE getType() const { return (irr::scene::ESCENE_NODE_TYPE)ESNT_CHBODY; }

  private:
    irr::core::aabbox3d<irr::f32> Box;
    irr::scene::IAnimatedMeshSceneNode* child_mesh;

    // Chrono Engine specific data
    std::shared_ptr<ChBody>* bodyp;
    bool ChronoControlled;

    static int body_identifier;
};

/// @} irrlicht_module

}  // end namespace irrlicht
}  // end namespace chrono

#endif
