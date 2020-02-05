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

#ifndef CHIRRPARTICLESSCENENODE_H
#define CHIRRPARTICLESSCENENODE_H

#include <irrlicht.h>

#include "chrono/physics/ChParticlesClones.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_irrlicht/ChApiIrr.h"
#include "chrono_irrlicht/ChIrrMeshTools.h"

#define ESNT_CHPARTICLES 1201

namespace chrono {
namespace irrlicht {

/// @addtogroup irrlicht_module
/// @{

/// Definition of an Irrlicht scene node for particles.
class ChApiIrr ChIrrParticlesSceneNode : public irr::scene::ISceneNode {
  public:
    /// Build a scene node for the Irrlicht Engine. This scene node is also a
    /// rigid body for the Chrono multibody simulation. As soon as
    /// created, the wrapped ChParticlesClones is also added to the Chrono
    /// To delete a ChParticlesClonesScene node from an Irrlicht scene, use the
    /// remove() function only! (it will also be removed from the Chrono)
    ChIrrParticlesSceneNode(
        ChSystem* msystem,                 ///< pointer to the Chrono physical simulation system
        irr::scene::IAnimatedMesh* mesh,   ///< a sample 3D mesh for representing the shape of each particle
        irr::core::vector3df mmesh_scale,  ///< scale of the sample mesh
        ISceneNode* parent,                ///< the parent node in Irrlicht hierarchy
        irr::scene::ISceneManager* mgr,    ///< the Irrlicht scene manager
        irr::s32 id                        ///< the Irrlicht identifier
        );

    /// Destructor.
    /// Note: as this Irrlicht node is destructed, it also automatically removes
    /// the wrapped ChParticlesClones from the ChronoEngine.
    ~ChIrrParticlesSceneNode();

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

    virtual irr::scene::ESCENE_NODE_TYPE getType() const { return (irr::scene::ESCENE_NODE_TYPE)ESNT_CHPARTICLES; }

    //
    // CHRONO::ENGINE SPECIFIC
    //

    /// Returns reference to the shared pointer which references the rigid body
    /// wrapped by this scene node.
    std::shared_ptr<ChParticlesClones>& GetParticles() { return *particlep; }

    /// Returns true if the node is moved by Chrono simulation system.
    virtual bool IsChronoControlled() const { return ChronoControlled; }

    /// Set true if you want Chrono to include this body in simulation.
    virtual void SetChronoControlled(const bool& controlled) { ChronoControlled = controlled; }

    // IAnimatedMeshSceneNode* GetChildMesh() {return child_mesh;}

  private:
    /// Check if the particle amount corresponds to the children meshes, and add
    /// other meshes if needed.
    void UpdateChildrenHierarchy();

    // Data
    irr::core::aabbox3d<irr::f32> Box;

    irr::scene::IAnimatedMeshSceneNode* child_mesh;
    irr::scene::IAnimatedMesh* sample_mesh;
    irr::core::vector3df mesh_scale;
    irr::video::ITexture* sample_texture;

    irr::s32 Nchildren;

    // Chrono Engine specific data
    std::shared_ptr<ChParticlesClones>* particlep;
    bool ChronoControlled;

    static int particles_identifier;
};

/// Easy-to-use function which creates a ChIrrParticlesSceneNode and inserts it
/// into the Irrlicht scene.
ChApiIrr irr::scene::ISceneNode* addChParticlesSceneNode(ChSystem* asystem,
                                                         irr::scene::ISceneManager* amanager,
                                                         irr::scene::IAnimatedMesh* amesh,
                                                         irr::core::vector3df amesh_scale,
                                                         double mmass = 1.0,
                                                         irr::scene::ISceneNode* aparent = 0,
                                                         irr::s32 mid = -1);

/// Easy-to-use function which creates a ChIrrParticlesSceneNode representing a
/// cluster of particles, ready to use for collisions (otherwise you could use
/// addChBodySceneNode() and add collision geometry by hand, but the following
/// is easier). The returned object has collision detection turned ON by default.
ChApiIrr irr::scene::ISceneNode* addChParticlesSceneNode_easySpheres(ChSystem* asystem,
                                                                     irr::scene::ISceneManager* amanager,
                                                                     double mmass = 1.0,
                                                                     double mradius = 1.0,
                                                                     int Hslices = 12,
                                                                     int Vslices = 6,
                                                                     irr::scene::ISceneNode* aparent = 0,
                                                                     irr::s32 mid = -1);

/// @} irrlicht_module

}  // end namespace irrlicht
}  // end namespace chrono

#endif
