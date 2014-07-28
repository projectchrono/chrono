//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.


#ifndef CHIRRPARTICLESSCENENODE_H
#define CHIRRPARTICLESSCENENODE_H

#include <irrlicht.h>

#include "physics/ChSystem.h"
#include "physics/ChParticlesClones.h"

#include "unit_IRRLICHT/ChApiIrr.h"
#include "unit_IRRLICHT/ChIrrMeshTools.h"


#define ESNT_CHPARTICLES 1201


namespace irr {
namespace scene {


class ChApiIrr ChIrrParticlesSceneNode : public scene::ISceneNode
{
public:

  /// Build a scene node for the Irrlicht Engine. This scene node is also a
  /// rigid body for the Chrono::Engine multibody simulation. As soon as
  /// created, the wrapped ChParticlesClones is also added to the Chrono::Engine
  /// To delete a ChParticlesClonesScene node from an Irrlicht scene, use the 
  /// remove() function only! (it will also be removed from the Chrono::Engine)
  ChIrrParticlesSceneNode(
    chrono::ChSystem*     msystem,      ///< pointer to the Chrono::Engine physical simulation system
    IAnimatedMesh*        mesh,         ///< a sample 3D mesh for representing the shape of each particle
    core::vector3df       mmesh_scale,  ///< scale of the sample mesh
    ISceneNode*           parent,       ///< the parent node in Irrlicht hierarchy
    ISceneManager*        mgr,          ///< the Irrlicht scene manager
    s32                   id);          ///< the Irrlicht identifier

  /// Destructor.
  /// Note: as this Irrlicht node is destructed, it also automatically removes 
  /// the wrapped ChParticlesClones from the ChronoEngine.
  ~ChIrrParticlesSceneNode();


  //
  // OVERRIDE/IMPLEMENT BASE IRRLICHT METHODS
  //

  virtual void OnRegisterSceneNode();

  virtual void render();

  virtual const core::aabbox3d<f32>& getBoundingBox() const;

  virtual void setMaterialTexture(s32               textureLayer,
                                  video::ITexture*  texture);

  virtual u32 getMaterialCount();

  virtual video::SMaterial& getMaterial(u32 i);

  void OnAnimate(u32 timeMs);

  virtual IShadowVolumeSceneNode* addShadowVolumeSceneNode(
                                      const IMesh* shadowMesh = 0,
                                      s32          id = -1,
                                      bool         zfailmethod = true,
                                      f32          infinity = 10000.0f);

  virtual ESCENE_NODE_TYPE getType() const {return (ESCENE_NODE_TYPE)ESNT_CHPARTICLES;}


  //
  // CHRONO::ENGINE SPECIFIC
  //

  /// Returns reference to the shared pointer which references the rigid body
  /// wrapped by this scene node.
  chrono::ChSharedPtr<chrono::ChParticlesClones>& GetParticles() {return *particlep;}

  /// Returns true if the node is moved by Chrono::Engine simulation system.
  virtual bool IsChronoControlled() const {return ChronoControlled;}

  /// Set true if you want Chrono::Engine to include this body in simulation.
  virtual void SetChronoControlled(const bool &controlled) {ChronoControlled = controlled;}

  //IAnimatedMeshSceneNode* GetChildMesh() {return child_mesh;}

private:

  /// Check if the particle amount corresponds to the children meshes, and add
  /// other meshes if needed.
  void UpdateChildrenHierarchy();

  // Data
  core::aabbox3d<f32>     Box;

  IAnimatedMeshSceneNode* child_mesh;
  IAnimatedMesh*          sample_mesh;
  core::vector3df         mesh_scale;
  video::ITexture*        sample_texture;

  s32 Nchildren;

  // Chrono Engine specific data
  chrono::ChSharedPtr<chrono::ChParticlesClones>* particlep;
  bool ChronoControlled;

  static int particles_identifier;
};



/// Easy-to-use function which creates a ChIrrParticlesSceneNode and inserts it
/// into the Irrlicht scene.
ChApiIrr
ISceneNode* addChParticlesSceneNode(chrono::ChSystem* asystem,
                                    ISceneManager*    amanager,
                                    IAnimatedMesh*    amesh,
                                    core::vector3df   amesh_scale,
                                    double            mmass = 1.0,
                                    ISceneNode*       aparent = 0,
                                    s32               mid = -1);


/// Easy-to-use function which creates a ChIrrParticlesSceneNode representing a
/// cluster of particles, ready to use for collisions (otherwise you could use
/// addChBodySceneNode() and add collision geometry by hand, but the following
/// is easier). The returned object has collision detection turned ON by default.
ChApiIrr
ISceneNode* addChParticlesSceneNode_easySpheres(chrono::ChSystem* asystem,
                                                ISceneManager*    amanager,
                                                double            mmass = 1.0,
                                                double            mradius = 1.0,
                                                int               Hslices = 12,
                                                int               Vslices = 6,
                                                ISceneNode*       aparent = 0,
                                                s32               mid = -1);


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif

