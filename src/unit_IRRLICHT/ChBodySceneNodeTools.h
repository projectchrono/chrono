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

#ifndef CHBODYSCENENODETOOLS_H
#define CHBODYSCENENODETOOLS_H


// =============================================================================
//  ***OBSOLETE*** 
//  better: use the ChIrrNodeAsset approach
//
//  Some functions to allow easy creation of ChBodySceneNode objects
// =============================================================================

#include <string>
#include <irrlicht.h>

#include "ChBodySceneNode.h"
#include "ChIrrMeshTools.h"

#include "unit_IRRLICHT/ChApiIrr.h"


namespace irr {
namespace scene {


/// Set the directory where the cube, cylinder, etc. primitives are stored as
/// .obj  files (by default, it is "../data/"). Must be set _before_ creating
/// the ChIrrApp or the ChIrrAssetConverter.
ChApiIrr
void SetDefaultObjectDir(const std::string& mdir);


/// Easy-to-use function which creates a ChBodySceneNode with given position of
/// COG, inserts it into the Irrlicht scene etc.
ChApiIrr
ISceneNode* addChBodySceneNode(
                  chrono::ChSystem* asystem,
                  ISceneManager* amanager,
                  IAnimatedMesh* amesh,
                  double mmass = 1.0, 
                  const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
                  const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
                  ISceneNode* aparent=0, 
                  s32 mid=-1);


/// Same as above, but allow user to specify an offset distance between COG and
/// mesh location.  Note: the mesh remains in the same location, but the COG of
/// the rigid body changes by the offset amount.
ChApiIrr
ISceneNode* addChBodySceneNode_offsetCOG(
                  chrono::ChSystem* asystem,
                  ISceneManager* amanager,
                  IAnimatedMesh* amesh, 
                  double mmass = 1.0, 
                  const chrono::ChVector<>& mesh_position = chrono::ChVector<>(0,0,0),
                  const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
                  const chrono::ChVector<>& COG_offset = chrono::ChVector<>(0,0,0),
                  ISceneNode* aparent=0, 
                  s32 mid=-1);


/// Easy-to-use function which creates a ChBodySceneNode representing a sphere,
/// ready to use for collisions (otherwise you could use addChBodySceneNode()
/// and add collision geometry by hand, but the following is easier).
/// The returned object has collision detection turned ON by default.
ChApiIrr
ISceneNode* addChBodySceneNode_easySphere(
                  chrono::ChSystem* asystem,
                  ISceneManager* amanager,
                  double mmass = 1.0, 
                  const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
                  double mradius= 1.0,
                  int Hslices = 15,
                  int Vslices = 8,
                  ISceneNode* aparent=0, 
                  s32 mid=-1);


/// Easy-to-use function which creates a ChBodySceneNode representing a box,
/// ready to use for collisions (otherwise you could use addChBodySceneNode()
/// and add collision geometry by hand, but the following is easier).
/// The returned object has collision detection turned ON by default.
ChApiIrr
ISceneNode* addChBodySceneNode_easyBox(
                  chrono::ChSystem* asystem,
                  ISceneManager* amanager,
                  double mmass = 1.0, 
                  const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
                  const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
                  const chrono::ChVector<>& size = chrono::ChVector<>(1,1,1),
                  ISceneNode* aparent=0, 
                  s32 mid=-1);


/// Easy-to-use function which creates a ChBodySceneNode representing a
/// cylinder, ready to use for collisions (otherwise you could use
/// addChBodySceneNode() and add collision geometry by hand, but the following
/// is easier).
/// The returned object has collision detection turned ON by default.
ChApiIrr
ISceneNode* addChBodySceneNode_easyCylinder(
                  chrono::ChSystem* asystem,
                  ISceneManager* amanager,
                  double mmass = 1.0, 
                  const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
                  const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
                  const chrono::ChVector<>& size = chrono::ChVector<>(1,1,1),
                  ISceneNode* aparent=0, 
                  s32 mid=-1);


/// Easy-to-use function which creates a ChBodySceneNode representing a barrel,
/// ready to use for collisions (otherwise you could use addChBodySceneNode()
/// and add collision geometry by hand, but the following is easier).
/// The returned object has collision detection turned ON by default.
ChApiIrr
ISceneNode* addChBodySceneNode_easyBarrel(
                  chrono::ChSystem* asystem,
                  ISceneManager* amanager,
                  double mmass = 1.0, 
                  const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
                  double mradiusH= 1.0,
                  double mradiusV= 1.0,
                  double mYlow = -0.5,
                  double mYhigh=  0.8,
                  double mOffset = 0.0,
                  int Hslices = 15,
                  int Vslices = 10,
                  ISceneNode* aparent=0, 
                  s32 mid=-1);


/// Easy-to-use function that creates a ChBodySceneNode representing a clone of
/// another ChBodySceneNode, but at a different position and rotation.
/// NOTE! the collision shapes (if any) in the source object will be _shared_
/// with the cloned object, so if you have to create for exampe 1000 identical
/// objects (ex. cubes) this function will allow you to replicate 999 times a
/// single cube that you once created calling addChBodySceneNode_easyBox():
/// this will be more memory-efficient than calling many times the function
/// addChBodySceneNode_easyBox(). Shared geometries will be deallocated
/// automatically thank to shared pointers.
ChApiIrr
ISceneNode* addChBodySceneNode_easyClone(
                  chrono::ChSystem* asystem,
                  ISceneManager* amanager,
                  ChBodySceneNode* source, 
                  const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
                  const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
                  ISceneNode* aparent=0, 
                  s32 mid=-1);


/// Easy-to-use function which creates a ChBodySceneNode representing a GENERIC
/// mesh of arbitrary shape, loaded from file using the Irrlicht formats
/// (.obj, .3ds, .X, etc) ready to use for collisions. The loaded mesh is used
/// BOTH for display in Irrlicht 3d view and for collision. The returned object
/// has collision detection turned ON by default.
ChApiIrr
ISceneNode* addChBodySceneNode_easyGenericMesh(
                  chrono::ChSystem* asystem,
                  ISceneManager* amanager,
                  double mmass = 1.0, 
                  const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
                  const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
                  const char* mesh_filemane =0,
                  bool  is_static = true,
                  bool  is_convex = true,
                  ISceneNode* aparent=0, 
                  s32 mid=-1);


/// Easy-to-use function which creates a ChBodySceneNode representing a STATIC
/// shape, as a mesh loaded from file using the Irrlicht formats (.obj, .3ds,
/// .X, etc). The mesh can be either convex or concave: it works anyway (but it
/// should NOT move!) The loaded mesh is used BOTH for display in Irrlicht 3d
/// view and for collision. The returned object has collision detection turned
/// ON by default. The returned object is fixed by default (no need to do
/// SetBodyFixed(true) ).
ChApiIrr
ISceneNode* addChBodySceneNode_easyStaticMesh(
                  chrono::ChSystem* asystem,
                  ISceneManager* amanager, 
                  const char* mesh_filename,
                  const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
                  const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
                  ISceneNode* aparent=0, 
                  s32 mid=-1);


/// Easy-to-use function which creates a ChBodySceneNode representing a CONVEX
/// shape, as a mesh loaded from file using the Irrlicht formats (.obj, .3ds,
/// .X, etc).  The mesh can be moving. If mesh is not concave or has holes/gaps,
/// its convex hull is used anyway. Best used with _simple_ meshes, not too many
/// points. The loaded mesh is used BOTH for display in Irrlicht 3d view and for
/// collision. The returned object has collision detection turned ON by default.
ChApiIrr
ISceneNode* addChBodySceneNode_easyConvexMesh(
                  chrono::ChSystem* asystem,
                  ISceneManager* amanager, 
                  const char* mesh_filename,
                  double mmass,
                  const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
                  const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
                  ISceneNode* aparent=0, 
                  s32 mid=-1);


/// Easy-to-use function which creates a ChBodySceneNode representing a CONCAVE
/// shape, as a mesh loaded from file using the Irrlicht formats (.obj, .3ds,
/// .X, etc). The mesh can be moving. The loaded mesh is used BOTH for display
/// in Irrlicht 3d view and for collision. The returned object has collision
/// detection turned ON by default. Works also for convex shapes, but the
/// performance is not as fast and robust as in case of native convex cases,
/// so whenever possible use addChBodySceneNode_easyConvexMesh.
ChApiIrr
ISceneNode* addChBodySceneNode_easyConcaveMesh(
                  chrono::ChSystem* asystem,
                  ISceneManager* amanager, 
                  const char* mesh_filename,
                  double mmass,
                  const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
                  const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
                  ISceneNode* aparent=0, 
                  s32 mid=-1);


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif // END of ChBodySceneNode.h

