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

#include "chrono_irrlicht/ChBodySceneNode.h"
#include "chrono_irrlicht/ChIrrMeshTools.h"

#include "chrono_irrlicht/ChApiIrr.h"

namespace chrono {
namespace irrlicht {

/// @addtogroup irrlicht_module
/// @{

/// Easy-to-use function which creates a ChBodySceneNode with given position of
/// COG, inserts it into the Irrlicht scene etc.
ChApiIrr irr::scene::ISceneNode* addChBodySceneNode(ChSystem* asystem,
                                                    irr::scene::ISceneManager* amanager,
                                                    irr::scene::IAnimatedMesh* amesh,
                                                    double mmass = 1.0,
                                                    const ChVector<>& position = ChVector<>(0, 0, 0),
                                                    const ChQuaternion<>& rotation = ChQuaternion<>(1, 0, 0, 0),
                                                    irr::scene::ISceneNode* aparent = 0,
                                                    irr::s32 mid = -1);

/// Same as above, but allow user to specify an offset distance between COG and
/// mesh location.  Note: the mesh remains in the same location, but the COG of
/// the rigid body changes by the offset amount.
ChApiIrr irr::scene::ISceneNode* addChBodySceneNode_offsetCOG(
    ChSystem* asystem,
    irr::scene::ISceneManager* amanager,
    irr::scene::IAnimatedMesh* amesh,
    double mmass = 1.0,
    const ChVector<>& mesh_position = ChVector<>(0, 0, 0),
    const ChQuaternion<>& rotation = ChQuaternion<>(1, 0, 0, 0),
    const ChVector<>& COG_offset = ChVector<>(0, 0, 0),
    irr::scene::ISceneNode* aparent = 0,
    irr::s32 mid = -1);

/// Easy-to-use function which creates a ChBodySceneNode representing a sphere,
/// ready to use for collisions (otherwise you could use addChBodySceneNode()
/// and add collision geometry by hand, but the following is easier).
/// The returned object has collision detection turned ON by default.
ChApiIrr irr::scene::ISceneNode* addChBodySceneNode_easySphere(ChSystem* asystem,
                                                               irr::scene::ISceneManager* amanager,
                                                               double mmass = 1.0,
                                                               const ChVector<>& position = ChVector<>(0, 0, 0),
                                                               double mradius = 1.0,
                                                               int Hslices = 15,
                                                               int Vslices = 8,
                                                               irr::scene::ISceneNode* aparent = 0,
                                                               irr::s32 mid = -1);

/// Easy-to-use function which creates a ChBodySceneNode representing a box,
/// ready to use for collisions (otherwise you could use addChBodySceneNode()
/// and add collision geometry by hand, but the following is easier).
/// The returned object has collision detection turned ON by default.
ChApiIrr irr::scene::ISceneNode* addChBodySceneNode_easyBox(ChSystem* asystem,
                                                            irr::scene::ISceneManager* amanager,
                                                            double mmass = 1.0,
                                                            const ChVector<>& position = ChVector<>(0, 0, 0),
                                                            const ChQuaternion<>& rotation = ChQuaternion<>(1, 0, 0, 0),
                                                            const ChVector<>& size = ChVector<>(1, 1, 1),
                                                            irr::scene::ISceneNode* aparent = 0,
                                                            irr::s32 mid = -1);

/// Easy-to-use function which creates a ChBodySceneNode representing a
/// cylinder, ready to use for collisions (otherwise you could use
/// addChBodySceneNode() and add collision geometry by hand, but the following
/// is easier).
/// The returned object has collision detection turned ON by default.
ChApiIrr irr::scene::ISceneNode* addChBodySceneNode_easyCylinder(
    ChSystem* asystem,
    irr::scene::ISceneManager* amanager,
    double mmass = 1.0,
    const ChVector<>& position = ChVector<>(0, 0, 0),
    const ChQuaternion<>& rotation = ChQuaternion<>(1, 0, 0, 0),
    const ChVector<>& size = ChVector<>(1, 1, 1),
    irr::scene::ISceneNode* aparent = 0,
    irr::s32 mid = -1);

/// Easy-to-use function which creates a ChBodySceneNode representing a barrel,
/// ready to use for collisions (otherwise you could use addChBodySceneNode()
/// and add collision geometry by hand, but the following is easier).
/// The returned object has collision detection turned ON by default.
ChApiIrr irr::scene::ISceneNode* addChBodySceneNode_easyBarrel(ChSystem* asystem,
                                                               irr::scene::ISceneManager* amanager,
                                                               double mmass = 1.0,
                                                               const ChVector<>& position = ChVector<>(0, 0, 0),
                                                               double mradiusH = 1.0,
                                                               double mradiusV = 1.0,
                                                               double mYlow = -0.5,
                                                               double mYhigh = 0.8,
                                                               double mOffset = 0.0,
                                                               int Hslices = 15,
                                                               int Vslices = 10,
                                                               irr::scene::ISceneNode* aparent = 0,
                                                               irr::s32 mid = -1);

/// Easy-to-use function that creates a ChBodySceneNode representing a clone of
/// another ChBodySceneNode, but at a different position and rotation.
/// NOTE! the collision shapes (if any) in the source object will be _shared_
/// with the cloned object, so if you have to create for exampe 1000 identical
/// objects (ex. cubes) this function will allow you to replicate 999 times a
/// single cube that you once created calling addChBodySceneNode_easyBox():
/// this will be more memory-efficient than calling many times the function
/// addChBodySceneNode_easyBox(). Shared geometries will be deallocated
/// automatically thank to shared pointers.
ChApiIrr irr::scene::ISceneNode* addChBodySceneNode_easyClone(
    ChSystem* asystem,
    irr::scene::ISceneManager* amanager,
    ChBodySceneNode* source,
    const ChVector<>& position = ChVector<>(0, 0, 0),
    const ChQuaternion<>& rotation = ChQuaternion<>(1, 0, 0, 0),
    irr::scene::ISceneNode* aparent = 0,
    irr::s32 mid = -1);

/// Easy-to-use function which creates a ChBodySceneNode representing a GENERIC
/// mesh of arbitrary shape, loaded from file using the Irrlicht formats
/// (.obj, .3ds, .X, etc) ready to use for collisions. The loaded mesh is used
/// BOTH for display in Irrlicht 3d view and for collision. The returned object
/// has collision detection turned ON by default.
ChApiIrr irr::scene::ISceneNode* addChBodySceneNode_easyGenericMesh(
    ChSystem* asystem,
    irr::scene::ISceneManager* amanager,
    double mmass = 1.0,
    const ChVector<>& position = ChVector<>(0, 0, 0),
    const ChQuaternion<>& rotation = ChQuaternion<>(1, 0, 0, 0),
    const char* mesh_filemane = 0,
    bool is_static = true,
    bool is_convex = true,
    irr::scene::ISceneNode* aparent = 0,
    irr::s32 mid = -1);

/// Easy-to-use function which creates a ChBodySceneNode representing a STATIC
/// shape, as a mesh loaded from file using the Irrlicht formats (.obj, .3ds,
/// .X, etc). The mesh can be either convex or concave: it works anyway (but it
/// should NOT move!) The loaded mesh is used BOTH for display in Irrlicht 3d
/// view and for collision. The returned object has collision detection turned
/// ON by default. The returned object is fixed by default (no need to do
/// SetBodyFixed(true) ).
ChApiIrr irr::scene::ISceneNode* addChBodySceneNode_easyStaticMesh(
    ChSystem* asystem,
    irr::scene::ISceneManager* amanager,
    const char* mesh_filename,
    const ChVector<>& position = ChVector<>(0, 0, 0),
    const ChQuaternion<>& rotation = ChQuaternion<>(1, 0, 0, 0),
    irr::scene::ISceneNode* aparent = 0,
    irr::s32 mid = -1);

/// Easy-to-use function which creates a ChBodySceneNode representing a CONVEX
/// shape, as a mesh loaded from file using the Irrlicht formats (.obj, .3ds,
/// .X, etc).  The mesh can be moving. If mesh is not concave or has holes/gaps,
/// its convex hull is used anyway. Best used with _simple_ meshes, not too many
/// points. The loaded mesh is used BOTH for display in Irrlicht 3d view and for
/// collision. The returned object has collision detection turned ON by default.
ChApiIrr irr::scene::ISceneNode* addChBodySceneNode_easyConvexMesh(
    ChSystem* asystem,
    irr::scene::ISceneManager* amanager,
    const char* mesh_filename,
    double mmass,
    const ChVector<>& position = ChVector<>(0, 0, 0),
    const ChQuaternion<>& rotation = ChQuaternion<>(1, 0, 0, 0),
    irr::scene::ISceneNode* aparent = 0,
    irr::s32 mid = -1);

/// Easy-to-use function which creates a ChBodySceneNode representing a CONCAVE
/// shape, as a mesh loaded from file using the Irrlicht formats (.obj, .3ds,
/// .X, etc). The mesh can be moving. The loaded mesh is used BOTH for display
/// in Irrlicht 3d view and for collision. The returned object has collision
/// detection turned ON by default. Works also for convex shapes, but the
/// performance is not as fast and robust as in case of native convex cases,
/// so whenever possible use addChBodySceneNode_easyConvexMesh.
ChApiIrr irr::scene::ISceneNode* addChBodySceneNode_easyConcaveMesh(
    ChSystem* asystem,
    irr::scene::ISceneManager* amanager,
    const char* mesh_filename,
    double mmass,
    const ChVector<>& position = ChVector<>(0, 0, 0),
    const ChQuaternion<>& rotation = ChQuaternion<>(1, 0, 0, 0),
    irr::scene::ISceneNode* aparent = 0,
    irr::s32 mid = -1);

/// @} irrlicht_module

}  // end namespace irrlicht
}  // end namespace chrono

#endif
