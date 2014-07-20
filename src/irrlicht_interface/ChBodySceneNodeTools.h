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

//////////////////////////////////////////////////
//
//   ChBodySceneNodeTools.h
//
//   FOR IRRLICHT USERS ONLY!
//
//   Some functions to allow easy creation of
//   ChBodySceneNode C++ objects in Irrlicht+ChronoEngine
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

//
//  ***OBSOLETE*** 
//  better: use the ChIrrNodeAsset approach
//


#include "ChIrrAppInterface.h"
#include "ChBodySceneNode.h"
#include "ChIrrMeshTools.h"
#include "geometry/ChCSphere.h"
#include "geometry/ChCBox.h"
#include "geometry/ChCTriangleMeshSoup.h"

namespace irr
{
namespace scene
{



/// Easy-to-use function which creates a ChBodySceneNode 
/// with given position of COG, inserts it into the Irrlicht 
/// scene etc.

static
ISceneNode* addChBodySceneNode(chrono::ChSystem* asystem,
										   ISceneManager* amanager,
										   IAnimatedMesh* amesh, 
										   double mmass = 1.0, 
										   const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),	
										   const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	if (!aparent)
		aparent = amanager->getRootSceneNode();

	// create a ChronoENGINE rigid body
	ChBodySceneNode* rigidBodyZ = new ChBodySceneNode(asystem, 
													amesh, 
													aparent,
													amanager,
													mid
													);
			// set some ChronoENGINE specific properties for the body...
	rigidBodyZ->GetBody()->SetPos(position);
	rigidBodyZ->GetBody()->SetRot(rotation);
	rigidBodyZ->GetBody()->SetMass(mmass);
	 
	rigidBodyZ->drop();

	return rigidBodyZ;	
}


/// JCM, added to allow user to specify an offset distance between COG and mesh location
//  Note: the mesh remains in the same location, but the COG of the rigid body changes by the offset amount
static
ISceneNode* addChBodySceneNode_offsetCOG(chrono::ChSystem* asystem,
										   ISceneManager* amanager,
										   IAnimatedMesh* amesh, 
										   double mmass = 1.0, 
										   const chrono::ChVector<>& mesh_position = chrono::ChVector<>(0,0,0),	
										   const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
										   const chrono::ChVector<>& COG_offset = chrono::ChVector<>(0,0,0),
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	if (!aparent)
		aparent = amanager->getRootSceneNode();

	// create a ChronoENGINE rigid body
	ChBodySceneNode* rigidBodyZ = new ChBodySceneNode(asystem, 
													amesh, 
													aparent,
													amanager,
													mid,
													COG_offset);
	// set some ChronoENGINE specific properties for the body...
	rigidBodyZ->GetBody()->SetPos(mesh_position);
	rigidBodyZ->GetBody()->SetRot(rotation);
	rigidBodyZ->GetBody()->SetMass(mmass);
	 
	rigidBodyZ->drop();

	return rigidBodyZ;	
}








/// Easy-to-use function which creates a ChBodySceneNode 
/// representing a sphere, ready to use for collisions (otherwise
/// you could use addChBodySceneNode() and add collision
/// geometry by hand, but the following is easier).
/// The returned object has collision detection turned ON by default.

static
ISceneNode* addChBodySceneNode_easySphere(chrono::ChSystem* asystem,
										   ISceneManager* amanager,
										   double mmass = 1.0, 
										   const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
										   double mradius= 1.0,
										   int Hslices = 15,
										   int Vslices = 8,
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	static IAnimatedMesh* sphereMesh = 0;
	
	if (!sphereMesh) 
		sphereMesh = createEllipticalMesh(1.0,1.0,-2,+2,0,Hslices,Vslices);

	// create a ChronoENGINE rigid body
	ChBodySceneNode* rigidBodyZ = (ChBodySceneNode*)addChBodySceneNode(asystem, amanager, sphereMesh, mmass, position, chrono::ChQuaternion<>(1,0,0,0), aparent, mid);
	
	rigidBodyZ->setScale(irr::core::vector3df((irr::f32)mradius,(irr::f32)mradius,(irr::f32)mradius));

	rigidBodyZ->GetBody()->GetCollisionModel()->ClearModel();
	rigidBodyZ->GetBody()->GetCollisionModel()->AddSphere(mradius); 
	rigidBodyZ->GetBody()->GetCollisionModel()->BuildModel();
	rigidBodyZ->GetBody()->SetCollide(true);

	return rigidBodyZ;	
}




/// Easy-to-use function which creates a ChBodySceneNode 
/// representing a box, ready to use for collisions (otherwise
/// you could use addChBodySceneNode() and add collision
/// geometry by hand, but the following is easier).
/// The returned object has collision detection turned ON by default.

static
ISceneNode* addChBodySceneNode_easyBox(chrono::ChSystem* asystem,
										   ISceneManager* amanager,
										   double mmass = 1.0, 
										   const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
										   const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
										   const chrono::ChVector<>& size = chrono::ChVector<>(1,1,1),
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	static IAnimatedMesh* cubeMesh = 0;
	
	if (!cubeMesh) 
		cubeMesh = amanager->getMesh((irrlicht_default_obj_dir+"cube.obj").c_str());
	
	// create a ChronoENGINE rigid body
	ChBodySceneNode* rigidBodyZ = (ChBodySceneNode*)addChBodySceneNode(asystem, amanager, cubeMesh, mmass, position, rotation, aparent, mid);
	
	chrono::ChVector<> hsize = size*0.5;

	core::vector3df irrsize((irr::f32)hsize.x, (irr::f32)hsize.y, (irr::f32)hsize.z);
	rigidBodyZ->setScale(irrsize);

	rigidBodyZ->GetBody()->GetCollisionModel()->ClearModel();
	rigidBodyZ->GetBody()->GetCollisionModel()->AddBox(hsize.x, hsize.y, hsize.z); 
	rigidBodyZ->GetBody()->GetCollisionModel()->BuildModel();
	rigidBodyZ->GetBody()->SetCollide(true);

	return rigidBodyZ;	
}


/// Easy-to-use function which creates a ChBodySceneNode 
/// representing a cylinder, ready to use for collisions (otherwise
/// you could use addChBodySceneNode() and add collision
/// geometry by hand, but the following is easier).
/// The returned object has collision detection turned ON by default.

static
ISceneNode* addChBodySceneNode_easyCylinder(chrono::ChSystem* asystem,
										   ISceneManager* amanager,
										   double mmass = 1.0, 
										   const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
										   const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
										   const chrono::ChVector<>& size = chrono::ChVector<>(1,1,1),
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	static IAnimatedMesh* cylinderMesh = 0;
	
	if (!cylinderMesh) 
		cylinderMesh = amanager->getMesh((irrlicht_default_obj_dir+"cylinder.obj").c_str());

	// create a ChronoENGINE rigid body
	ChBodySceneNode* rigidBodyZ = (ChBodySceneNode*)addChBodySceneNode(asystem, amanager, cylinderMesh, mmass, position, rotation, aparent, mid);
	
	chrono::ChVector<> hsize = size*0.5;

	core::vector3df irrsize((irr::f32)hsize.x, (irr::f32)hsize.y, (irr::f32)hsize.z);
	rigidBodyZ->setScale(irrsize);

	rigidBodyZ->GetBody()->GetCollisionModel()->ClearModel();
	rigidBodyZ->GetBody()->GetCollisionModel()->AddCylinder(hsize.x, hsize.z,   hsize.y);  // radius, radius, height on y
	rigidBodyZ->GetBody()->GetCollisionModel()->BuildModel();
	rigidBodyZ->GetBody()->SetCollide(true);

	return rigidBodyZ;	
}


/// Easy-to-use function which creates a ChBodySceneNode 
/// representing a barrel, ready to use for collisions (otherwise
/// you could use addChBodySceneNode() and add collision
/// geometry by hand, but the following is easier).
/// The returned object has collision detection turned ON by default.

static
ISceneNode* addChBodySceneNode_easyBarrel(chrono::ChSystem* asystem,
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
										   s32 mid=-1
										   )
{
	IAnimatedMesh* barrellMesh = createEllipticalMesh((f32)mradiusH, (f32)mradiusV,(f32)mYlow,(f32)mYhigh,(f32)mOffset,Hslices,Vslices);

	// create a ChronoENGINE rigid body
	ChBodySceneNode* rigidBodyZ = (ChBodySceneNode*)addChBodySceneNode(asystem, amanager, barrellMesh, mmass, position, chrono::ChQuaternion<>(1,0,0,0), aparent, mid);

	rigidBodyZ->GetBody()->GetCollisionModel()->ClearModel();
	rigidBodyZ->GetBody()->GetCollisionModel()->AddBarrel(mYlow, mYhigh, mradiusV, mradiusH, mOffset); 
	rigidBodyZ->GetBody()->GetCollisionModel()->BuildModel();
	rigidBodyZ->GetBody()->SetCollide(true);

	return rigidBodyZ;	
}




/// Easy-to-use function that creates a ChBodySceneNode 
/// representing a clone of another ChBodySceneNode, but at a
/// different position and rotation.
/// NOTE! the collision shapes (if any) in the source object
/// will be _shared_ with the cloned object, so if you have to 
/// create for exampe 1000 identical objects (ex. cubes) this function
/// will allow you to replicate 999 times a single cube that you
/// once created calling addChBodySceneNode_easyBox() : this will 
/// be more memory-efficient than calling 1000 times addChBodySceneNode_easyBox().
/// Shared geometries will be deallocated automatically thank to shared pointers.


static
ISceneNode* addChBodySceneNode_easyClone(chrono::ChSystem* asystem,
										   ISceneManager* amanager,
										   ChBodySceneNode* source, 
										   const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
										   const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	// create a ChronoENGINE rigid body
	ChBodySceneNode* rigidBodyZ = (ChBodySceneNode*)addChBodySceneNode(asystem, amanager,
		0,//source->GetChildMesh()->getMesh(), 
		1, 
		position, rotation, aparent, mid);
	
	rigidBodyZ->GetBody()->Copy(source->GetBody().get_ptr()); // copy all settings of the original body (masses, inertia, etc.)
	rigidBodyZ->GetBody()->SetSystem(source->GetBody()->GetSystem()); // because Copy() set system to null..
	rigidBodyZ->GetBody()->SetPos(position);	// because Copy() changed it
	rigidBodyZ->GetBody()->SetRot(rotation);	// because Copy() changed it

	rigidBodyZ->setScale(source->getScale()); 

	rigidBodyZ->GetBody()->GetCollisionModel()->ClearModel();
	rigidBodyZ->GetBody()->GetCollisionModel()->AddCopyOfAnotherModel(source->GetBody()->GetCollisionModel());
	rigidBodyZ->GetBody()->GetCollisionModel()->BuildModel();
	rigidBodyZ->GetBody()->SetCollide(true);

	return rigidBodyZ;	
}




/// Easy-to-use function which creates a ChBodySceneNode 
/// representing a GENERIC mesh of arbitrary shape, loaded from file using the 
/// Irrlicht formats (.obj, .3ds, .X, etc) ready to use for collisions.
/// The loaded mesh is used BOTH for display in Irrlicht 3d view and for collision.
/// The returned object has collision detection turned ON by default.

static
ISceneNode* addChBodySceneNode_easyGenericMesh(chrono::ChSystem* asystem,
										   ISceneManager* amanager,
										   double mmass = 1.0, 
										   const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
										   const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
										   const char* mesh_filemane =0,
										   bool  is_static = true,
										   bool  is_convex = true,
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	IAnimatedMesh* genericMesh = 0;
	
	genericMesh = amanager->getMesh(mesh_filemane);
	
	assert(genericMesh);


	chrono::geometry::ChTriangleMeshSoup temp_trianglemesh; // temp., only in function scope, since AddTriangleMesh doesn't reference by striding interface -just copy 
	fillChTrimeshFromIrlichtMesh(&temp_trianglemesh, genericMesh->getMesh(0));

	// create a ChronoENGINE rigid body
	ChBodySceneNode* rigidBodyZ = (ChBodySceneNode*)addChBodySceneNode(asystem, amanager, genericMesh, mmass, position, rotation, aparent, mid);
	
	rigidBodyZ->GetBody()->GetCollisionModel()->ClearModel();
	rigidBodyZ->GetBody()->GetCollisionModel()->AddTriangleMesh(temp_trianglemesh, is_static, is_convex); 
	rigidBodyZ->GetBody()->GetCollisionModel()->BuildModel();
	rigidBodyZ->GetBody()->SetCollide(true);

	rigidBodyZ->GetBody()->SetBodyFixed(is_static);

	return rigidBodyZ;	
}

/// Easy-to-use function which creates a ChBodySceneNode 
/// representing a STATIC shape, as a mesh loaded from file using the 
/// Irrlicht formats (.obj, .3ds, .X, etc).
/// The mesh can be either convex or concave: it works anyway (but it should NOT move!)
/// The loaded mesh is used BOTH for display in Irrlicht 3d view and for collision.
/// The returned object has collision detection turned ON by default.
/// The returned object is fixed by default (no need to do SetBodyFixed(true) ).

static
ISceneNode* addChBodySceneNode_easyStaticMesh(chrono::ChSystem* asystem,
										   ISceneManager* amanager, 
										   const char* mesh_filename,
										   const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
										   const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	return addChBodySceneNode_easyGenericMesh(asystem, amanager, 1.0, position, rotation, mesh_filename, true, false, aparent, mid); 
}

/// Easy-to-use function which creates a ChBodySceneNode 
/// representing a CONVEX shape, as a mesh loaded from file using the 
/// Irrlicht formats (.obj, .3ds, .X, etc).
/// The mesh can be moving. 
/// If mesh is not concave or has holes/gaps, its convex hull is used anyway. 
/// Best used with _simple_ meshes, not too many points.
/// The loaded mesh is used BOTH for display in Irrlicht 3d view and for collision.
/// The returned object has collision detection turned ON by default.

static
ISceneNode* addChBodySceneNode_easyConvexMesh(chrono::ChSystem* asystem,
										   ISceneManager* amanager, 
										   const char* mesh_filename,
										   double mmass,
										   const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
										   const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	return addChBodySceneNode_easyGenericMesh(asystem, amanager, mmass, position, rotation, mesh_filename, false, true, aparent, mid); 
}

/// Easy-to-use function which creates a ChBodySceneNode 
/// representing a CONCAVE shape, as a mesh loaded from file using the 
/// Irrlicht formats (.obj, .3ds, .X, etc).
/// The mesh can be moving. 
/// The loaded mesh is used BOTH for display in Irrlicht 3d view and for collision.
/// The returned object has collision detection turned ON by default.
/// Works also for convex shapes, but the performance is not as fast and robust as in case of native
/// convex cases, so whenever possible use addChBodySceneNode_easyConvexMesh.

static
ISceneNode* addChBodySceneNode_easyConcaveMesh(chrono::ChSystem* asystem,
										   ISceneManager* amanager, 
										   const char* mesh_filename,
										   double mmass,
										   const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),
										   const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	return addChBodySceneNode_easyGenericMesh(asystem, amanager, mmass, position, rotation, mesh_filename, false, false, aparent, mid); 
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif // END of ChBodySceneNode.h

