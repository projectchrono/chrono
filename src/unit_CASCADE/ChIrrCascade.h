//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHIRRCASCADE_H
#define CHIRRCASCADE_H

//////////////////////////////////////////////////
//
//   ChIrrCascade.h
//
//   FOR IRRLICHT USERS ONLY!
//
//   Some functions to allow easy creation of
//   ChBodySceneNode C++ objects in Irrlicht+ChronoEngine+OpenCascade
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "unit_IRRLICHT/ChBodySceneNode.h"
#include "unit_IRRLICHT/ChIrrMeshTools.h"
#include "geometry/ChCTriangleMesh.h"
#include "physics/ChBodyAuxRef.h"
#include "ChIrrCascadeMeshTools.h"
#include "ChCascadeDoc.h"

namespace irr
{
namespace scene
{





class ChBodySceneNodeAuxRef : public scene::ISceneNode
{

	core::aabbox3d<f32> Box;

	IAnimatedMeshSceneNode* child_mesh;

	// Chrono Engine specific data
	chrono::ChSharedPtr<chrono::ChBodyAuxRef>* bodyp;
	int body_identifier;
	bool ChronoControlled;

public:

		/// Build a scene node for the Irrlicht Engine.
		/// This scene node is also a rigid body for the Chrono::Engine 
		/// multibody simulation.
		/// As soon as created, the wrapped ChBody is also added to 
		/// the Chrono::Engine
		/// To delete a ChBodyScene node from an Irrlicht scene, use the remove() 
		/// function only! (it will also be removed from the Chrono::Engine)
	ChBodySceneNodeAuxRef(chrono::ChSystem* msystem,   ///< pointer to the Chrono::Engine physical simulation system
					IAnimatedMesh* mesh,		 ///< a 3D mesh for representing the shape of the body 
					ISceneNode* parent,  ///< the parent node in Irrlicht hierarchy
					ISceneManager* mgr,	 ///< the Irrlicht scene manager 
					s32 id				 ///< the Irrlicht identifier
					)
		:	scene::ISceneNode(parent, mgr, id) , 						 
			ChronoControlled(true)
	{
		assert(msystem);

		#ifdef _DEBUG
			setDebugName("ChBodySceneNodeAuxRef");
		#endif

		child_mesh =0;
		if (mesh)
			child_mesh = mgr->addAnimatedMeshSceneNode(mesh, this);
		
		if (child_mesh)
			child_mesh->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);

		// Create the shared pointer, and the ChBody object
		// pointed by the shared pointer.
		// Creating dynamically the shared pointer from heap is not
		// nice to see, but it must be managed dynamically in this wrapper node..

		bodyp = new chrono::ChSharedPtr<chrono::ChBodyAuxRef>(new chrono::ChBodyAuxRef);
		
		// set an unique identifier
		body_identifier++;
		GetBody()->SetIdentifier(body_identifier);

		// Automatically add to the Chrono::Engine system.
		msystem->AddBody(GetBody());

	}


		/// Destructor.
		/// Note: as this Irrlicht node is destructed, it also automatically removes 
		/// the wrapped ChBody from the ChronoEngine.
	~ChBodySceneNodeAuxRef()
	{
		// Automatically remove from the Chrono::Engine system, if 
		// currently inserted in a system.
		if (GetBody()->GetSystem())
		{
			GetBody()->GetSystem()->RemoveBody(GetBody());
		}

		// Deleting the shared pointer will automatically delete
		// also the pointed ChBody, if needed (ie. if none else is
		// referencing it).
		delete bodyp; bodyp=0;
	}

		//
		// OVERRIDE/IMPLEMENT BASE IRRLICHT METHODS
		//

	virtual void OnRegisterSceneNode()
	{
		if (IsVisible)
			SceneManager->registerNodeForRendering(this);

		ISceneNode::OnRegisterSceneNode();
	}

	virtual void render()
	{
		if (child_mesh)
			child_mesh->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
	}

	virtual const core::aabbox3d<f32>& getBoundingBox() const
	{
		if (child_mesh)
			return child_mesh->getBoundingBox();
		else
			return Box;
	}

	virtual void setMaterialTexture (s32 textureLayer, video::ITexture *texture)
	{
		if (child_mesh)
			return child_mesh->setMaterialTexture (textureLayer, texture);
	}

	virtual u32 getMaterialCount()
	{
		if (child_mesh)
			return child_mesh->getMaterialCount();
		else 
			return 0;
	}

	virtual video::SMaterial& getMaterial(u32 i)
	{
		assert (child_mesh);
		return child_mesh->getMaterial(i);
	}	

	void OnAnimate(u32 timeMs)
	{
		//setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);

		if (child_mesh)
			child_mesh->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);

		if (IsVisible)
		{
			// reorient/reposition the scene node every frame
			if (bodyp && ChronoControlled)
			{

				// Output: will be an Irrlicht 4x4 matrix
				core::matrix4 irrMat;

				// Get the rigid body actual rotation, as a 3x3 matrix [A]
				chrono::ChMatrix33<> chMat = GetBody()->GetFrame_REF_to_abs().GetA();
				
				// Fill the upper 3x3 submatrix with the [A] matrix
				// transposed, since Irrlicht uses the row-major style as in D3D
				irrMat[0] = (irr::f32)chMat.GetElementN(0);
				irrMat[1] = (irr::f32)chMat.GetElementN(3);
				irrMat[2] = (irr::f32)chMat.GetElementN(6);

				irrMat[4] = (irr::f32)chMat.GetElementN(1);
				irrMat[5] = (irr::f32)chMat.GetElementN(4);
				irrMat[6] = (irr::f32)chMat.GetElementN(7);
				
				irrMat[8] = (irr::f32)chMat.GetElementN(2);
				irrMat[9] = (irr::f32)chMat.GetElementN(5);
				irrMat[10]= (irr::f32)chMat.GetElementN(8);

				irrMat[12]= (irr::f32)GetBody()->GetFrame_REF_to_abs().GetPos().x;
				irrMat[13]= (irr::f32)GetBody()->GetFrame_REF_to_abs().GetPos().y;
				irrMat[14]= (irr::f32)GetBody()->GetFrame_REF_to_abs().GetPos().z;

				// Clear the last column to 0 and set low-right corner to 1 
				// as in Denavitt-Hartemberg matrices, transposed.
				irrMat[3] = irrMat[7] = irrMat[11] = 0.0f;
				irrMat[15] = 1.0f;

				// Set position and rotation of node using the 4x4 Irrlicht matrix.
				setPosition(irrMat.getTranslation());
				setRotation(irrMat.getRotationDegrees());
			}
			
		}

		ISceneNode::OnAnimate(timeMs);
	}

	virtual IShadowVolumeSceneNode* addShadowVolumeSceneNode(const IMesh* shadowMesh=0, s32 id =-1,
	bool zfailmethod=true, f32 infinity=10000.0f)
	{
		if (child_mesh)
			return child_mesh->addShadowVolumeSceneNode(shadowMesh, id, zfailmethod, infinity);
		return false;
	};

		//
		// CHRONO::ENGINE SPECIFIC
		//

		/// Returns reference to the shared pointer which references the
		/// rigid body wrapped by this scene node.
	chrono::ChSharedPtr<chrono::ChBodyAuxRef>& GetBody() {return *bodyp;}

		/// Returns true if the node is moved by Chrono::Engine
		/// simulaiton system.
	virtual bool IsChronoControlled() const { return ChronoControlled; }
		/// Set true if you want Chrono::Engine to include this body in simulation.
	virtual void SetChronoControlled(const bool &controlled) { ChronoControlled = controlled; }

	IAnimatedMeshSceneNode* GetChildMesh() {return child_mesh;}

	virtual ESCENE_NODE_TYPE getType() const  {  return (ESCENE_NODE_TYPE)ESNT_CHBODY; }	
};



/// Easy-to-use function which creates a ChBodySceneNode 
/// corresponding to a OpenCascade mesh (assuming TopoDS_Shape location is relative to body csys)
/// Version (A), with full parameters:

static
ISceneNode* addChBodySceneNode_Cascade_A(chrono::ChSystem* asystem,
										   ISceneManager* amanager,
										   TopoDS_Shape& mshape_ref,				///< shape, in coords  relative to body ref
										   const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),	
										   const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
										   double mmass = 1.0, 
										   const chrono::ChVector<>& XXinertia = chrono::ChVector<>(1,1,1),
										   const chrono::ChVector<>& XYinertia = chrono::ChVector<>(0,0,0),
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	if (!aparent)
		aparent = amanager->getRootSceneNode();

	scene::SMesh* mmesh = new scene::SMesh();
	video::SColor clr(255, 100,120,125);
	irr::scene::ChIrrCascadeMeshTools::fillIrrlichtMeshFromCascade(mmesh, mshape_ref, 0.5);
	scene::SAnimatedMesh* amesh = new scene::SAnimatedMesh();
	amesh->addMesh(mmesh);
	mmesh->drop();

	// create a ChronoENGINE rigid body
	ChBodySceneNodeAuxRef* rigidBodyZ = new ChBodySceneNodeAuxRef(asystem, 
													amesh, 
													aparent,
													amanager,
													mid
													);

			// set some ChronoENGINE specific properties for the body...
	rigidBodyZ->GetBody()->SetPos(position);
	rigidBodyZ->GetBody()->SetRot(rotation);
	rigidBodyZ->GetBody()->SetMass(mmass);
	rigidBodyZ->GetBody()->SetInertiaXX(XXinertia);
	rigidBodyZ->GetBody()->SetInertiaXY(XYinertia); 
	rigidBodyZ->drop();

	amesh->drop();

	return rigidBodyZ;	
}



/// Super-easy-to-use function which creates a ChBodySceneNode 
/// corresponding to a OpenCascade mesh, where the position and rotation
/// of the reference is automatically set according to the OpenCascade shape:
/// Version (B), with auto computation of reference position and rotation:

static
ISceneNode* addChBodySceneNode_Cascade_B(chrono::ChSystem* asystem,
										   ISceneManager* amanager,
										   TopoDS_Shape& mshape_abs, 		///< shape, in absolute coords 
										   double mmass = 1.0, 
										   const chrono::ChVector<>& XXinertia = chrono::ChVector<>(1,1,1),
										   const chrono::ChVector<>& XYinertia = chrono::ChVector<>(0,0,0),
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	TopLoc_Location mloc = mshape_abs.Location();

	chrono::ChFrame<> mframe_ref_to_abs;
	chrono::cascade::ChCascadeDoc::FromCascadeToChrono(mloc, mframe_ref_to_abs);

	TopoDS_Shape objshape_abs = mshape_abs;
	objshape_abs.Location( TopLoc_Location() ); // Reset shape location to local ref csys (identity). 

	return addChBodySceneNode_Cascade_A(asystem, amanager, objshape_abs, mframe_ref_to_abs.GetPos(), mframe_ref_to_abs.GetRot(), mmass, XXinertia, XYinertia, aparent, mid);
}




/// Super-Super-easy-to-use function which creates a ChBodySceneNode 
/// corresponding to a OpenCascade mesh, where the position and rotation
/// of the reference is automatically set according to the COG of the OpenCascade shape:
/// Version (C), with auto computation of reference position and rotation, and mass and inertia

static
ISceneNode* addChBodySceneNode_Cascade_C(chrono::ChSystem* asystem,
										   ISceneManager* amanager,
										   TopoDS_Shape& mshape,				///< shape, with location respect to abs coords 
										   chrono::ChFrame<>* ref_to_abs = 0,	///< body reference position (if =0, use the mshape location)
										   double density = 1000.0, 
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	chrono::ChFrame<> frame_ref_to_abs;

	if (!ref_to_abs)
	{
		TopLoc_Location loc_shape_to_abs = mshape.Location();
		chrono::cascade::ChCascadeDoc::FromCascadeToChrono(loc_shape_to_abs, frame_ref_to_abs);
	}
	else
	{
		frame_ref_to_abs = *ref_to_abs;
	}
	
	TopoDS_Shape objshape = mshape;
	objshape.Location( TopLoc_Location() ); // Reset shape location to local ref csys (identity). 

	chrono::ChVector<> mcog;
	chrono::ChVector<> minertiaXX;
	chrono::ChVector<> minertiaXY;
	double mvol;
	double mmass;
	chrono::cascade::ChCascadeDoc::GetVolumeProperties(objshape, density, mcog, minertiaXX, minertiaXY, mvol, mmass); 


	ChBodySceneNodeAuxRef* mbody = (ChBodySceneNodeAuxRef*)addChBodySceneNode_Cascade_A(asystem, amanager, objshape, frame_ref_to_abs.GetPos(), frame_ref_to_abs.GetRot(), mmass, minertiaXX, minertiaXY, aparent, mid);

	chrono::ChFrame<>* frame_cog_to_ref = (chrono::ChFrame<>*)mbody->GetBody().get_ptr();
	frame_cog_to_ref->SetPos(mcog);
	frame_cog_to_ref->SetRot(chrono::QUNIT);

	return mbody;
}






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif // END of ChIrrCascade.h
