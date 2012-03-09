#ifndef CHBODYSCENENODE_H
#define CHBODYSCENENODE_H

//////////////////////////////////////////////////
//
//   ChBodySceneNode.h
//
//   FOR IRRLICHT USERS ONLY!
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <irrlicht.h>
#include <ITimer.h>
#include "physics/ChSystem.h"


#define ESNT_ChBODY 1200



static int body_identifier = 0;

namespace irr
{
namespace scene
{



class ChBodySceneNode : public scene::ISceneNode
{

	core::aabbox3d<f32> Box;

	IAnimatedMeshSceneNode* child_mesh;

	// Chrono Engine specific data
	chrono::ChSharedPtr<chrono::ChBody>* bodyp;
	bool ChronoControlled;

public:

		/// Build a scene node for the Irrlicht Engine.
		/// This scene node is also a rigid body for the Chrono::Engine 
		/// multibody simulation.
		/// As soon as created, the wrapped ChBody is also added to 
		/// the Chrono::Engine
		/// To delete a ChBodyScene node from an Irrlicht scene, use the remove() 
		/// function only! (it will also be removed from the Chrono::Engine)
	ChBodySceneNode(chrono::ChSystem* msystem,   ///< pointer to the Chrono::Engine physical simulation system
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
			setDebugName("ChBodySceneNode");
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

		bodyp = new chrono::ChSharedPtr<chrono::ChBody>(new chrono::ChBody);
		
		// set an unique identifier
		body_identifier++;
		GetBody()->SetIdentifier(body_identifier);

		// Automatically add to the Chrono::Engine system.
		msystem->AddBody(GetBody());

	}


		/// Destructor.
		/// Note: as this Irrlicht node is destructed, it also automatically removes 
		/// the wrapped ChBody from the ChronoEngine.
	~ChBodySceneNode()
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
				chrono::ChMatrix33<>* chMat = GetBody()->GetFrame_REF_to_abs().GetA();
				
				// Fill the upper 3x3 submatrix with the [A] matrix
				// transposed, since Irrlicht uses the row-major style as in D3D
				irrMat[0] = (irr::f32)chMat->GetElementN(0);
				irrMat[1] = (irr::f32)chMat->GetElementN(3);
				irrMat[2] = (irr::f32)chMat->GetElementN(6);

				irrMat[4] = (irr::f32)chMat->GetElementN(1);
				irrMat[5] = (irr::f32)chMat->GetElementN(4);
				irrMat[6] = (irr::f32)chMat->GetElementN(7);
				
				irrMat[8] = (irr::f32)chMat->GetElementN(2);
				irrMat[9] = (irr::f32)chMat->GetElementN(5);
				irrMat[10]= (irr::f32)chMat->GetElementN(8);

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
	chrono::ChSharedPtr<chrono::ChBody>& GetBody() {return *bodyp;}

		/// Returns true if the node is moved by Chrono::Engine
		/// simulaiton system.
	virtual bool IsChronoControlled() const { return ChronoControlled; }
		/// Set true if you want Chrono::Engine to include this body in simulation.
	virtual void SetChronoControlled(const bool &controlled) { ChronoControlled = controlled; }

	IAnimatedMeshSceneNode* GetChildMesh() {return child_mesh;}

	virtual ESCENE_NODE_TYPE getType()   {  return (ESCENE_NODE_TYPE)ESNT_ChBODY; }	
};














} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif // END of ChBodySceneNode.h

