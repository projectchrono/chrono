//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHIRRNODE_H
#define CHIRRNODE_H

//////////////////////////////////////////////////
//
//   ChIrrNode.h
//
//   FOR IRRLICHT USERS ONLY!
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <irrlicht.h>
#include <ITimer.h>
#include "physics/ChSystem.h"


#define ESNT_CHIRRNODE 1201



namespace irr
{
namespace scene
{



/// Class for Irrlicht visualization. It will be managed by a ChIrrNodeAsset 
/// asset to be added among the ChBody assets. Such ChIrrNodeAsset can automatically
/// be populated with Irrlicht meshes that will correspond to shapes that
/// have been added as assets in ChBody.

class ChIrrNode : public scene::ISceneNode
{
	static int static_item_identifier;

	core::aabbox3d<f32> Box;

	chrono::ChSharedPtr<chrono::ChPhysicsItem> physicsitem;

	bool ChronoControlled;

public:

		/// Build a scene node for the Irrlicht Engine.
		/// This scene node also has a pointer to a rigid body for the Chrono::Engine 
		/// multibody simulation.
		/// To delete a ChIrrSceneNode node from an Irrlicht scene, use the remove() 
		/// function from the Irrlicht side (it won't delete the C::E body, though),
		/// or better delete the corresponding ChIrrlichtObj asset from the C::E side, or
		/// delete the full C::E body.
	ChIrrNode( 
					chrono::ChSharedPtr<chrono::ChPhysicsItem> mphysicsitem,   ///< pointer to the Chrono::Engine item (es. rigid body)
					ISceneNode* parent,  ///< the parent node in Irrlicht hierarchy
					ISceneManager* mgr,	 ///< the Irrlicht scene manager 
					s32 id				 ///< the Irrlicht identifier
					)
		:	scene::ISceneNode(parent, mgr, id) , 						 
			ChronoControlled(true)
	{
		#ifdef _DEBUG
			setDebugName("ChIrrNode");
		#endif
	

		// Set the shared pointer to the owner ChBody
		physicsitem = mphysicsitem;
		
		// set an unique identifier
		static_item_identifier++;
		GetPhysicsItem()->SetIdentifier(static_item_identifier);

	}


		/// Destructor.
	~ChIrrNode()
	{
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

	}

	virtual const core::aabbox3d<f32>& getBoundingBox() const
	{
		return Box;
	}

	virtual void setMaterialTexture (s32 textureLayer, video::ITexture *texture)
	{

	}

	ISceneNode* clone(ISceneNode* newParent, ISceneManager* newManager)
	{
		chrono::GetLog() << "Cloning!\n";
		if (!newParent)
			newParent = Parent;
		if (!newManager)
			newManager = SceneManager;

		ChIrrNode* nb = new ChIrrNode(this->physicsitem,newParent,
			newManager, this->ID);

		nb->cloneMembers(this, newManager);
		nb->Box = this->Box;
		nb->ChronoControlled = this->ChronoControlled;
		nb->physicsitem = this->physicsitem;

		if ( newParent )
			nb->drop();
		return nb;
	}


			/// This function is needed only when using the 'clones' feature, i.e. 
			/// shapes in assets define a sample that must be cloned (ex in ChParticleClones)
	bool SetupClones()
	{
		unsigned int needed_clones = physicsitem->GetAssetsFrameNclones();
		
		if (needed_clones)
		{
			unsigned int actual_clones = this->getChildren().getSize();
			
			// At least one clone 'sample' must be present (created with ChIrrAssetConverter::Update...)
			// otherwise just bail out.
			if (actual_clones ==0)
				return false;

			// Actual n.of children is already matching the n.of clones??
			// ...then do nothing
			if (needed_clones == actual_clones)
				return true;

			// Actual n.of children is less??
			// ...then clone the sample (the first node child of this node) 
			while (needed_clones > actual_clones)
			{
				core::list<ISceneNode*>::ConstIterator it = this->getChildren().begin();
				(*it)->clone(); 
				++actual_clones;
			}

			// Actual n.of children is more??
			// ...then delete clones 
			while (needed_clones < actual_clones)
			{
				core::list<ISceneNode*>::ConstIterator it = this->getChildren().end();
				(*it)->remove(); 
				--actual_clones;
			}
		}

		return true;
	}

	void OnAnimate(u32 timeMs)
	{

		if (IsVisible && ChronoControlled)
		{
			// reorient/reposition the scene node every frame
			if (physicsitem)
			{
				if (!physicsitem->GetAssetsFrameNclones())
				{
					ChIrrTools::alignIrrlichtNodeToChronoCsys(this, physicsitem->GetAssetsFrame().GetCoord() );	
				}
				else
				{
					// check that children clones are already as many as 
					// assets frame clones, and adjust it if not:
					if (SetupClones())
					{
						// make each clone node match the corresponding asset frame :
						unsigned int nclones = physicsitem->GetAssetsFrameNclones();
						unsigned int iclone = 0;
						core::list<ISceneNode*>::ConstIterator it = this->getChildren().begin();
						for (; it != Children.end(); ++it)
						{
							ChIrrTools::alignIrrlichtNodeToChronoCsys((*it), physicsitem->GetAssetsFrame(iclone).GetCoord() );	
							++iclone;
						}
					}
					
				}
			}
			
		}

		ISceneNode::OnAnimate(timeMs);
	}

	/*
	virtual IShadowVolumeSceneNode* addShadowVolumeSceneNode(const IMesh* shadowMesh=0, s32 id =-1,
	bool zfailmethod=true, f32 infinity=10000.0f)
	{
		if (child_mesh)
			return child_mesh->addShadowVolumeSceneNode(shadowMesh, id, zfailmethod, infinity);
		
		return false;
	};
	*/

		//
		// CHRONO::ENGINE SPECIFIC
		//

		/// Returns reference to the shared pointer which references the
		/// ChPhysicsItem (ex. a ChBody) wrapped by this scene node.
	chrono::ChSharedPtr<chrono::ChPhysicsItem>& GetPhysicsItem() {return physicsitem;}

		/// Returns true if the node is moved by Chrono::Engine
		/// simulation system.
	virtual bool IsChronoControlled() const { return ChronoControlled; }
		/// Set true if the node must be moved by Chrono::Engine simulation 
		/// system (it follows a ChPhysicsItem that contains a ChIrrNodeAsset, proxy to this).
	virtual void SetChronoControlled(const bool &controlled) { ChronoControlled = controlled; }
	

	virtual ESCENE_NODE_TYPE getType()   {  return (ESCENE_NODE_TYPE)ESNT_CHIRRNODE; }	
};



// Initialize static counter
int ChIrrNode::static_item_identifier = 0;




} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif // END of ChIrrNode.h

