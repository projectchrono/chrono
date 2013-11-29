//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHPARTICLESSCENENODE_H
#define CHPARTICLESSCENENODE_H

//////////////////////////////////////////////////
//
//   ChParticlesSceneNode.h
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
#include "physics/ChParticlesClones.h"
#include "ChIrrMeshTools.h"

#define ESNT_CHPARTICLES 1201



static int particles_identifier = 0;

namespace irr
{
namespace scene
{



class ChParticlesSceneNode : public scene::ISceneNode
{

	core::aabbox3d<f32> Box;

	IAnimatedMeshSceneNode* child_mesh;
	IAnimatedMesh* sample_mesh;
	irr::core::vector3df mesh_scale;

	s32 Nchildren;

	// Chrono Engine specific data
	chrono::ChSharedPtr<chrono::ChParticlesClones>* particlep;
	bool ChronoControlled;

	video::ITexture *sample_texture;	

public:

		/// Build a scene node for the Irrlicht Engine.
		/// This scene node is also a rigid body for the Chrono::Engine 
		/// multibody simulation.
		/// As soon as created, the wrapped ChParticlesClones is also added to 
		/// the Chrono::Engine
		/// To delete a ChParticlesClonesScene node from an Irrlicht scene, use the remove() 
		/// function only! (it will also be removed from the Chrono::Engine)
	ChParticlesSceneNode(chrono::ChSystem* msystem,   ///< pointer to the Chrono::Engine physical simulation system
					IAnimatedMesh* mesh,		 ///< a sample 3D mesh for representing the shape of each particle 
					irr::core::vector3df mmesh_scale, ///< scale of the sample mesh
					ISceneNode* parent,  ///< the parent node in Irrlicht hierarchy
					ISceneManager* mgr,	 ///< the Irrlicht scene manager 
					s32 id				 ///< the Irrlicht identifier	
					)
		:	scene::ISceneNode(parent, mgr, id) , 						 
			ChronoControlled(true)
	{
		assert(msystem);

		#ifdef _DEBUG
			setDebugName("ChParticlesSceneNode");
		#endif

		child_mesh =0;

		sample_mesh = mesh;
		sample_mesh->grab();

		mesh_scale = mmesh_scale;

/*		if (mesh)
		{
			for (int j = 0; j<num; j++)
			{
				child_mesh = mgr->addAnimatedMeshSceneNode(sample_mesh, this);
				child_mesh->setScale(mesh_scale);

				if (child_mesh)
						child_mesh->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
			}
		}

		Nchildren = num;
*/
		Nchildren = 0;
		// Create the shared pointer, and the ChParticlesClones object
		// pointed by the shared pointer.
		// Creating dynamically the shared pointer from heap is not
		// nice to see, but it must be managed dynamically in this wrapper node..

		particlep = new chrono::ChSharedPtr<chrono::ChParticlesClones>(new chrono::ChParticlesClones);
		
		// set an unique identifier
		particles_identifier++;
		GetParticles()->SetIdentifier(particles_identifier);

		// Automatically add to the Chrono::Engine system.
		msystem->Add(GetParticles());

		sample_texture = 0;
	}

		/// Destructor.
		/// Note: as this Irrlicht node is destructed, it also automatically removes 
		/// the wrapped ChParticlesClones from the ChronoEngine.
	~ChParticlesSceneNode()
	{
		// Automatically remove from the Chrono::Engine system, if 
		// currently inserted in a system.
		if (GetParticles()->GetSystem())
		{
			GetParticles()->GetSystem()->Remove(GetParticles());
		}

		// Deleting the shared pointer will automatically delete
		// also the pointed ChParticlesClones, if needed (ie. if none else is
		// referencing it).
		delete particlep; particlep=0;

		if (sample_mesh)
			sample_mesh->drop();
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
		//if (child_mesh)
		//	child_mesh->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
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
		this->sample_texture = texture;
		//if (child_mesh)
		//	return child_mesh->setMaterialTexture (textureLayer, texture);
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
		UpdateChildrenHierarchy();

		//setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);

		//if (child_mesh)
		//	child_mesh->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);

		if (IsVisible)
		{
			// reorient/reposition the particle nodes every frame
			if (particlep && ChronoControlled)
			{	
				core::list<ISceneNode*>::ConstIterator it = getChildren().begin();

				for (unsigned int j = 0; j< (*particlep)->GetNparticles(); j++)
				{
					// Output: will be an Irrlicht 4x4 matrix
					core::matrix4 irrMat;

					// Get the particle actual rotation, as a 3x3 matrix [A]
					chrono::ChMatrix33<>* chMat = (*particlep)->GetParticle(j).GetA();
					
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

					irrMat[12]= (irr::f32)(*particlep)->GetParticle(j).GetPos().x;
					irrMat[13]= (irr::f32)(*particlep)->GetParticle(j).GetPos().y;
					irrMat[14]= (irr::f32)(*particlep)->GetParticle(j).GetPos().z;

					// Clear the last column to 0 and set low-right corner to 1 
					// as in Denavitt-Hartemberg matrices, transposed.
					irrMat[3] = irrMat[7] = irrMat[11] = 0.0f;
					irrMat[15] = 1.0f;

					// Set position and rotation of node using the 4x4 Irrlicht matrix.
					(*it)->setPosition(irrMat.getTranslation());
					(*it)->setRotation(irrMat.getRotationDegrees());

					++it;
					if (it == getChildren().end()) 
						break;
				}

			}
			
		}

		ISceneNode::OnAnimate(timeMs);
	}

	virtual IShadowVolumeSceneNode* addShadowVolumeSceneNode(const IMesh* shadowMesh=0, s32 id =-1,
	bool zfailmethod=true, f32 infinity=10000.0f)
	{
		if (child_mesh)
			return child_mesh->addShadowVolumeSceneNode(shadowMesh, id, zfailmethod, infinity);
	};

	virtual ESCENE_NODE_TYPE getType() const  {  return (ESCENE_NODE_TYPE)ESNT_CHPARTICLES; }


		//
		// CHRONO::ENGINE SPECIFIC
		//

		/// Returns reference to the shared pointer which references the
		/// rigid body wrapped by this scene node.
	chrono::ChSharedPtr<chrono::ChParticlesClones>& GetParticles() {return *particlep;}

		/// Returns true if the node is moved by Chrono::Engine
		/// simulaiton system.
	virtual bool IsChronoControlled() const { return ChronoControlled; }
		/// Set true if you want Chrono::Engine to include this body in simulation.
	virtual void SetChronoControlled(const bool &controlled) { ChronoControlled = controlled; }

	//IAnimatedMeshSceneNode* GetChildMesh() {return child_mesh;}

private:
		/// Check if the particle amount corresponds to the children meshes, and add other meshes if needed.
	void UpdateChildrenHierarchy()
	{
		unsigned int npart = (*particlep)->GetNparticles();
		if (this->Nchildren != npart)
		{
			// delete all children mesh nodes
			this->removeAll();
			// add the new amount of mesh nodes (suboptimal update, but enough for demos)
			if (sample_mesh)
			{
				for (unsigned int j = 0; j<npart; j++)
				{
					child_mesh = SceneManager->addAnimatedMeshSceneNode(sample_mesh, this);
					child_mesh->setScale(mesh_scale);
					if (this->sample_texture)
						child_mesh->setMaterialTexture(0, this->sample_texture);

					if (child_mesh)
						child_mesh->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
				}
			}
			this->Nchildren = npart;
		}
	}


};




/// Easy-to-use function which creates a ChParticlesSceneNode 
/// and inserts it into the Irrlicht scene.

static
ISceneNode* addChParticlesSceneNode(chrono::ChSystem* asystem,
										   ISceneManager* amanager,
										   IAnimatedMesh* amesh,
										   irr::core::vector3df amesh_scale, 
										   double mmass = 1.0, 
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	if (!aparent)
		aparent = amanager->getRootSceneNode();

	// create a ChronoENGINE rigid body
	ChParticlesSceneNode* particleObj = new ChParticlesSceneNode(asystem, 
													amesh, 
													amesh_scale,
													aparent,
													amanager,
													mid
													);
			// set some ChronoENGINE specific properties for the particle cluster...
	particleObj->GetParticles()->SetMass(mmass);
	 
	particleObj->drop();

	return particleObj;	
}


/// Easy-to-use function which creates a ChParticlesSceneNode  
/// representing a cluster of particles, ready to use for collisions (otherwise
/// you could use addChBodySceneNode() and add collision
/// geometry by hand, but the following is easier).
/// The returned object has collision detection turned ON by default.

static
ISceneNode* addChParticlesSceneNode_easySpheres(chrono::ChSystem* asystem,
										   ISceneManager* amanager,
										   double mmass = 1.0, 
										   double mradius= 1.0,
										   int Hslices = 12,
										   int Vslices = 6,
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	static IAnimatedMesh* sphereMesh = 0;
	
	if (!sphereMesh) 
		sphereMesh = createEllipticalMesh(1.0,1.0,-2,+2,0,Hslices,Vslices);

	irr::core::vector3df mmeshscale((irr::f32)mradius,(irr::f32)mradius,(irr::f32)mradius);

	// create a ChronoENGINE rigid body
	ChParticlesSceneNode* particleObj = (ChParticlesSceneNode*)addChParticlesSceneNode(asystem, amanager, sphereMesh, mmeshscale, mmass, aparent, mid);
	
	particleObj->GetParticles()->SetCollide(false); // make sure you are not defining the 'sample' collision model with collide =on

	particleObj->GetParticles()->GetCollisionModel()->ClearModel();
	particleObj->GetParticles()->GetCollisionModel()->AddSphere(mradius); 
	particleObj->GetParticles()->GetCollisionModel()->BuildModel();

	particleObj->GetParticles()->SetCollide(true);

	return particleObj;	
}








} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif // END of ChBodySceneNode.h

