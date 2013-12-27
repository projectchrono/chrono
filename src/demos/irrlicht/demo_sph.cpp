//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   Demo code about  
//
//     - SPH smooth particle hydrodynamics
//   
//  
//	 CHRONO 
//   ------
//   Multibody dinamics engine
//  
// ------------------------------------------------ 
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
 
   
 
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "physics/ChProximityContainerSPH.h"
#include "physics/ChMatterSPH.h"
#include "physics/ChContactContainerNodes.h"
#include "irrlicht_interface/ChBodySceneNode.h"
#include "irrlicht_interface/ChBodySceneNodeTools.h" 
#include "irrlicht_interface/ChIrrAppInterface.h"
#include "core/ChRealtimeStep.h"

#include <irrlicht.h>
 


// Use the namespace of Chrono

using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;
         
using namespace core;
using namespace scene; 
using namespace video;
using namespace io; 
using namespace gui; 




void create_some_falling_items(ChIrrAppInterface& mapp) 
{
	// box data
	double xsize =0.5;
	double zsize =0.5;
	double height=0.3;
	double thick =0.1;

	ChSystem* mphysicalSystem = mapp.GetSystem();
	ISceneManager* msceneManager = mapp.GetSceneManager();
	IVideoDriver* driver = mapp.GetVideoDriver();
	
		// Create the SPH fluid
	ChSharedPtr<ChMatterSPH> myfluid(new ChMatterSPH);

		// Use the FillBox easy way to create the set of SPH particles
	myfluid->FillBox(ChVector<>(xsize-0.2,height,zsize), // size of box 
					xsize/11.0,			// resolution step
					1000,				// initial density
					ChCoordsys<>(ChVector<>(0.1,height*0.5+0.1,0),QUNIT), // position & rotation of box
					true,				// do a centered cubic lattice initial arrangement 
					2.2,				// set the kernel radius (as multiples of step)
					0.3);				// the randomness to avoid too regular initial lattice

		// Set some material properties of the SPH fluid
	myfluid->GetMaterial().Set_viscosity(0.05);
	myfluid->GetMaterial().Set_pressure_stiffness(300);

		// Add the SPH fluid matter to the system
	myfluid->SetCollide(true);
	mphysicalSystem->Add(myfluid);



	// Create some spheres that will fall

	ChBodySceneNode* mrigidBody; 

	for (int bi = 0; bi < 0; bi++) 
	{    
		mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easySphere(
											mphysicalSystem, msceneManager,
											10.0,
											ChVector<>(-5+ChRandom()*10, 4+bi*0.05, -5+ChRandom()*10),
											1.1);
   
		mrigidBody->GetBody()->SetFriction(0.0f); 
		mrigidBody->addShadowVolumeSceneNode();


		video::ITexture* sphereMap = driver->getTexture("../data/bluwhite.png");
		mrigidBody->setMaterialTexture(0,	sphereMap);
	} 

	// Create the five walls of the rectangular container, using
	// fixed rigid bodies of 'box' type:

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(0,-thick*0.5,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(xsize+2*thick,thick,zsize+2*thick) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->GetBody()->SetFriction(0.2f); 

	video::ITexture* cubeMap = driver->getTexture("../data/blu.png");
	mrigidBody->setMaterialTexture(0,	cubeMap);

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(-xsize*0.5-thick*0.5, height*0.5,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(thick,height,zsize) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);


	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(xsize*0.5+thick*0.5, height*0.5,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(thick,height,zsize) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(0, height*0.5,-zsize*0.5-thick*0.5),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(xsize+2*thick,height,thick) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(0.2, height*0.5, zsize*0.5+thick*0.5),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(xsize+2*thick,height,thick) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);
 
	// another floor

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(0,-0.5,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(2,0.1,2) );
	mrigidBody->GetBody()->SetBodyFixed(true); 
	mrigidBody->GetBody()->SetFriction(0.2f);


} 
     
  

 
   
 
int main(int argc, char* argv[])
{

	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed. 
	DLL_CreateGlobals();

	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem;

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrAppInterface application(&mphysicalSystem, L"SPH fluid",core::dimension2d<u32>(800,600),false);


	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0,1,-1));

 
	// Create all the rigid bodies.

	create_some_falling_items(application);
 

	// IMPORTANT!
	// This takes care of the interaction between the particles of the SPH material
	ChSharedPtr<ChProximityContainerSPH> my_sph_proximity(new ChProximityContainerSPH);
	mphysicalSystem.Add(my_sph_proximity);
	
	// IMPORTANT!
	// This takes care of the contact between the particles of the SPH material and the walls
	ChSharedPtr<ChContactContainerNodes> my_nodes_container(new ChContactContainerNodes);
	mphysicalSystem.Add(my_nodes_container);
	
	
	// Modify some setting of the physical system for the simulation, if you want

	mphysicalSystem.SetIterLCPmaxItersSpeed(8); // lower the LCP iters, no needed here


	// 
	// THE SOFT-REAL-TIME CYCLE
	//

	static int printed_prox = 0;

	application.SetStepManage(true);
	application.SetTimestep(0.01);

	while(application.GetDevice()->run()) 
	{
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		application.DrawAll();
		
		ChSystem::IteratorOtherPhysicsItems myiter = mphysicalSystem.IterBeginOtherPhysicsItems();
		while (myiter != mphysicalSystem.IterEndOtherPhysicsItems())
		{ 
			if (ChMatterSPH* myfluid = dynamic_cast<ChMatterSPH*> ( (*myiter).get_ptr() ) )
			{
				for (unsigned int ip = 0; ip < myfluid->GetNnodes(); ip++)
				{
					ChSharedPtr<ChNodeSPH> mnode (myfluid->GetNode(ip));
					ChVector<> mv = mnode->GetPos();
					float rad = (float)mnode->GetKernelRadius(); 
					core::vector3df mpos((irr::f32)mv.x, (irr::f32)mv.y, (irr::f32)mv.z);
					core::position2d<s32> spos = application.GetSceneManager()->getSceneCollisionManager()->getScreenCoordinatesFrom3DPosition(mpos);
					application.GetVideoDriver()->draw2DRectangle(video::SColor(100,200,200,230), 
									core::rect<s32>(spos.X-2, spos.Y-2, spos.X+2, spos.Y+2) );
					/*
					application.GetVideoDriver()->setTransform(video::ETS_WORLD, core::matrix4());
					application.GetVideoDriver()->draw3DBox( core::aabbox3d<f32>(
									(irr::f32)mv.x-rad ,(irr::f32)mv.y-rad , (irr::f32)mv.z-rad    , 
									(irr::f32)mv.x+rad ,(irr::f32)mv.y+rad , (irr::f32)mv.z+rad )   ,
									video::SColor(300,200,200,230) );
					*/
					
					/*
					double strain_scale =1;
					ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(), mnode->GetPos()+(VECT_X*mnode->p_strain.XX()* strain_scale), video::SColor(255,255,0,0),false);
					ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(), mnode->GetPos()+(VECT_Y*mnode->p_strain.YY()* strain_scale), video::SColor(255,0,255,0),false);
					ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(), mnode->GetPos()+(VECT_Z*mnode->p_strain.ZZ()* strain_scale), video::SColor(255,0,0,255),false);
					*/

					/*
					double stress_scale =0.008;
					ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(), mnode->GetPos()+(VECT_X*mnode->e_stress.XX()* stress_scale), video::SColor(100,255,0,0),false);
					ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(), mnode->GetPos()+(VECT_Y*mnode->e_stress.YY()* stress_scale), video::SColor(100,0,255,0),false);
					ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(), mnode->GetPos()+(VECT_Z*mnode->e_stress.ZZ()* stress_scale), video::SColor(100,0,0,255),false);
					*/

					//ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(), mnode->GetPos()+(mnode->UserForce * 0.1), video::SColor(100,0,0,0),false);
					
				}
			}
			++myiter;
		}

		

		application.DoStep();
		
		application.GetVideoDriver()->endScene();  
	}
	
 
 
	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}
  
