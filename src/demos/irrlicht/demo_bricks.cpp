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

///////////////////////////////////////////////////
//      
//   Demo code about   
//   
//     - collisions and contacts 
//     - sharing a ChMaterialSurface property between bodies
//
//       (This is just a possible method of integration
//       of Chrono::Engine + Irrlicht: many others 
//       are possible.)
//     
//	 CHRONO   
//   ------
//   Multibody dinamics engine 
//   
// ------------------------------------------------ 
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
    
  
 
#include "physics/ChSystem.h"
#include "physics/ChBodyEasy.h"
#include "physics/ChMaterialSurface.h"
#include "assets/ChTexture.h"
#include "unit_IRRLICHT/ChIrrApp.h"









 
// Use the namespace of Chrono

using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;



	// Create a bunch of ChronoENGINE rigid bodies that
	// represent bricks in a large wall.

void create_wall_bodies(ChSystem& mphysicalSystem)
{
	// Create a material that will be shared between bricks
	ChSharedPtr<ChMaterialSurface> mmaterial(new ChMaterialSurface);
	mmaterial->SetFriction(0.4f);
	mmaterial->SetCompliance (0.0000005f);
	mmaterial->SetComplianceT(0.0000005f);
	mmaterial->SetDampingF(0.2f);
		
	// Create bricks
	for (int ai = 0; ai < 1; ai++)  // N. of walls
	{ 
		for (int bi = 0; bi < 10; bi++)  // N. of vert. bricks
		{ 
			for (int ui = 0; ui < 15; ui++)  // N. of hor. bricks
			{ 
				ChSharedPtr<ChBodyEasyBox> mrigidBody(new ChBodyEasyBox(
											3.96, 2, 4, // x,y,z size
											 100,		// density
											true,		// collide enable?
											true));		// visualization?
				mrigidBody->SetPos(ChVector<>(-8+ui*4.0+2*(bi%2),  1.0+bi*2.0, ai*9));
				mrigidBody->SetMaterialSurface(mmaterial); // use shared surface properties
				
				mphysicalSystem.Add(mrigidBody);
				
				// optional, attach a texture for better visualization
				ChSharedPtr<ChTexture> mtexture(new ChTexture());
				mtexture->SetTextureFilename(GetChronoDataFile("cubetexture_borders.png"));
				mrigidBody->AddAsset(mtexture);
			}
		}
	}

	// Create the floor using
	// fixed rigid body of 'box' type:

	ChSharedPtr<ChBodyEasyBox> mrigidFloor(new ChBodyEasyBox(
											250, 4, 250, // x,y,z size
											1000,		// density
											true,		// collide enable?
											true));		// visualization?
	mrigidFloor->SetPos(ChVector<>(0,-2,0));
	mrigidFloor->SetMaterialSurface(mmaterial);
	mrigidFloor->SetBodyFixed(true);
	
	mphysicalSystem.Add(mrigidFloor);


	
	// Create a ball that will collide with wall
	ChSharedPtr<ChBodyEasySphere> mrigidBall(new ChBodyEasySphere(
											4,			// radius
											8000,		// density
											true,		// collide enable?
											true));		// visualization?
	mrigidBall->SetPos(ChVector<>(0,-2,0));
	mrigidBall->SetMaterialSurface(mmaterial);
	mrigidBall->SetPos(ChVector<>(0, 3, -8));
	mrigidBall->SetPos_dt(ChVector<>(0,0,16)); // set initial speed 
	mrigidBall->GetMaterialSurface()->SetFriction(0.4f); // use own (not shared) matrial properties
	mrigidBall->GetMaterialSurface()->SetCompliance(0.0);
	mrigidBall->GetMaterialSurface()->SetComplianceT(0.0);
	mrigidBall->GetMaterialSurface()->SetDampingF(0.2f);

	mphysicalSystem.Add(mrigidBall);

	// optional, attach a texture for better visualization
	ChSharedPtr<ChTexture> mtextureball(new ChTexture());
	mtextureball->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
	mrigidBall->AddAsset(mtextureball);


}


// Create a bunch of ChronoENGINE rigid bodies that
// represent bricks in a Jenga tower

void create_jengatower_bodies(ChSystem& mphysicalSystem)
{
	// Create a material that will be shared between bricks
	ChSharedPtr<ChMaterialSurface> mmaterial(new ChMaterialSurface);
	mmaterial->SetFriction(0.4f);
	mmaterial->SetCompliance (0.0000005f);
	mmaterial->SetComplianceT(0.0000005f);
	mmaterial->SetDampingF(0.2f);

	// Create bricks
	for (int bi = 0; bi < 12; bi+=2) 
	{ 
		ChSharedPtr<ChBodyEasyBox> mrigidBody1(new ChBodyEasyBox(
											2, 2, 14,   // x,y,z size
											 100,		// density
											true,		// collide enable?
											true));		// visualization?
		mrigidBody1->SetPos( ChVector<>(-5, 1.0+bi*2.0,  0) );
		mrigidBody1->SetMaterialSurface(mmaterial); // use shared surface properties	
		mphysicalSystem.Add(mrigidBody1);


		ChSharedPtr<ChBodyEasyBox> mrigidBody2(new ChBodyEasyBox(
											2, 2, 14,   // x,y,z size
											 100,		// density
											true,		// collide enable?
											true));		// visualization?
		mrigidBody2->SetPos( ChVector<>( 5, 1.0+bi*2.0,  0) );
		mrigidBody2->SetMaterialSurface(mmaterial); // use shared surface properties	
		mphysicalSystem.Add(mrigidBody2);


		ChSharedPtr<ChBodyEasyBox> mrigidBody3(new ChBodyEasyBox(
											14, 2, 2,   // x,y,z size
											 100,		// density
											true,		// collide enable?
											true));		// visualization?
		mrigidBody3->SetPos( ChVector<>(0, 3.0+bi*2.0,  5) );
		mrigidBody3->SetMaterialSurface(mmaterial); // use shared surface properties	
		mphysicalSystem.Add(mrigidBody3);


		ChSharedPtr<ChBodyEasyBox> mrigidBody4(new ChBodyEasyBox(
											14, 2, 2,   // x,y,z size
											 100,		// density
											true,		// collide enable?
											true));		// visualization?
		mrigidBody4->SetPos( ChVector<>(0, 3.0+bi*2.0,  -5) );
		mrigidBody4->SetMaterialSurface(mmaterial); // use shared surface properties	
		mphysicalSystem.Add(mrigidBody4);
	}

	// Create the floor using
	// fixed rigid body of 'box' type:

	ChSharedPtr<ChBodyEasyBox> mrigidFloor(new ChBodyEasyBox(
											250, 4, 250, // x,y,z size
											1000,		// density
											true,		// collide enable?
											true));		// visualization?
	mrigidFloor->SetPos(ChVector<>(0,-2,0));
	mrigidFloor->SetMaterialSurface(mmaterial);
	mrigidFloor->SetBodyFixed(true);
	
	mphysicalSystem.Add(mrigidFloor);


	
	// Create a ball that will collide with tower
	ChSharedPtr<ChBodyEasySphere> mrigidBall(new ChBodyEasySphere(
											4,			// radius
											1000,		// density
											true,		// collide enable?
											true));		// visualization?
	mrigidBall->SetMaterialSurface(mmaterial);
	mrigidBall->SetPos(ChVector<>(0, 3, -8));
	mrigidBall->SetPos_dt(ChVector<>(0,0,2)); // set initial speed 
	mrigidBall->GetMaterialSurface()->SetFriction(0.4f); // use own (not shared) matrial properties
	mrigidBall->GetMaterialSurface()->SetCompliance(0.0);
	mrigidBall->GetMaterialSurface()->SetComplianceT(0.0);
	mrigidBall->GetMaterialSurface()->SetDampingF(0.2f);

	mphysicalSystem.Add(mrigidBall);

	// optional, attach a texture for better visualization
	ChSharedPtr<ChTexture> mtextureball(new ChTexture());
  mtextureball->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
	mrigidBall->AddAsset(mtextureball);


}
 

 
 
int main(int argc, char* argv[])
{ 
	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem; 

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&mphysicalSystem, L"Bricks test",core::dimension2d<u32>(800,600),false, true); 

 
	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo  (application.GetDevice());
	ChIrrWizard::add_typical_Sky   (application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice(), core::vector3df(70.f, 120.f, -90.f), core::vector3df(30.f, 80.f, 60.f), 290,  190);
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(-15,14,-30), core::vector3df(0,5,0)); 

	// 
	// HERE YOU POPULATE THE MECHANICAL SYSTEM OF CHRONO... 
	// 

 
	// Create all the rigid bodies.
    create_wall_bodies (mphysicalSystem);
	//create_jengatower_bodies (mphysicalSystem);


	// Use this function for adding a ChIrrNodeAsset to all items
	// If you need a finer control on which item really needs a visualization proxy in 
	// Irrlicht, just use application.AssetBind(myitem); on a per-item basis.
	application.AssetBindAll();

	// Use this function for 'converting' into Irrlicht meshes the assets 
	// into Irrlicht-visualizable meshes
	application.AssetUpdateAll();


  
	// Prepare the physical system for the simulation 

	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD);

	mphysicalSystem.SetUseSleeping(false);

	mphysicalSystem.SetMaxPenetrationRecoverySpeed(1.6); // used by Anitescu stepper only
	mphysicalSystem.SetIterLCPmaxItersSpeed(40);
	mphysicalSystem.SetIterLCPmaxItersStab(20); // unuseful for Anitescu, only Tasora uses this
	mphysicalSystem.SetIterLCPwarmStarting(true);
	mphysicalSystem.SetParallelThreadNumber(4);

	//
	// THE SOFT-REAL-TIME CYCLE
	//
 
	application.SetStepManage(true);
	application.SetTimestep(0.02);

	while(application.GetDevice()->run())
	{
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		ChIrrTools::drawGrid(application.GetVideoDriver(), 5,5, 20,20, 
			ChCoordsys<>(ChVector<>(0,0.2,0),Q_from_AngAxis(CH_C_PI/2,VECT_X)), video::SColor(50,90,90,150),true);

		application.DrawAll();

		application.DoStep();
 
		application.GetVideoDriver()->endScene();  
	}
	 

	return 0;
}
  
