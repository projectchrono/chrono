///////////////////////////////////////////////////
//      
//   Demo code about   
//   
//     - collisions and contacts 
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
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
    
  
 
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "irrlicht_interface/ChBodySceneNode.h"
#include "irrlicht_interface/ChBodySceneNodeTools.h" 
#include "irrlicht_interface/ChIrrAppInterface.h"
#include "core/ChRealtimeStep.h"
#include "lcp/ChLcpIterativeMINRES.h" // test
#include "physics/ChMaterialSurface.h"


 
// Use the namespace of Chrono

using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;



void create_some_falling_items(ChSystem& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver)
{

	ChBodySceneNode* mrigidBody; 

	ChSharedPtr<ChMaterialSurface> mmaterial(new ChMaterialSurface);
	mmaterial->SetFriction(0.4f);
	mmaterial->SetCompliance (0.00005f);
	mmaterial->SetComplianceT(0.00005f);
	mmaterial->SetDampingF(0.2);

	// Create a bunch of ChronoENGINE rigid bodies (spheres and
	// boxes) which will fall..
	// Bodies are Irrlicht nodes of the special class ChBodySceneNode, 
	// which encapsulates ChBody items).  
	
	video::ITexture* cubeMap   = driver->getTexture("../data/cubetexture_borders.png");
	video::ITexture* sphereMap = driver->getTexture("../data/bluwhite.png");
		

	for (int ai = 0; ai < 1; ai++)  // N. of walls
	{ 
		for (int bi = 0; bi < 10; bi++)  // N. of vert. bricks
		{ 
			for (int ui = 0; ui < 15; ui++)  // N. of hor. bricks
			{ 
				mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
													&mphysicalSystem, msceneManager,
													0.8,
													ChVector<>(-8+ui*4.0+2*(bi%2),  1.0+bi*2.0, ai*9),
													ChQuaternion<>(1,0,0,0), 
													ChVector<>(3.96,2,4) );
				mrigidBody->GetBody()->SetMaterialSurface(mmaterial);
				mrigidBody->setMaterialTexture(0,	cubeMap);
				mrigidBody->addShadowVolumeSceneNode();
			}
		}
	}


	// Jenga tower
	/*
	for (int bi = 0; bi < 12; bi+=2) 
	{ 
		ChBodySceneNode* mrigidBody1;
		mrigidBody1 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											0.2,
											ChVector<>(-5, 1.0+bi*2.0,  0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(2,2, 14) );
		mrigidBody1->GetBody()->SetMaterialSurface(mmaterial);
		mrigidBody1->setMaterialTexture(0,	cubeMap);

		ChBodySceneNode* mrigidBody2;
		mrigidBody2 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											0.2,
											ChVector<>( 5, 1.0+bi*2.0,  0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(2,2, 14) );
		mrigidBody2->GetBody()->SetMaterialSurface(mmaterial);
		mrigidBody2->setMaterialTexture(0,	cubeMap);

		ChBodySceneNode* mrigidBody3;
		mrigidBody3 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											0.2,
											ChVector<>(0, 3.0+bi*2.0,  5),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(14,2, 2) );
		mrigidBody3->GetBody()->SetMaterialSurface(mmaterial);
		mrigidBody3->setMaterialTexture(0,	cubeMap);

		ChBodySceneNode* mrigidBody4;
		mrigidBody4 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											0.2,
											ChVector<>(0, 3.0+bi*2.0,  -5),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(14,2, 2) );
		mrigidBody4->GetBody()->SetMaterialSurface(mmaterial);
		mrigidBody4->setMaterialTexture(0,	cubeMap);
	}
	*/

	// Create the floor using
	// fixed rigid body of 'box' type:

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											200.0,
											ChVector<>(0,-2,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(550,4,550) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->GetBody()->SetMaterialSurface(mmaterial);


	// Create a ball that will collide with wall
	double mradius = 4;
	double density = 1.01;
	double mmass = (4./3.)*CH_C_PI*pow(mradius,3)*density; 
	GetLog() << "Ball mass = " << mmass << "\n";
	double minert = (2./5.)* mmass * pow(mradius,2);

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easySphere(
										&mphysicalSystem, msceneManager,
										mmass, // mass
										ChVector<>(0, 3, -8), // pos
										mradius, // radius
										20,  // hslices, for rendering
										15); // vslices, for rendering

	// set moment of inertia (more realistic than default 1,1,1).
	mrigidBody->GetBody()->SetInertiaXX(ChVector<>(minert,minert,minert));
	mrigidBody->GetBody()->SetPos_dt(ChVector<>(0,0,16));
	mrigidBody->GetBody()->GetMaterialSurface()->SetFriction(0.4f);
	mrigidBody->GetBody()->GetMaterialSurface()->SetCompliance(0.0);
	mrigidBody->GetBody()->GetMaterialSurface()->SetComplianceT(0.0);
	mrigidBody->GetBody()->GetMaterialSurface()->SetDampingF(0.2);

	// Some aesthetics for 3d view..
	mrigidBody->addShadowVolumeSceneNode();
	mrigidBody->setMaterialTexture(0,	sphereMap);

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
	ChIrrAppInterface application(&mphysicalSystem, L"Bricks test",core::dimension2d<u32>(800,600),false, true); 

 
	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo  (application.GetDevice());
	ChIrrWizard::add_typical_Sky   (application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice(), core::vector3df(70.f, 120.f, -90.f), core::vector3df(30.f, 80.f, 60.f), 590,  400);
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(-15,14,-30), core::vector3df(0,5,0)); 

	// 
	// HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO... 
	// 

 
	// Create all the rigid bodies.
	create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver());
  
  
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
  
