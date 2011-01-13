///////////////////////////////////////////////////
//
//   Demo code about 
//
//     - using rolling friction (not only sliding and
//       static friction, available in all objects by default)
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



void create_some_falling_items(ChSystem& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver)
{
	ChBodySceneNode* mrigidBody; 

	video::ITexture* sphereMapP = driver->getTexture("../data/pinkwhite.png");
	video::ITexture* sphereMapB = driver->getTexture("../data/bluwhite.png");

	double mradius = 0.5;
	double density = 1000;
	double mmass = (4./3.)*CH_C_PI*pow(mradius,3.)*density; 
	double minert = (2./5.)* mmass * pow(mradius,2.);

	// Create some spheres that roll horizontally, 
	// with increasing rolling friction values
	for (int bi = 0; bi < 10; bi++) 
	{    
		
		double initial_angspeed = 10;
		double initial_linspeed = initial_angspeed*mradius;

		mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easySphere(
											&mphysicalSystem, msceneManager,
											mmass, // mass
											ChVector<>(-7, mradius-0.5, -5+bi*mradius*2.5), // pos
											mradius, // radius
											20,  // hslices
											15); // vslices
    
		// set moment of inertia too (otherwise default is 1,1,1).
		mrigidBody->GetBody()->SetInertiaXX(ChVector<>(minert,minert,minert));
		// set initial velocity: rolling in horizontal direction
		mrigidBody->GetBody()->SetWvel_par(ChVector<>(0,0,-initial_angspeed));
		mrigidBody->GetBody()->SetPos_dt(ChVector<>(initial_linspeed,0,0));

		mrigidBody->GetBody()->SetFriction(0.4);

		// Set a non zero value of rolling friction to have a rolling resisting torque:
		mrigidBody->GetBody()->SetRollingFriction( ((double)bi/10.)*0.2 );

		// Some aesthetics for 3d view..
		mrigidBody->addShadowVolumeSceneNode();
		mrigidBody->setMaterialTexture(0,	sphereMapP);

	} 

	// Create some spheres that spin on place, for a 'drilling friction' case, 
	// with increasing spinning friction values
	for (int bi = 0; bi < 10; bi++) 
	{    
		mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easySphere(
											&mphysicalSystem, msceneManager,
											mmass, // mass
											ChVector<>(-8, mradius-0.5, -5+bi*mradius*2.5), // pos
											mradius, // radius
											20,  // hslices
											15); // vslices
    
		// set moment of inertia too (otherwise default is 1,1,1).
		mrigidBody->GetBody()->SetInertiaXX(ChVector<>(minert,minert,minert));
		// set initial speed: spinning in vertical direction
		mrigidBody->GetBody()->SetWvel_par(ChVector<>(0, 20,0));

		mrigidBody->GetBody()->SetFriction(0.4); 

		// Set a non zero value of spinning friction that brakes the spinning on vertical axis
		// of the contact: 
		mrigidBody->GetBody()->SetSpinningFriction( ((double)bi/10.)*0.1 );

		// Notes: 
		// - setting nonzero spinning frition and/or setting nonzero rolling friction 
		//   affects the speed of the solver (each contact eats 2x of CPU time repsect to the
		//   case of simple sliding/staic contact)
		// - avoid using zero spinning friction with nonzero rolling friction.

		// Some aesthetics for 3d view..
		mrigidBody->addShadowVolumeSceneNode();
		mrigidBody->setMaterialTexture(0,	sphereMapB);

	}


	// Create the five walls of the rectangular container, using
	// fixed rigid bodies of 'box' type:

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											1000.0,
											ChVector<>(0,-1,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(20,1,20) );
	mrigidBody->GetBody()->SetBodyFixed(true);

	video::ITexture* cubeMap = driver->getTexture("../data/blu.png");
	mrigidBody->setMaterialTexture(0,	cubeMap);


	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(-10,0,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(1,2,20.99) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);


	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(10,0,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(1,2,20.99) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(0,0,-10),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(20.99,2,1) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(0,0, 10),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(20.99,2,1) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);
 


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
	ChIrrAppInterface application(&mphysicalSystem, L"Contacts with rolling friction",core::dimension2d<u32>(800,600),false, true);


	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0,14,-20));

 
	// Create all the rigid bodies.
	create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver());
    

	// Modify some setting of the physical system for the simulation, if you want

	mphysicalSystem.SetIterLCPmaxItersSpeed(26);
 

 

	// 
	// THE SOFT-REAL-TIME CYCLE
	//

	while(application.GetDevice()->run()) 
	{
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		application.DrawAll();
		
		// ChIrrTools::drawAllContactLabels(mphysicalSystem, application.GetDevice());

		mphysicalSystem.DoStepDynamics( 0.01);
		

		application.GetVideoDriver()->endScene();  
	}
	
 
 
	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}
  
