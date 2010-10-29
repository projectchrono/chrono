///////////////////////////////////////////////////
//
//   Demo code about  
//
//     - advanced contact feature: cohesion
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
#include "lcp/ChLcpIterativeMINRES.h"

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


// Static values valid through the entire program (bad
// programming practice, but enough for quick tests)

double GLOBAL_friction = 0.3;
double GLOBAL_cohesion = 300;



// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface

class MyEventReceiver : public IEventReceiver
{
public:

	MyEventReceiver(ChIrrAppInterface* myapp)
			{
				// store pointer applicaiton
				application = myapp;

				// ..add a GUI slider to control friction 
				scrollbar_friction = application->GetIGUIEnvironment()->addScrollBar(
								true, rect<s32>(510, 85, 650, 100), 0, 101);
				scrollbar_friction->setMax(100); 
				scrollbar_friction->setPos(30);
				text_friction = application->GetIGUIEnvironment()->addStaticText(
								L"Friction coefficient:", rect<s32>(650,85,750,100), false);

				// ..add GUI slider to control the speed
				scrollbar_cohesion = application->GetIGUIEnvironment()->addScrollBar(
								true, rect<s32>(510, 125, 650, 140), 0, 102);
				scrollbar_cohesion->setMax(500); 
				scrollbar_cohesion->setPos(300);
				text_cohesion = application->GetIGUIEnvironment()->addStaticText(
								L"Cohesion [N]:", rect<s32>(650,125,750,140), false);
			}

	bool OnEvent(const SEvent& event)
			{

				// check if user moved the sliders with mouse..
				if (event.EventType == EET_GUI_EVENT)
				{
					s32 id = event.GUIEvent.Caller->getID();
					IGUIEnvironment* env = application->GetIGUIEnvironment();

					switch(event.GUIEvent.EventType)
					{
					case EGET_SCROLL_BAR_CHANGED:
							if (id == 101) // id of 'flow' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								GLOBAL_friction = (double)pos/100.0;
							}
							if (id == 102) // id of 'speed' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								GLOBAL_cohesion = ((double)pos);
							}
					break;
					}
					
				} 

				return false;
			}

private: 
	ChIrrAppInterface* application;

	IGUIScrollBar*  scrollbar_friction;
	IGUIStaticText* text_friction;
	IGUIScrollBar*  scrollbar_cohesion;
	IGUIStaticText* text_cohesion;
};


void create_some_falling_items(ChSystem& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver)
{
	ChBodySceneNode* mrigidBody; 
 

	for (int bi = 0; bi < 300; bi++) 
	{    
		// Create a bunch of ChronoENGINE rigid bodies (spheres and
		// boxes) which will fall..
		// Falling bodies are Irrlicht nodes of the special class ChBodySceneNode, 
		// which encapsulates ChBody items).  
		// Note that ChBodySceneNode have collision turned ON by default (so if you
		// want to have them 'transparent' to collision detection, you may use
		// mrigidBody->GetBody()->SetCollide(false) if you need..) 

  
		mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easySphere(
											&mphysicalSystem, msceneManager,
											1.0,
											ChVector<>(-5+ChRandom()*10, 4+bi*0.05, -5+ChRandom()*10),
											0.85);
   
		mrigidBody->GetBody()->SetFriction(0.3); 
		mrigidBody->addShadowVolumeSceneNode();

		video::ITexture* sphereMap = driver->getTexture("../data/pinkwhite.png");
		mrigidBody->setMaterialTexture(0,	sphereMap);

	} 


	// Create the five walls of the rectangular container, using
	// fixed rigid bodies of 'box' type:

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(0,-5,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(20,1,20) );
	mrigidBody->GetBody()->SetBodyFixed(true);

	video::ITexture* cubeMap = driver->getTexture("../data/concrete.jpg");
	mrigidBody->setMaterialTexture(0,	cubeMap);


	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(-10,0,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(1,10,20.99) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);


	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(10,0,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(1,10,20.99) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(0,0,-10),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(20.99,10,1) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(0,0, 10),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(20.99,10,1) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);
 

	// Add the rotating mixer 
	ChBodySceneNode* rotatingBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											1.0,
											ChVector<>(0,-1.6,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(10,5.5,1) ); 
	rotatingBody->GetBody()->SetMass(100);
	rotatingBody->GetBody()->SetInertiaXX(ChVector<>(100,100,100));
	rotatingBody->GetBody()->SetFriction(0.4);
	rotatingBody->addShadowVolumeSceneNode();

	// .. an engine between mixer and truss	
	ChSharedPtr<ChLinkEngine> my_motor(new ChLinkEngine);
	my_motor->Initialize(rotatingBody->GetBody(), mrigidBody->GetBody(), 
				ChCoordsys<>(ChVector<>(0,0,0),
							 Q_from_AngAxis(CH_C_PI_2, VECT_X)) );
	my_motor->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
	my_motor->Get_spe_funct()->Set_yconst(CH_C_PI/2.0); // speed w=90°/s
	mphysicalSystem.AddLink(my_motor);

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
	ChIrrAppInterface application(&mphysicalSystem, L"Contacts with cohesion",core::dimension2d<u32>(800,600),false);


	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0,14,-20));

 
	// Create all the rigid bodies.

	create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver());
 

	// This is for GUI tweaking of system parameters..
	MyEventReceiver receiver(&application);
	  // note how to add the custom event receiver to the default interface:
	application.SetUserEventReceiver(&receiver);


	// Modify some setting of the physical system for the simulation, if you want

	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD);
	mphysicalSystem.SetIterLCPmaxItersSpeed(20);
	//mphysicalSystem.SetIterLCPmaxItersStab(5);

/*
// The new solver - under testing -
mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_PMINRES);
ChLcpIterativeMINRES* msolver = (ChLcpIterativeMINRES*) mphysicalSystem.GetLcpSolverSpeed();
msolver->SetMaxIterations(40);
msolver->SetFeasTolerance(0.1);
msolver->SetOmega(0.05);
msolver->SetMaxFixedpointSteps(3);
*/

	// Differently from friction, that has always a default value that 
	// is computed as the average of two friction values of the two rigid 
	// bodies, the 'cohesion' has no body-specific setting (at least in 
	// this Chrono::Engine release) so it is always zero by default. 
	// Therefore it is up to the user to set it when each contact is created, 
	// by instancing a callback as in the following example:

	class MyContactCallback : public ChSystem::ChCustomCollisionPointCallback
	{
		public:	virtual void ContactCallback(
								const collision::ChCollisionInfo& mcontactinfo, ///< get info about contact (cannot change it)				
								ChMaterialCouple&  material )			  		///< you can modify this!	
		{
			// Set friction according to user setting:
			material.cohesion = GLOBAL_friction;

			// Set cohesion according to user setting:
			// Note that we must scale the cohesion force value by time step, because 
			// the material 'cohesion' value has the dimension of an impulse.
			double my_cohesion_force = GLOBAL_cohesion;
			material.cohesion = msystem->GetStep() * my_cohesion_force; //<- all contacts will have this cohesion!

			// Note that here you might decide to modify the cohesion 
			// depending on object sizes, type, time, position, etc. etc.
			// For example, after some time disable cohesion at all, just
			// add here:  
			//    if (msystem->GetChTime() > 10) material.cohesion = 0;

		};
		ChSystem* msystem;
	};

	MyContactCallback mycontact_callback;  // create the callback object
	mycontact_callback.msystem = &mphysicalSystem; // will be used by callback
	// Tell the system to use the callback above, per each created contact!
	mphysicalSystem.SetCustomCollisionPointCallback(&mycontact_callback);	


	// 
	// THE SOFT-REAL-TIME CYCLE
	//

	while(application.GetDevice()->run()) 
	{
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		application.DrawAll();

		mphysicalSystem.DoStepDynamics( 0.02);
		
		application.GetVideoDriver()->endScene();  
	}
	
 
 
	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}
  
