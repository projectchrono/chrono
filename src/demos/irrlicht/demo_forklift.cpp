///////////////////////////////////////////////////
//
//   Demo code about 
//
//     - modeling a complex mechanism:  a forklift
//     - loading .obj 3D meshes for 3d viewing
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
#include "physics/ChLinkLinActuator.h"
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



// First of all, define a class for the 'forklift' (that is, a set of
// bodies and links which are grouped within this class; so it is 
// easier to manage data structures in this example).

class MySimpleForklift {
public:
		// THE DATA

	double throttle; // actual value 0...1 of gas throttle.
	double steer;	 // actual value of steering
	double lift;	 // actual value of fork lifting
	

		// The parts making the forklift, as 3d Irrlicht scene nodes, each containing
		// the ChBody object
			// .. truss:
	ChBodySceneNode* truss;
			// .. right front wheel:
	ChBodySceneNode* wheelRF;
	ChSharedPtr<ChLinkLockRevolute> link_revoluteRF;
			// .. left front wheel:
	ChBodySceneNode* wheelLF;
	ChSharedPtr<ChLinkLockRevolute> link_revoluteLF;
			// .. back wheel:
	ChBodySceneNode* spindleB;
	ChBodySceneNode* wheelB;
	ChSharedPtr<ChLinkEngine> link_steer_engineB;
	ChSharedPtr<ChLinkEngine>       link_engineB;
			// ..the vertical arm
	ChBodySceneNode* arm;
	ChSharedPtr<ChLinkEngine>   link_engineArm;
			// ..the fork
	ChBodySceneNode* fork;
	ChSharedPtr<ChLinkLinActuator>   link_actuatorFork;
	ChSharedPtr<ChLinkLockPrismatic> link_prismaticFork;

	video::ITexture* forkliftTiremap;


		// THE FUNCTIONS

		// Build and initialize the forklift, creating all bodies corresponding to
		// the various parts and adding them to the physical system - also creating
		// and adding constraints to the system.
	MySimpleForklift(ChIrrAppInterface* app, ChVector<> offset = ChVector<>(0,0,0))
			{
				throttle = 0; // initially, gas throttle is 0.
				steer = 0;
				lift =0;

				ChVector<> COG_truss(0, 0.4, 0.5);
				ChVector<> COG_wheelRF( 0.566, 0.282, 1.608);
				ChVector<> COG_wheelLF(-0.566, 0.282, 1.608);
				ChVector<> COG_arm(0, 1.300, 1.855);
				ChVector<> COG_fork(0, 0.362, 2.100);
				ChVector<> COG_wheelB(0, 0.282, 0.003);
				ChVector<> POS_pivotarm(0, 0.150, 1.855);
				ChVector<> POS_prismatic(0, 0.150, 1.855);
				double RAD_back_wheel =0.28;
				double RAD_front_wheel = 0.28;

				forkliftTiremap = app->GetVideoDriver()->getTexture("../data/tire_truck.png");
				

				// --- The car body --- 

				IAnimatedMesh*	forklift_bodyMesh = app->GetSceneManager()->getMesh("../data/forklift_body.obj");
				 truss = (ChBodySceneNode*)addChBodySceneNode(
														app->GetSystem(), app->GetSceneManager(), forklift_bodyMesh,
														200.0,
														COG_truss,
														QUNIT);
				truss->GetBody()->SetInertiaXX(ChVector<>(100, 100, 100));
				truss->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
				truss->setMaterialTexture(0,	forkliftTiremap);
				truss->GetChildMesh()->setPosition(-vector3df(COG_truss.x, COG_truss.y, COG_truss.z));// offset the mesh
				truss->addShadowVolumeSceneNode();
				truss->GetBody()->GetCollisionModel()->ClearModel();
				truss->GetBody()->GetCollisionModel()->AddBox(1.227/2., 1.621/2., 1.864/2., &ChVector<>(-0.003, 1.019, 0.192));
				truss->GetBody()->GetCollisionModel()->AddBox(0.187/2., 0.773/2., 1.201/2., &ChVector<>( 0.486 , 0.153,-0.047));
				truss->GetBody()->GetCollisionModel()->AddBox(0.187/2., 0.773/2., 1.201/2., &ChVector<>(-0.486 , 0.153,-0.047));
				truss->GetBody()->GetCollisionModel()->BuildModel();
				truss->GetBody()->SetCollide(true);

				// ..the right-front wheel
				IAnimatedMesh*	forklift_wheelMesh = app->GetSceneManager()->getMesh("../data/wheel.obj");
				wheelRF = (ChBodySceneNode*)addChBodySceneNode(
														app->GetSystem(), app->GetSceneManager(), forklift_wheelMesh,
														20.0,
														COG_wheelRF,
														QUNIT);
				wheelRF->GetBody()->SetInertiaXX(ChVector<>(2, 2, 2));
				wheelRF->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
				wheelRF->setMaterialTexture(0,	forkliftTiremap);
				wheelRF->GetChildMesh()->setPosition(-vector3df(COG_wheelRF.x, COG_wheelRF.y, COG_wheelRF.z));// offset the mesh
				// Describe the (invisible) colliding shape
				ChMatrix33<>Arot(chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Z));
				wheelRF->GetBody()->GetCollisionModel()->ClearModel();
				wheelRF->GetBody()->GetCollisionModel()->AddCylinder(RAD_front_wheel,RAD_front_wheel, 0.1, &ChVector<>(0,0,0), &Arot); 
				wheelRF->GetBody()->GetCollisionModel()->BuildModel();
				wheelRF->GetBody()->SetCollide(true);

				// .. create the revolute joint between the wheel and the truss
				link_revoluteRF = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); // right, front, upper, 1
				link_revoluteRF->Initialize(wheelRF->GetBody(), truss->GetBody(), 
					ChCoordsys<>(COG_wheelRF , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
				app->GetSystem()->AddLink(link_revoluteRF);


		
				// ..the left-front wheel
				wheelLF = (ChBodySceneNode*)addChBodySceneNode(
														app->GetSystem(), app->GetSceneManager(), forklift_wheelMesh,
														20.0,
														COG_wheelLF,
														chrono::Q_from_AngAxis(CH_C_PI, VECT_Y)); // reuse RF wheel shape, flipped
				wheelLF->GetBody()->SetInertiaXX(ChVector<>(2, 2, 2));
				wheelLF->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
				wheelLF->setMaterialTexture(0,	forkliftTiremap);
				wheelLF->GetChildMesh()->setPosition(-vector3df(COG_wheelRF.x, COG_wheelRF.y, COG_wheelRF.z));// offset the mesh (reuse RF)
				// Describe the (invisible) colliding shape
				wheelLF->GetBody()->GetCollisionModel()->ClearModel();
				wheelLF->GetBody()->GetCollisionModel()->AddCylinder(RAD_front_wheel,RAD_front_wheel, 0.1, &ChVector<>(0,0,0), &Arot); 
				wheelLF->GetBody()->GetCollisionModel()->BuildModel();
				wheelLF->GetBody()->SetCollide(true);

				// .. create the revolute joint between the wheel and the truss
				link_revoluteLF = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); // right, front, upper, 1
				link_revoluteLF->Initialize(wheelLF->GetBody(), truss->GetBody(), 
					ChCoordsys<>(COG_wheelLF , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
				app->GetSystem()->AddLink(link_revoluteLF);



						
				// ..the back steering spindle (invisible, no mesh)
				spindleB = (ChBodySceneNode*)addChBodySceneNode(
														app->GetSystem(), app->GetSceneManager(), 0,
														10.0,
														COG_wheelB,
														QUNIT); 
				spindleB->GetBody()->SetInertiaXX(ChVector<>(1, 1, 1));
				spindleB->GetBody()->SetCollide(false);

				// .. create the vertical steering link between the spindle structure and the truss
				link_steer_engineB = ChSharedPtr<ChLinkEngine>(new ChLinkEngine); 
				link_steer_engineB->Initialize(spindleB->GetBody(), truss->GetBody(), 
					ChCoordsys<>(COG_wheelB , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_X)) ); // vertical axis
				link_steer_engineB->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK); 
				link_steer_engineB->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
				app->GetSystem()->AddLink(link_steer_engineB);



				// ..the back wheel
				wheelB = (ChBodySceneNode*)addChBodySceneNode(
														app->GetSystem(), app->GetSceneManager(), forklift_wheelMesh,
														20.0,
														COG_wheelB,
														QUNIT); 
				wheelB->GetBody()->SetInertiaXX(ChVector<>(2, 2, 2));
				wheelB->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
				wheelB->setMaterialTexture(0,	forkliftTiremap);
				wheelB->GetChildMesh()->setPosition(-vector3df(COG_wheelRF.x, COG_wheelRF.y, COG_wheelRF.z));// offset the mesh (reuse RF wheel)
				double rescale = RAD_back_wheel/RAD_front_wheel;
				// Describe the (invisible) colliding shape
				wheelB->GetBody()->GetCollisionModel()->ClearModel();
				wheelB->GetBody()->GetCollisionModel()->AddCylinder(RAD_back_wheel,RAD_back_wheel, 0.1, &ChVector<>(0,0,0), &Arot); 
				wheelB->GetBody()->GetCollisionModel()->BuildModel();
				wheelB->GetBody()->SetCollide(true);

				// .. create the motor between the back wheel and the steering spindle structure
				link_engineB = ChSharedPtr<ChLinkEngine>(new ChLinkEngine); 
				link_engineB->Initialize(wheelB->GetBody(), spindleB->GetBody(), 
					ChCoordsys<>(COG_wheelB , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
				link_engineB->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK); 
				link_engineB->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
				app->GetSystem()->AddLink(link_engineB);


				// ..the arm
				IAnimatedMesh*	forklift_armMesh = app->GetSceneManager()->getMesh("../data/forklift_arm.obj");
				arm = (ChBodySceneNode*)addChBodySceneNode(
														app->GetSystem(), app->GetSceneManager(), forklift_armMesh,
														100.0,
														COG_arm,
														QUNIT);
				arm->GetBody()->SetInertiaXX(ChVector<>(30, 30, 30));
				arm->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
				arm->GetChildMesh()->setPosition(-vector3df(COG_arm.x, COG_arm.y, COG_arm.z));// offset the mesh

				// .. create the revolute joint between the arm and the truss
				link_engineArm = ChSharedPtr<ChLinkEngine>(new ChLinkEngine); // right, front, upper, 1
				link_engineArm->Initialize(arm->GetBody(), truss->GetBody(), 
					ChCoordsys<>(POS_pivotarm , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
				link_engineArm->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK); 
				link_engineArm->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
				app->GetSystem()->AddLink(link_engineArm);



				// ..the fork
				IAnimatedMesh*	forklift_forkMesh = app->GetSceneManager()->getMesh("../data/forklift_forks.obj");
				fork = (ChBodySceneNode*)addChBodySceneNode(
														app->GetSystem(), app->GetSceneManager(), forklift_forkMesh,
														60.0,
														COG_fork,
														QUNIT);
				fork->GetBody()->SetInertiaXX(ChVector<>(15, 15, 15));
				//fork->GetBody()->SetCollide(false);
				fork->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
				//fork->setMaterialTexture(0,	forkliftTiremap);
				fork->GetChildMesh()->setPosition(-vector3df(COG_fork.x, COG_fork.y, COG_fork.z));// offset the mesh
				// Describe the (invisible) colliding shapes - two cubes for the two fingers, etc
				fork->GetBody()->GetCollisionModel()->ClearModel();
				fork->GetBody()->GetCollisionModel()->AddBox(0.1/2., 0.032/2., 1.033/2., &ChVector<>(-0.352, -0.312, 0.613)); 
				fork->GetBody()->GetCollisionModel()->AddBox(0.1/2., 0.032/2., 1.033/2., &ChVector<>( 0.352, -0.312, 0.613)); 
				fork->GetBody()->GetCollisionModel()->AddBox(0.344/2., 1.134/2., 0.101/2., &ChVector<>(-0.000, 0.321, -0.009));
				fork->GetBody()->GetCollisionModel()->BuildModel();
				fork->GetBody()->SetCollide(true);

				// .. create the revolute joint between the wheel and the truss
				link_prismaticFork = ChSharedPtr<ChLinkLockPrismatic>(new ChLinkLockPrismatic); 
				link_prismaticFork->Initialize(fork->GetBody(), arm->GetBody(), 
					ChCoordsys<>(POS_prismatic , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_X)) ); // set prism as vertical (default would be aligned to z, horizontal
				app->GetSystem()->AddLink(link_prismaticFork);

				// .. create the linear actuator that pushes upward the fork
				link_actuatorFork = ChSharedPtr<ChLinkLinActuator>(new ChLinkLinActuator); 
				link_actuatorFork->Initialize(fork->GetBody(), arm->GetBody(), false,
					ChCoordsys<>(POS_prismatic+ChVector<>(0,0.01,0) , QUNIT),
					ChCoordsys<>(POS_prismatic                     , QUNIT) ); 
				app->GetSystem()->AddLink(link_actuatorFork);


				// ..a pallet

				IAnimatedMesh*	palletMesh = app->GetSceneManager()->getMesh("../data/pallet.obj");
				video::ITexture* palletmap = app->GetVideoDriver()->getTexture("../data/cubetexture.png");

				ChBodySceneNode* pallet = (ChBodySceneNode*)addChBodySceneNode_easyConcaveMesh(
														app->GetSystem(), app->GetSceneManager(), "../data/pallet.obj",
														15.0,
														ChVector<>(0,1,3),
														QUNIT);
				pallet->GetBody()->SetInertiaXX(ChVector<>(3, 3, 3));
				pallet->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
				pallet->setMaterialTexture(0,	palletmap);


				//
				// Move the forklift to initial offset position
				//

//				pallet->GetBody()->Move(offset);
				truss->GetBody()->Move(offset);
				wheelRF->GetBody()->Move(offset);
				wheelLF->GetBody()->Move(offset);
				wheelB->GetBody()->Move(offset);
				spindleB->GetBody()->Move(offset);
				arm->GetBody()->Move(offset);
				fork->GetBody()->Move(offset);
			}

		// Delete the car object, deleting also all bodies corresponding to
		// the various parts and removing them from the physical system.  Also
		// removes constraints from the system.
	~MySimpleForklift()
			{
				ChSystem* mysystem = truss->GetBody()->GetSystem(); // trick to get the system here				
					// When a ChBodySceneNode is removed via ->remove() from Irrlicht 3D scene manager,
					// it is also automatically removed from the ChSystem (the ChSystem::RemoveBody() is
					// automatically called at Irrlicht node deletion - see ChBodySceneNode.h ).

					// For links, just remove them from the ChSystem using ChSystem::RemoveLink()
				mysystem->RemoveLink(link_revoluteRF);	
				mysystem->RemoveLink(link_revoluteLF);				
				mysystem->RemoveLink(link_steer_engineB);				
				mysystem->RemoveLink(link_engineB);
				mysystem->RemoveLink(link_engineArm);
				mysystem->RemoveLink(link_prismaticFork);
				mysystem->RemoveLink(link_actuatorFork);

				truss->remove();
				wheelRF->remove();
				wheelLF->remove();
				wheelB->remove();
				spindleB->remove();
				arm->remove();
				fork->remove();
			}

};



// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface (the interface will
// be created with the basic -yet flexible- platform
// independent toolset of Irrlicht).

class MyEventReceiver : public IEventReceiver
{
public:

	MyEventReceiver(ChIrrAppInterface* myapp, MySimpleForklift* mlift)
	{
		// store pointer to physical system & other stuff so we can tweak them by user keyboard
		app = myapp;
		forklift = mlift;
	}

	bool OnEvent(const SEvent& event)
	{

		// check if user presses keys
		if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown)
		{
			switch (event.KeyInput.Key)
			{
			case irr::KEY_KEY_Q: 
				if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(forklift->link_steer_engineB->Get_rot_funct())) 
					mfun->Set_yconst(-0.6+mfun->Get_yconst());
				return true;
			case irr::KEY_KEY_W: 
				if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(forklift->link_steer_engineB->Get_rot_funct())) 
					mfun->Set_yconst(+0.3+mfun->Get_yconst());
				return true;
			case irr::KEY_KEY_A: 
				if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(forklift->link_engineB->Get_spe_funct())) 
					mfun->Set_yconst(0.5+mfun->Get_yconst());
				return true;
			case irr::KEY_KEY_Z: 
				if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(forklift->link_engineB->Get_spe_funct()))
					mfun->Set_yconst(-0.5+mfun->Get_yconst());
				return true;
			case irr::KEY_KEY_S: 
				if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(forklift->link_actuatorFork->Get_dist_funct()))
					mfun->Set_yconst(0.05+mfun->Get_yconst());
				return true;
			case irr::KEY_KEY_X: 
				if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(forklift->link_actuatorFork->Get_dist_funct()))
					mfun->Set_yconst(-0.05+mfun->Get_yconst());
				return true;
			case irr::KEY_KEY_D: 
				if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(forklift->link_engineArm->Get_rot_funct()))
					mfun->Set_yconst(0.005+mfun->Get_yconst());
				return true;
			case irr::KEY_KEY_C: 
				if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(forklift->link_engineArm->Get_rot_funct()))
					mfun->Set_yconst(-0.005+mfun->Get_yconst());
				return true;

			}
		}  

		return false;
	}

private: 
	ChIrrAppInterface* app;
	MySimpleForklift* forklift;
};



//
// This is the program which is executed
//

int main(int argc, char* argv[])
{

	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed.
	DLL_CreateGlobals();

	// Create a ChronoENGINE physical system
	ChSystem my_system;


	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrAppInterface application(&my_system, L"Drive a forklift",core::dimension2d<u32>(800,600),false); 

	// add text with info
	IGUIStaticText* textFPS = application.GetIGUIEnvironment()->addStaticText(L"Keys: steer=Q,W; throttle=A,Z; lift=S,X; bank=D,C", rect<s32>(150,10,430,40), true);

	// Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(-6,3,-6));

 
	 // ..the world
	ChBodySceneNode* my_ground = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&my_system, application.GetSceneManager(),
											1.0,
											ChVector<>(0,-1,0),
											QUNIT, 
											ChVector<>(40,2,40) );
	my_ground->GetBody()->SetBodyFixed(true);
	my_ground->GetBody()->SetCollide(true);
	my_ground->GetBody()->SetSfriction(1.0);
	my_ground->GetBody()->SetKfriction(1.0);
	video::ITexture* groundMap = application.GetVideoDriver()->getTexture("../data/concrete.jpg");
	my_ground->setMaterialTexture(0,groundMap);


	// ..some obstacles on the ground:
	video::ITexture* cubesmap = application.GetVideoDriver()->getTexture("../data/cubetexture.png");
	for (int i=0; i<6; i++)
	{
		ChBodySceneNode* my_obstacle = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&my_system, application.GetSceneManager(),
											3.0,
											ChVector<>(20*ChRandom(),2, 20*ChRandom()),
											QUNIT, 
											ChVector<>(1,0.5,1) );
		my_obstacle->setMaterialTexture(0,	cubesmap);
	}

	// ..the forklift (this class - see above - is a 'set' of bodies and links, automatically added at creation)
	MySimpleForklift* myforklift = new MySimpleForklift(&application);


	//
	// USER INTERFACE
	//
	 
	// Create some graphical-user-interface (GUI) items to show on the screen.
	// This requires an event receiver object -see above.
	// This is for GUI tweaking of system parameters..
	MyEventReceiver receiver(&application, myforklift);
	  // note how to add a custom event receiver to the default interface:
	application.SetUserEventReceiver(&receiver);



	//
	// SETTINGS 
	// 	

	my_system.SetIterLCPmaxItersSpeed(20); // the higher, the easier to keep the constraints 'mounted'.

	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD);  


	//
	// THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
	//


	// This will help choosing an integration step which matches the
	// real-time step of the simulation..
	ChRealtimeStepTimer m_realtime_timer;

	while(application.GetDevice()->run())
	{ 
		// Irrlicht must prepare frame to draw
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		// Irrlicht application draws all 3D objects and all GUI items
		application.DrawAll();

		// Advance the simulation time step
		//my_system.DoStepDynamics( m_realtime_timer.SuggestSimulationStep(0.005) );
		my_system.DoStepDynamics( 0.005 );

		// Irrlicht must finish drawing the frame
		application.GetVideoDriver()->endScene();

	}

	if (myforklift) delete myforklift;	


	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}


