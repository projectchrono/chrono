///////////////////////////////////////////////////
//
// Vehicle full simulator
//
///////////////////////////////////////////////////
 
 
#include "physics/ChSystem.h"
#include "physics/ChLinkDistance.h"
#include "unit_IRRLICHT/ChBodySceneNode.h"
#include "unit_IRRLICHT/ChBodySceneNodeTools.h" 
#include "unit_IRRLICHT/ChDisplayTools.h" 
#include "unit_IRRLICHT/ChIrrWizard.h"
#include "core/ChRealtimeStep.h"

#include "vehicle_car.h"
#include "vehicle_gui.h"
#include "vehicle_simulator.h"

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





/// Creation of a 'MySimulator' object. Here a 3D Irrlicht view is
/// opened, the car object (containing Chrono::Engine bodies and constraints) is
/// instanced, etc.

MySimulator::MySimulator()
{
	// CREATE THE IRRLICHT CONTEXT (device, etc.)
	device = createDevice(video::EDT_DIRECT3D9, 
							core::dimension2d<u32>(640, 480),	// resolution
							32,									// 32 bit depth 
							false,								// full screen
							true,								// do shadows (might be slow on old PC!)
							true);								// vsync at 60Hz?
	if (device == 0)
	{
		GetLog() << "Cannot use DirectX - switch to OpenGL \n"; 
		device = createDevice(video::EDT_OPENGL, core::dimension2d<u32>(640, 480));
		if (!device) return;
	}

	device->setWindowCaption(L"Parma 43100 car simulator");

	GetLog() << "Press F1-F5 to change the camera views. \n"; 
	GetLog() << "Use mouse wheel to throttle and brake. \n"; 



	// Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(device);
	ChIrrWizard::add_typical_Sky(device);
	ChIrrWizard::add_typical_Lights(device);
	camera = device->getSceneManager()->addCameraSceneNode();
			camera->setPosition(core::vector3df(-4,7,-4));
	camera_view_type = 5;
	camera->setNearValue(0.1);

	// HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO...	 

	// ..the physical system
	my_system = new ChSystem;

	// ..the ground (flat, is a large cube..)

	my_ground = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											my_system, device->getSceneManager(),
											1.0,
											ChVector<>(0,-2,0),
											QUNIT, 
											ChVector<>(180,2,180) );
	my_ground->GetBody()->SetBodyFixed(true);
	my_ground->GetBody()->SetCollide(true);
	my_ground->GetBody()->SetFriction(1.3);
	my_ground->GetBody()->SetName("ground");
	video::ITexture* groundMap = device->getVideoDriver()->getTexture("../data/blu.png");
	my_ground->setMaterialTexture(0,groundMap); 

	// ..or do you prefer a detailed ground, with bumps, roads, uneven pavement etc.?
	//   If so, use the following instead, providing a mesh with the uneven ground:

	ChBodySceneNode* my_ground2 = (ChBodySceneNode*)addChBodySceneNode_easyGenericMesh(
											my_system, device->getSceneManager(),
											1.0,
											ChVector<>(0,-1,0),
											QUNIT,
											"../data/bumpers.obj",    // mesh name on disk
											true,					 // is static? yes - will be optimized for static case
											false					 // is convex? no, might require some preprocessing time for convex decomposition
											);
	my_ground2->GetBody()->SetFriction(1.3);
	my_ground2->GetBody()->SetBodyFixed(true);
    my_ground2->GetBody()->SetName("bumpers");

	ChBodySceneNode* my_ground3 = (ChBodySceneNode*)addChBodySceneNode_easyGenericMesh(
											my_system, device->getSceneManager(),
											1.0,
											ChVector<>(0,-1,0),
											QUNIT,
											"../data/houses.obj",    // mesh name on disk
											true,					 // is static? yes - will be optimized for static case
											false					 // is convex? no, might require some preprocessing time for convex decomposition
											);
	my_ground3->GetBody()->SetFriction(0.8);
	my_ground3->GetBody()->SetBodyFixed(true);
	my_ground3->GetBody()->SetCollide(true);
    my_ground3->GetBody()->SetName("bumpers");


	// Do you need some trees/houses/detail stuff which you want to see in 3D view but you
	// don't want to consider in collisions and simulation? use the following:
	/*
	IAnimatedMesh* details_mesh = device->getSceneManager()->getMesh("../data/details.obj");
	IAnimatedMeshSceneNode* details_node = device->getSceneManager()->addAnimatedMeshSceneNode(details_mesh, device->getSceneManager()->getRootSceneNode());
	*/


	// ..throw some debris obstacles on the ground:
	/*
	for (int i=0; i<4; i++)
	{
		ChBodySceneNode* my_obstacle_big = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											my_system, device->getSceneManager(),
											12.0,
											ChVector<>(20*ChRandom(),2, 20*ChRandom()),
											QUNIT, 
											ChVector<>(1,1.09,0.5) );
		ChBodySceneNode* my_obstacle_small = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											my_system, device->getSceneManager(),
											3.0,
											ChVector<>(20*ChRandom(),2, 20*ChRandom()),
											QUNIT, 
											ChVector<>(1,0.05,0.8) );
	}
	*/

	// ..the car (this class - see above - is a 'set' of bodies and links, automatically added at creation)
	mycar = new MySimpleCar(*my_system, device->getSceneManager(), device->getVideoDriver());





	// CREATE & INIT THE USER INTERFACE
	 
	// Create some graphical-user-interface (GUI) items to show on the screen.
	// This requires an event receiver object -see above.
	GUIinterface = new MyEventReceiver(this);

	// DO SOME SETTINGS 

	my_system->SetIterLCPmaxItersSpeed(16); // the higher, the easier to keep the constraints 'mounted'.
	my_system->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR); // better than _SYMMSOR - to discover why!!!
	
	my_system->SetIntegrationType(ChSystem::INT_TASORA); // can handle large penetrations
	//my_system->SetIntegrationType(ChSystem::INT_ANITESCU); // a bit faster but more 'spongy'

}
  



/// Delete the simulator.
/// Note that here all instanced contained objects must be deallocated, in
/// reverse order respect to the allocation in the MySimulator() creation code!

MySimulator::~MySimulator()
{		
	// Delete the GUI receiver
	if (GUIinterface)  delete GUIinterface; 

	// Delete the car..
	if (mycar) delete mycar;

	// Delete the system..
	if (my_system) delete my_system;

	// This safely delete every Irrlicht item..
	device->drop();
}





/// The simulation cycle. Perform endlessly a cycle (until the user clicks on 
/// window close) where time step integration is repeated and 3D view is redrawn.

void MySimulator::Run()
{
	IVideoDriver*    driver        = device->getVideoDriver();
	ISceneManager*	 msceneManager = device->getSceneManager();
	IGUIEnvironment* guienv        = device->getGUIEnvironment();

	// This will help choosing an integration step which matches the
	// real-time step of the simulation..
	ChRealtimeStepTimer m_realtime_timer;

	while(device->run())
	{ 
		// Irrlicht must prepare frame to draw
		driver->beginScene(true, true, SColor(255,140,161,192));
	
		// Update the position and the target of the video camera.
		this->UpdateVideoCamera();

		// Irrlicht now draws simple lines in 3D world representing a 
		// skeleton of the mechanism, in this instant:
		//
		// .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
		msceneManager->drawAll();

		// .. draw a grid (rotated so that it's horizontal)
		ChIrrTools::drawGrid(driver, 2, 2, 30,30, 
			ChCoordsys<>(ChVector<>(0,-0.99,0), Q_from_AngX(CH_C_PI_2) ),
			video::SColor(255, 80,130,130), true);

		// .. draw GUI user interface items (sliders, buttons) belonging to Irrlicht screen, if any
		guienv->drawAll();

		// .. draw the oscilloscope showing the graph of some recorded variable
		ChIrrTools::drawChFunction(this->device, &this->mycar->speed_recorder, 
			this->my_system->GetChTime()-2, 
			this->my_system->GetChTime(), 
			-3, 100);

		// .. update informations
		char message[50];
		sprintf(message,"Convergenza ant.: %g°", this->mycar->convergenza_anteriore);
		this->GUIinterface->text_convergenza_anteriore->setText(core::stringw(message).c_str());
		sprintf(message,"Convergenza pos.: %g°", this->mycar->convergenza_posteriore);
		this->GUIinterface->text_convergenza_posteriore->setText(core::stringw(message).c_str());


		// .. draw the distance constraints (the massless rods) as simplified lines
		std::list<chrono::ChLink*>::iterator iterlink =  my_system->Get_linklist()->begin();
		while(iterlink !=  my_system->Get_linklist()->end())
		{
			if (ChLinkDistance* mylinkdis = ChDynamicCast(ChLinkDistance,(*iterlink)))
				ChIrrTools::drawSegment(driver, 
					mylinkdis->GetEndPoint1Abs(), 
					mylinkdis->GetEndPoint2Abs(),
					video::SColor(255,   0,20,0), true);
			iterlink++;
		}

		// .. draw the spring constraints as simplified spring helix
		iterlink =  my_system->Get_linklist()->begin();
		while(iterlink !=  my_system->Get_linklist()->end())
		{
			if (ChLinkSpring* mylinkspri = ChDynamicCast(ChLinkSpring,(*iterlink)))
				ChIrrTools::drawSpring(driver, 0.03, 
					mylinkspri->GetEndPoint1Abs(),
					mylinkspri->GetEndPoint2Abs(),
					video::SColor(255,   150,20,20),   80,  5,  true);
			iterlink++;
		}
				

		// HERE CHRONO INTEGRATION IS PERFORMED: THE 
		// TIME OF THE SIMULATION ADVANCES FOR A SINGLE
		// STEP: 
		
		// double mstep = m_realtime_timer.SuggestSimulationStep(0.004);
		// mycar->Update(mstep);
		// my_system->DoStepDynamics( mstep );

		// EXPERIMENTAL: instead of computing on the fly the real-time step, just do N steps with fixed 1.6ms of
		// step - this works almost equal to real time on my Toshiba laptop, because it is able to compute 
		// the ten steps within the 1/60th of second of OpenGl Vsync.

		for (int j = 0; j < 10; j++)
		{
			double mstep = (1.0/60.0)/10;
			mycar->Update(mstep);
			my_system->DoStepDynamics(mstep);
		}

		//ChIrrTools::drawAllContactPoints(*my_system, driver, 1);

		// Irrlicht must finish drawing the frame
		driver->endScene(); 
	}
}






/// Updates the postion of the video camera in 3D scene. 

void MySimulator::UpdateVideoCamera()
{
	if (camera_view_type == 1) 
	{
		ChVector<> truss_relative_camera_target(0,0.5,-20);
		camera->setTarget( vector3dfCH(mycar->truss->GetBody()->Point_Body2World( &truss_relative_camera_target ) ) );
		core::matrix4 irrMat;
		ChQuaternion<> chq = mycar->truss->GetBody()->GetRot();
		core::quaternion irrQuat(chq.e0, chq.e1, chq.e2, chq.e3);
		irrMat = irrQuat.getMatrix();
		camera->setRotation(irrMat.getRotationDegrees());
		camera->updateAbsolutePosition();
	}
	if ((camera_view_type == 2) ||  
		(camera_view_type == 3) ||
		(camera_view_type == 4) )
	{
		ChVector<> truss_relative_camera_target(0,0,0);
		camera->setTarget( vector3dfCH(mycar->truss->GetBody()->Point_Body2World( &truss_relative_camera_target ) ) );
	}
	if (camera_view_type == 5)  
	{
		ChVector<> truss_relative_camera_target(0,0,0);
		camera->setTarget( vector3dfCH(mycar->truss->GetBody()->Point_Body2World( &truss_relative_camera_target ) ) );
	}
}
