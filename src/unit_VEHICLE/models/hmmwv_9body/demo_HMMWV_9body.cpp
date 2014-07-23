//  - Demo of a  High Mobility Multi-Wheeled Vehicle (HMMWV).
//  - a nice user-based GUI to drive the car with your keyboard.
//  - Also works without the GUI (i.e., no Irrlicht), in which you must specify the 
//  driver throttle/torque each timestep yourself
//  - Using IrrAssets in an efficient way to create bodies, collision
//  objects and visualization assets quickly
//
//  Author: Justin Madsen, 2014
//	Part of the Chrono-T project


// CE includes
#include "physics/CHapidll.h" 
#include "physics/CHsystem.h"
#include "physics/CHlinkDistance.h"
// terramechanics include. Note: this file needs to be updated based on demo_TMsoilbin.cpp

// Irrlicht includes
#ifdef ENABLE_UNIT_IRRLICHT
 #include "irrlicht_interface/ChIrrApp.h"
 #include <irrlicht.h>
 #include "HMMWVEventReceiver.h"
#endif

// Include the source for the car class, nothing special here
#include "HMMWV_9body.h"
#include "HMMWVTerrain.h"

#include <omp.h>		// just using this for timers
// Use the namespace of Chrono
using namespace chrono;

#ifdef USE_IRRLICHT
// Use the main namespaces of Irrlicht
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;
#endif

int main(int argc, char* argv[]){
	// create the system, set the solver settings
	DLL_CreateGlobals();
	ChSystem m_system;
	// Integration and Solver settings
	m_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR); 
	m_system.SetIterLCPmaxItersSpeed(150);
	m_system.SetIterLCPmaxItersStab(150);

	// chassis CM position, in SAE units [inches]
	ChVector<> chassis_cmSAE = ChVector<>(600.5, 600.5, -36.0/2.0);
	// create the HMMWV vehicle
	ChVector<> chassisCM = chassis_cmSAE * 0.0254;
	ChQuaternion<> chassisOri(1,0,0,0);	// forward is the positive x-direction

	// ***** vehicle module
	HMMWV_9body* mycar = new HMMWV_9body(m_system, chassisCM, chassisOri,false);

	// set the location of the "ground" relative to the chassisCM.
	ChVector<> ground_cm = ChVector<>(0,0,0); 
	ground_cm = chassisCM;
	double terrainWidth = 100.0;
	double terrainLength =100.0;
	// create the ground. NOTE: orientation will have to be in the x-y plane
	HMMWVTerrain* terrain = new HMMWVTerrain(m_system, ground_cm, terrainWidth, terrainLength,0.5,0.5,false);
//	ChVector<> obstacle_location(10,1,1);
//	terrain->create_some_obstacles(obstacle_location);
	
	// USER INTERFACE, if using Irrlicht
#ifdef USE_IRRLICHT
	// Create the Irrlicht device, defaults to DirectX
	ChIrrApp application(&m_system, L"Justin's HMMWV demo",	core::dimension2d<u32>(1000,800),false,true);
	application.AddTypicalSky();
	application.AddTypicalLights();
	// camera is behind and above chassis, its target
	core::vector3df camera_pos = core::vector3df(chassisCM.x, chassisCM.y, chassisCM.z );
	core::vector3df camera_targ = core::vector3df(camera_pos);
	camera_pos.X -= 1.5; camera_pos.Y += 0.6;
	application.AddTypicalCamera(camera_pos,camera_targ );

	// This is for GUI for the on-road HMMWV vehicle
	HMMWVEventReceiver receiver(&application, &m_system, mycar, terrain);
	application.SetUserEventReceiver(&receiver);
	// Use real-time step of the simulation, OR...
//	application.SetStepManage(true);
	application.SetTimestep(0.001);
//	application.SetTryRealtime(true);
	

	// last thing to add: if we're using Irrlicht, add some stuff to the scene
	ChSharedPtr<ChCamera> mcamera(new ChCamera);
	mcamera->SetPosition(ChVector<>(3.5,2.5f,-2.4) );
//	mcamera->SetAimPoint(wheelCMpos);
	// attach this to the wheel?
	// mwheel->wheel->AddAsset(mcamera);
	application.AddTypicalCamera(core::vector3df(0,2,-4));

	// 
	// set up the assets for rendering
	application.AssetBindAll();
	application.AssetUpdateAll();
#else
	m_system.SetStep(0.005);
	double tend = 10.0;
#endif

	// keep track of the steps we save
	int stepNum = 0;

#ifdef USE_TERRAIN
	// change a few parameters in the soil model
	// TILLED LOAM -- from Wulfsohn (1992), and USCS CL type soil
	// Bekker/Reece parameters 
	inject_SoilData(3.21,2.32,	// clay and grain index
		47.1,24.0,0.9,			// pClay, pSilt, rho0 [g/cm^3]
		50,50,					// S1, Sk
		100, 200,				// rci6, rci12
		6.3, 12.5, 12.0, 0.73,	// kc, kphi, width, n
		4.0, 6.5,				// kc, kphi (Reece)
		0.37*180./CH_C_PI, 1.03, 1.5);		// phi, coh, shear_K (Janosi-hanamoto)

	/*
	// FIRM LOAM, from Wulfsohn (1992), and USCS MH type soil
	inject_SoilData(2.38,4.27,
		34.8, 50.0, 1.449,
		50,50,
		100,200,
		12.3, 18.5, 12.0, 1.0,
		11.0, 17.0
		0.4*180./CH_C_PI, 5.18, 0.417);
		*/

	// hold onto the ChBodies of each wheel, to update the TM db with each timestep
	std::vector<ChSharedPtr<ChBody>> wheel_vect;
	wheel_vect.push_back(mycar->wheelRF->wheel);
	wheel_vect.push_back(mycar->wheelLF->wheel);
	wheel_vect.push_back(mycar->wheelRB->wheel);
	wheel_vect.push_back(mycar->wheelLB->wheel);
#endif
#ifdef USE_IRRLICHT
	// Main time-stepping loop
	while(application.GetDevice()->run()) { 
		double simtime = application.GetSystem()->GetChTime();	// current sim time
		double timeStep = application.GetTimestep();	// current step size
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
		// try to have the camera follow the vehicle
		irr::scene::ICameraSceneNode *mCamera = application.GetSceneManager()->getActiveCamera();
		// camera should trail the car
		ChVector<> cameraOffset(-1,2,2);
		ChVector<> tmp_offset =  mycar->getCM_pos_chassis() - cameraOffset;
		irr::core::vector3df mVect = irr::core::vector3df(tmp_offset.x, tmp_offset.y, tmp_offset.z);
		mCamera->setPosition( mVect);
		ChVector<> chCM = mycar->getCM_pos_chassis();
		mCamera->setTarget( irr::core::vector3df( chCM.x, chCM.y, chCM.z) );

		// .. draw distance constraints (the massless rods) as simplified lines
		receiver.drawLinks();
		receiver.drawSprings();

#ifdef USE_TERRAIN
		// draw the terramechanics forces as vectors on each tire
		for(int i = 0; i < 4; i++){
			receiver.drawTMvisGrid(i);
 			receiver.drawTM_forces(i);
		}
#endif
		if( receiver.gad_tab_carData->isVisible() )
			receiver.drawCarDataOutput();

		// Irrlicht draws the scene
		application.DrawAll();
#else
	while( m_system.GetChTime() <= tend)
	{
#endif

		
#ifdef USE_IRRLICHT

		// update the position, velocity, orientation w/ Chrono data from the last time step
		// also, since we're passing in the ChBodies, apply wheel terramechanics rxn force/torque directly
		// only do a TM time step if the Chrono simulation is running
 #ifdef USE_TERRAIN
		if( !application.GetPaused() ) {
 			m_TM->do_TM_Step(wheel_vect, simtime, timeStep);
			// apply a driving torque to the wheel via the test mechanism
		}
 #endif
		// based on current shaft speed, user torque, find the driving torque on the wheels
		mycar->ComputeWheelTorque();
		mycar->ComputeSteerDisplacement();

		// each timestep, need to pass new forces and torques to the wheel hub!
		std::vector<ChVector<>> F_hub;
		std::vector<ChVector<>> M_hub;
 #ifdef USE_TERRAIN
		for(int i = 0; i < 4; i++) {
			F_hub.push_back( m_TM->getSpindleForce(i) );
			M_hub.push_back( m_TM->getSpindleTorque(i) );
		}
 #else
		// here, use the Pac tire model to compute F and M on wheel spindle
//		for(int i = 0; i < 4; i++) {
//			F_hub.push_back( getPacTire_SpindleForce(i) );
//			M_hub.push_back( getPacTire_SpindleTorque(i) );
//		}
 #endif

		// apply the wheel hub reaction forces and moments. Does not interfere w/ applied torque (I think)
		// That is applied through the ChLinkEngine, this is force accumulators
		mycar->applyHub_FM(F_hub, M_hub );

		// ------------------------------------------------
		// main time stepping loop, using Irrlicht
		application.DoStep();
		application.GetVideoDriver()->endScene();
#else
		// main time stepping loop, using the ChSystem
		m_system.DoStepDynamics( m_system.GetStep() );
#endif

		// end of time stepping loop ------------------------
	}


#ifdef USE_IRRLICHT
	// This safely delete every Irrlicht item..
	application.GetDevice()->drop();
#endif


	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}