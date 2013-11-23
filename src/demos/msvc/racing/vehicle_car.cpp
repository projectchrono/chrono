///////////////////////////////////////////////////
//
//   Vehicle 'car object'
//
///////////////////////////////////////////////////
   
 
#include "vehicle_car.h" 
#include "irrlicht_interface/ChDisplayTools.h"  

using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;



MySimpleCar::MySimpleCar(ChSystem&  my_system,	///< the chrono::engine physical system 
				ISceneManager* msceneManager,	///< the Irrlicht scene manager for 3d shapes
				IVideoDriver* mdriver			///< the Irrlicht video driver
				)
{
	throttle = 0; // initially, gas throttle is 0.
	conic_tau = 0.2; 

	gears[0] = -0.3; // rear gear<
	gears[1] =  0.3; // 1st gear
	gears[2] =  0.4; // 2nd gear
	gears[3] =  0.6; // 3rd gear
	gears[4] =  0.8; // 4th gear
	gears[5] =  1.0; // 5th gear

	actual_gear = 1;
	gear_tau = gears[actual_gear];

	// Define the torque curve with some polygonal approximation, entering 
	// a sequence of x-y pairs (speed-torque, with speed in [rad/s] and torque in [Nm])
	// (Of course a real engine should have not-null torque only from some positive value, but
	// here we start from -50 rad/s otherwise we cannot start - given we don't model the clutch)
	torque_curve.AddPoint(-50, 35);  
	torque_curve.AddPoint(0,  45);
	torque_curve.AddPoint(400,50);
	torque_curve.AddPoint(700,54);
	torque_curve.AddPoint(850, 0);
	torque_curve.AddPoint(1000, -54); // finish in 4th quadrant
	torque_curve.AddPoint(2000, -60); // finish in 4th quadrant
	
	ChStreamOutAsciiFile myfiletq("data_torquecurve.dat");
	torque_curve.FileAsciiPairsSave(myfiletq,-100,1000,200);
	
	motorspeed= motortorque=0;

	passo = 1.73162;
	carr  = 0.59963;

	wanted_steer = 0;
	actual_steer = 0;
	max_steer_speed = 0.5;

	convergenza_anteriore = 0;
	convergenza_posteriore = 0;
 
	max_brake_torque_post = 200;
	max_brake_torque_ant  = 300;
	braking = 0;

	// --- The car body ---

		// Set the position of the center of gravity of the truss, respect to the auxiliary reference that
		// we will use to measure all relative positions of joints on the truss. 
	truss_COG.Set( 0, 0, 0); 

	truss = (ChBodySceneNode*)addChBodySceneNode(
											&my_system, msceneManager, 0,
											150.0,
											ChVector<>(0, 0.016 ,1.99) +truss_COG,
											QUNIT);
	truss->GetBody()->SetInertiaXX(ChVector<>(4.8, 4.5, 1));
	truss->GetBody()->SetBodyFixed(false);
	 


		// Add some 'invisible' boxes which roughly define the collision shape of
		// the car truss. Each colliding box must be defined in x y z half-sizes and position of 
		// box center respect to the truss COG.
	truss->GetBody()->GetCollisionModel()->ClearModel();
	truss->GetBody()->GetCollisionModel()->AddBox(0.3, 0.3, 1.5, (ChVector<>(0,0.2,0)-truss_COG) ); // just example.. roughly the body
	truss->GetBody()->GetCollisionModel()->AddBox(0.1, 0.3, 0.1, (ChVector<>(0,0.4,0)-truss_COG) ); // just example.. roughly the rollbar
	truss->GetBody()->GetCollisionModel()->BuildModel();
	truss->GetBody()->SetCollide(true);

		// Add a visible mesh for Irrlicht representation of the
		// car - it must be found on disk with the relative path as indicated..
	IAnimatedMesh* scocca_mesh = msceneManager->getMesh("../data/mycar.obj");
	IAnimatedMeshSceneNode* scocca_node = msceneManager->addAnimatedMeshSceneNode( scocca_mesh, truss );
	scocca_node->setPosition(vector3dfCH(-truss_COG)); // if COG is not the reference we use for all stuff in truss..
	scocca_node->addShadowVolumeSceneNode();

	// --- Right Front suspension --- 

	// ..the car right-front spindle
	spindleRF = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&my_system, msceneManager,
											8.0,// mass of the spindle
											ChVector<>(-(carr-0.05216), 0.02865 ,0.93534),
											QUNIT, 
											ChVector<>(0.05, 0.22, 0.16) );
	spindleRF->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
	spindleRF->GetBody()->SetCollide(false);

	// ..the car right-front wheel
	wheelRF = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
											&my_system, msceneManager,
											3.0,
											ChVector<>(-carr ,0.02865 ,0.93534),
											chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Z),  //rotate, otherwise flat,horizontal
											ChVector<>(0.42, 0.17, 0.42) );
	wheelRF->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
	wheelRF->GetBody()->SetCollide(true);
	wheelRF->GetBody()->SetFriction(1.3);;
	wheelRF->GetBody()->SetName("wheelRF");
		video::ITexture* cylinderMap = mdriver->getTexture("../data/bluwhite.png");
	wheelRF->setMaterialTexture(0,	cylinderMap);
	wheelRF->addShadowVolumeSceneNode();

	// .. create the revolute joint between the wheel and the spindle
	//    Note that we specify two ChCoordinates of two markers which will be added in the two bodies,
	//    but both rotated with Z their axes pointing laterally thank to the Q_from_AngAxis() 90 degrees 
	//    rotations if necessary, because ChLinkLockRevolute will use the Z axes of markers as shafts.
	link_revoluteRF = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); // right, front, upper, 1
	link_revoluteRF->Initialize(wheelRF->GetBody(), spindleRF->GetBody(), 
		true,			// 'true' means: following two positions are relative to wheel and body
		ChCoordsys<>(ChVector<>(0,0,0) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_X)) ,		  // shaft in wheel coords
		ChCoordsys<>(ChVector<>(-0.05216,0,0) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) ); // shaft in spindle coords
	my_system.AddLink(link_revoluteRF);

		// .. create the brake between the wheel and the spindle, using the same markers already created for wheel-spindle revolute joint:
	link_brakeRF = ChSharedPtr<ChLinkBrake>(new ChLinkBrake); 
	link_revoluteRF->GetMarker1()->AddRef();
	link_revoluteRF->GetMarker2()->AddRef();
	link_brakeRF->Initialize(ChSharedMarkerPtr(link_revoluteRF->GetMarker1()), 
							 ChSharedMarkerPtr(link_revoluteRF->GetMarker2()) );
	my_system.AddLink(link_brakeRF);

	// .. impose distance between two parts (as a massless rod with two spherical joints at the end)
	link_distRFU1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, front, upper, 1
	link_distRFU1->Initialize(truss->GetBody(), spindleRF->GetBody(), 
		true,										// false = positions are relative to COG csystems of bodies, true = absolute positions
		ChVector<>(-0.24708, 0.09313, -1.17789)-truss_COG,  // position of 1st end rod in truss COG coordinate
		ChVector<>( 0.04263,0.0925,0),   			        // position of 2nd end rod in spindle COG coordinates
		false, 0.2895 );									// no auto-distance computation: use imposed distance (length of rod).
	my_system.AddLink(link_distRFU1);
	
	link_distRFU2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, front, upper, 2 
	link_distRFU2->Initialize(truss->GetBody(), spindleRF->GetBody(), 
		true, 
		ChVector<>(-0.24708,0.09313,-0.71972)-truss_COG,
		ChVector<>( 0.04263,0.0925,0),
		false, 0.4228 );								    // no auto-distance computation: use imposed distance (length of rod).
	my_system.AddLink(link_distRFU2);

	link_distRFL1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, front, lower, 1
	link_distRFL1->Initialize(truss->GetBody(), spindleRF->GetBody(), 
		true, 
		ChVector<>(-0.24708,-0.10273,-1.17789)-truss_COG, 
		ChVector<>( 0.04263,-0.1225,0),
		false, 0.2857 );								    // no auto-distance computation: use imposed distance (length of rod).
	my_system.AddLink(link_distRFL1);

	link_distRFL2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, front, lower, 2
	link_distRFL2->Initialize(truss->GetBody(), spindleRF->GetBody(), 
		true, 
		ChVector<>(-0.24708,-0.10273,-0.71972)-truss_COG, 
		ChVector<>( 0.04263,-0.1225,0),
		false, 0.4227 );								    // no auto-distance computation: use imposed distance (length of rod).
	my_system.AddLink(link_distRFL2);
	
	// .. create the spring between the truss and the spindle
	link_springRF = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
	link_springRF->Initialize(truss->GetBody(), spindleRF->GetBody(), 
		true, 
		ChVector<>(-0.31394,0.19399,-1.06243)-truss_COG, 
		ChVector<>( 0.06983,-0.08962,-0.00777), 
		false, 0.316);										// no auto-distance computation: use imposed spring restlength
	link_springRF->Set_SpringK(28300);
	link_springRF->Set_SpringR(80);
	my_system.AddLink(link_springRF);


	// .. create the rod for steering the wheel
	link_distRSTEER = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right steer
	link_distRSTEER->Initialize(truss->GetBody(), spindleRF->GetBody(), 
		true, 
		ChVector<>(-0.25457,0.10598,-1.24879)-truss_COG, 
		ChVector<>( 0.06661,0.09333,-0.15003),
		false, 0.2330);										// no auto-distance computation: use imposed distance (rod length)
	my_system.AddLink(link_distRSTEER);


	// --- Left Front suspension --- 

	// ..the car left-front spindle
	spindleLF = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&my_system, msceneManager,
											8.0,// mass of the spindle
											ChVector<>( (carr-0.05216), 0.02865 ,0.93534),
											QUNIT, 
											ChVector<>(0.05, 0.22, 0.16) );
	spindleLF->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
	spindleLF->GetBody()->SetCollide(false);

	// ..the car left-front wheel
	wheelLF = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
											&my_system, msceneManager,
											3.0,
											ChVector<>( carr ,0.02865 ,0.93534),
											chrono::Q_from_AngAxis(-CH_C_PI/2, VECT_Z),   //rotate, otherwise flat,horizontal
											ChVector<>(0.42, 0.17, 0.42) );
	wheelLF->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
	wheelLF->GetBody()->SetCollide(true);
	wheelLF->GetBody()->SetFriction(1.3);
	wheelLF->GetBody()->SetName("wheelLF");
	wheelLF->setMaterialTexture(0,	cylinderMap);
	wheelLF->addShadowVolumeSceneNode();

	// .. create the revolute joint between the wheel and the spindle
	link_revoluteLF = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); // left, front, upper, 1
	link_revoluteLF->Initialize(wheelLF->GetBody(), spindleLF->GetBody(), 
		true,			// 'true' means: following two positions are relative to wheel and body
		ChCoordsys<>(ChVector<>(0,0,0) , chrono::Q_from_AngAxis(-CH_C_PI/2, VECT_X)) ,  // shaft in wheel coords
		ChCoordsys<>(ChVector<>( 0.05216,0,0) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) ); // shaft in spindle coords
	my_system.AddLink(link_revoluteLF);

	// .. create the brake between the wheel and the spindle, using the same markers already created for wheel-spindle revolute joint:
	link_brakeLF = ChSharedPtr<ChLinkBrake>(new ChLinkBrake); 
	link_revoluteLF->GetMarker1()->AddRef();
	link_revoluteLF->GetMarker2()->AddRef();
	link_brakeLF->Initialize(ChSharedMarkerPtr(link_revoluteLF->GetMarker1()), 
							 ChSharedMarkerPtr(link_revoluteLF->GetMarker2()) );
	my_system.AddLink(link_brakeLF);

	// .. impose distance between two parts (as a massless rod with two spherical joints at the end)
	link_distLFU1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, front, upper, 1
	link_distLFU1->Initialize(truss->GetBody(), spindleLF->GetBody(), 
		true,										// false = positions are relative to COG csystems of bodies, true = absolute positions
		ChVector<>( 0.24708, 0.09313, -1.17789)-truss_COG,	// position of 1st end rod in truss COG coordinate
		ChVector<>(-0.04263,0.0925,0),   			        // position of 2nd end rod in spindle COG coordinates
		false, 0.2895 );									// no auto-distance computation: use imposed distance (length of rod).
	my_system.AddLink(link_distLFU1);

	link_distLFU2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, front, upper, 2 
	link_distLFU2->Initialize(truss->GetBody(), spindleLF->GetBody(), 
		true, 
		ChVector<>( 0.24708,0.09313, -0.71972)-truss_COG,
		ChVector<>(-0.04263,0.0925,0),
		false, 0.4228 );								    // no auto-distance computation: use imposed distance (length of rod).
	my_system.AddLink(link_distLFU2);


	link_distLFL1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, front, lower, 1
	link_distLFL1->Initialize(truss->GetBody(), spindleLF->GetBody(), 
		true, 
		ChVector<>( 0.24708,-0.10273,-1.17789)-truss_COG, 
		ChVector<>(-0.04263,-0.1225,0),
		false, 0.2857 );								    // no auto-distance computation: use imposed distance (length of rod).
	my_system.AddLink(link_distLFL1);


	link_distLFL2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, front, lower, 2
	link_distLFL2->Initialize(truss->GetBody(), spindleLF->GetBody(), 
		true, 
		ChVector<>( 0.24708,-0.10273,-0.71972)-truss_COG, 
		ChVector<>(-0.04263,-0.1225,0),
		false, 0.4227 );								    // no auto-distance computation: use imposed distance (length of rod).
	my_system.AddLink(link_distLFL2);
	
	// .. create the spring between the truss and the spindle
	link_springLF = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
	link_springLF->Initialize(truss->GetBody(), spindleLF->GetBody(), 
		true, 
		ChVector<>( 0.31394,0.19399,-1.06243)-truss_COG, 
		ChVector<>(-0.06983,-0.08962,-0.00777), 
		false, 0.316);										// no auto-distance computation: use imposed spring restlength
	link_springLF->Set_SpringK(28300);
	link_springLF->Set_SpringR(80);
	my_system.AddLink(link_springLF);


	// .. create the rod for steering the wheel
	link_distLSTEER = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right steer
	link_distLSTEER->Initialize(truss->GetBody(), spindleLF->GetBody(), 
		true, 
		ChVector<>( 0.25457,0.10598,-1.24879)-truss_COG, 
		ChVector<>(-0.06661,0.09333,-0.15003),
		false, 0.2330);										// no auto-distance computation: use imposed distance (rod length)
	my_system.AddLink(link_distLSTEER);




	// --- Right Back suspension --- 


	// ..the car right-back spindle
	spindleRB = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&my_system, msceneManager,
											8.0,
											ChVector<>(-(carr-0.05216), 0.02865 ,(0.93534+passo)),
											QUNIT, 
											ChVector<>(0.05, 0.22, 0.16) );
	spindleRB->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
	spindleRB->GetBody()->SetCollide(false);

	// ..the car right-back wheel
	wheelRB = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
											&my_system, msceneManager,
											3.0,
											ChVector<>(-carr ,0.02865 ,(0.93534+passo)),
											chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Z),    //rotate, otherwise flat,horizontal
											ChVector<>(0.42, 0.17, 0.42) );
	wheelRB->GetBody()->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
	wheelRB->GetBody()->SetCollide(true);
	wheelRB->GetBody()->SetFriction(1.3);
	wheelRB->GetBody()->SetName("wheelRB");
		cylinderMap = mdriver->getTexture("../data/bluwhite.png");
	wheelRB->setMaterialTexture(0,	cylinderMap);
	wheelRB->addShadowVolumeSceneNode();

	// .. create the revolute joint between the wheel and the spindle
	link_revoluteRB = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); // right, back, upper, 1
	link_revoluteRB->Initialize(wheelRB->GetBody(), spindleRB->GetBody(), 
		true,			// 'true' means: following two positions are relative to wheel and body
		ChCoordsys<>(ChVector<>(0,0,0) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_X)) ,		  // shaft in wheel coords
		ChCoordsys<>(ChVector<>(-0.05216,0,0) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) ); // shaft in spindle coords
	my_system.AddLink(link_revoluteRB);

	// .. create the brake between the wheel and the spindle, using the same markers already created for wheel-spindle revolute joint:
	link_brakeRB = ChSharedPtr<ChLinkBrake>(new ChLinkBrake); 
	link_revoluteRB->GetMarker1()->AddRef();
	link_revoluteRB->GetMarker2()->AddRef();
	link_brakeRB->Initialize(ChSharedMarkerPtr(link_revoluteRB->GetMarker1()), 
							 ChSharedMarkerPtr(link_revoluteRB->GetMarker2()) );
	my_system.AddLink(link_brakeRB);

	// .. create the motor transmission joint between the wheel and the truss (assuming small changes of alignment)
	link_engineR = ChSharedPtr<ChLinkEngine>(new ChLinkEngine); 
	link_engineR->Initialize(wheelRB->GetBody(), truss->GetBody(), 
		ChCoordsys<>(ChVector<>(-carr,0.02865,(0.93534+passo)) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
	link_engineR->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_CARDANO); // approx as a double Rzeppa joint
	link_engineR->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
	my_system.AddLink(link_engineR);

	// .. impose distance between two parts (as a massless rod with two spherical joints at the end)
	link_distRBU1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, back, upper, 1
	link_distRBU1->Initialize(truss->GetBody(), spindleRB->GetBody(), 
		true,										
		ChVector<>(-0.24708, 0.09313, -1.17789+passo)-truss_COG,	
		ChVector<>( 0.04263,0.0925,0),   			        // position of 2nd end rod in spindle COG coordinates
		false, 0.2895 );									// no auto-distance computation: use imposed distance (length of rod).			
	my_system.AddLink(link_distRBU1);
 
	link_distRBU2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, back, upper, 2
	link_distRBU2->Initialize(truss->GetBody(), spindleRB->GetBody(), 
		true, 
		ChVector<>(-0.24708,0.09313,-0.71972+passo)-truss_COG,
		ChVector<>( 0.04263,0.0925,0),
		false, 0.4228 );								    // no auto-distance computation: use imposed distance (length of rod).
	my_system.AddLink(link_distRBU2);

	link_distRBL1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, back, lower, 1
	link_distRBL1->Initialize(truss->GetBody(), spindleRB->GetBody(), 
		true, 
		ChVector<>(-0.24708,-0.10273,-1.17789+passo)-truss_COG, 
		ChVector<>( 0.04263,-0.1225,0),
		false, 0.2857 );								    // no auto-distance computation: use imposed distance (length of rod).
	my_system.AddLink(link_distRBL1);

	link_distRBL2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, back, lower, 2
	link_distRBL2->Initialize(truss->GetBody(), spindleRB->GetBody(), 
		true, 
		ChVector<>(-0.24708,-0.10273,-0.71972+passo)-truss_COG, 
		ChVector<>( 0.04263,-0.1225,0),
		false, 0.4227 );								    // no auto-distance computation: use imposed distance (length of rod).
	my_system.AddLink(link_distRBL2);
	
	// .. create the spring between the truss and the spindle
	link_springRB = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
	link_springRB->Initialize(truss->GetBody(), spindleRB->GetBody(), 
		true, 
		ChVector<>(-0.31394,0.19399,-1.06243+passo)-truss_COG, 
		ChVector<>( 0.06983,-0.08962,-0.00777), 
		false, 0.316);										// no auto-distance computation: use imposed spring restlength
	link_springRB->Set_SpringK(28300);
	link_springRB->Set_SpringR(80);
	my_system.AddLink(link_springRB);

	// .. create the rod for avoid the steering of the wheel
	link_distRBlat = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right rod
	link_distRBlat->Initialize(truss->GetBody(), spindleRB->GetBody(), 
		true, 
		ChVector<>(-0.25457,0.10598,-1.24879+passo)-truss_COG, 
		ChVector<>( 0.06661,0.09333,-0.10003),
		false, 0.2450);										// no auto-distance computation: use imposed distance (rod length)						
	my_system.AddLink(link_distRBlat);


	// --- Left Back suspension --- 

	// ..the car right-back spindle
	spindleLB = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&my_system, msceneManager,
											8.0,
											ChVector<>((carr-0.05216), 0.02865 ,(0.93534+passo)),
											QUNIT, 
											ChVector<>(0.05, 0.22, 0.16) );
	spindleLB->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
	spindleLB->GetBody()->SetCollide(false);

	// ..the car left-back wheel
	wheelLB = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
											&my_system, msceneManager,
											3.0,
											ChVector<>(carr ,0.02865 ,(0.93534+passo)),
											chrono::Q_from_AngAxis(-CH_C_PI/2, VECT_Z),  //rotate, otherwise flat,horizontal
											ChVector<>(0.42, 0.17, 0.42) );
	wheelLB->GetBody()->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
	wheelLB->GetBody()->SetCollide(true);
	wheelLB->GetBody()->SetFriction(1.3);
	wheelLB->GetBody()->SetName("wheelLB");
	wheelLB->setMaterialTexture(0,	cylinderMap);
	wheelLB->addShadowVolumeSceneNode();

	// .. create the revolute joint between the wheel and the spindle
	link_revoluteLB = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); // left, back, upper, 1
	link_revoluteLB->Initialize(wheelLB->GetBody(), spindleLB->GetBody(), 
		true,			// 'true' means: following two positions are relative to wheel and body
		ChCoordsys<>(ChVector<>(0,0,0) , chrono::Q_from_AngAxis(-CH_C_PI/2, VECT_X)) ,		  // shaft in wheel coords
		ChCoordsys<>(ChVector<>(0.05216,0,0) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) ); // shaft in spindle coords
	my_system.AddLink(link_revoluteLB);

	// .. create the brake between the wheel and the spindle, using the same markers already created for wheel-spindle revolute joint:
	link_brakeLB = ChSharedPtr<ChLinkBrake>(new ChLinkBrake); 
	link_revoluteLB->GetMarker1()->AddRef();
	link_revoluteLB->GetMarker2()->AddRef();
	link_brakeLB->Initialize(ChSharedMarkerPtr(link_revoluteLB->GetMarker1()), 
							 ChSharedMarkerPtr(link_revoluteLB->GetMarker2()) );
	my_system.AddLink(link_brakeLB);

	// .. create the motor transmission joint between the wheel and the truss (assuming small changes of alignment)
	link_engineL = ChSharedPtr<ChLinkEngine>(new ChLinkEngine); 
	link_engineL->Initialize(wheelLB->GetBody(), truss->GetBody(), 
		ChCoordsys<>(ChVector<>(carr,0.02865,(0.93534+passo)) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
	link_engineL->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_CARDANO); // approx as a double Rzeppa joint
	link_engineL->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
	my_system.AddLink(link_engineL);

	// .. impose distance between two parts (as a massless rod with two spherical joints at the end)
	link_distLBU1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, back, upper, 1
	link_distLBU1->Initialize(truss->GetBody(), spindleLB->GetBody(), 
		true,										
		ChVector<>( 0.24708, 0.09313, -1.17789+passo)-truss_COG,
		ChVector<>(-0.04263,0.0925,0),   			        // position of 2nd end rod in spindle COG coordinates
		false, 0.2895 );									// no auto-distance computation: use imposed distance (length of rod).		
	my_system.AddLink(link_distLBU1);
 
	link_distLBU2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, back, upper, 2
	link_distLBU2->Initialize(truss->GetBody(), spindleLB->GetBody(), 
		true, 
		ChVector<>( 0.24708,0.09313, -0.71972+passo)-truss_COG,
		ChVector<>(-0.04263,0.0925,0),
		false, 0.4228 );								    // no auto-distance computation: use imposed distance (length of rod).
	my_system.AddLink(link_distLBU2);

	link_distLBL1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, back, lower, 1
	link_distLBL1->Initialize(truss->GetBody(), spindleLB->GetBody(), 
		true, 
		ChVector<>( 0.24708,-0.10273,-1.17789+passo)-truss_COG, 
		ChVector<>(-0.04263,-0.1225,0),
		false, 0.2857 );								    // no auto-distance computation: use imposed distance (length of rod).
	my_system.AddLink(link_distLBL1);

	link_distLBL2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, back, lower, 2
	link_distLBL2->Initialize(truss->GetBody(), spindleLB->GetBody(), 
		true, 
		ChVector<>( 0.24708,-0.10273,-0.71972+passo)-truss_COG, 
		ChVector<>(-0.04263,-0.1225,0),
		false, 0.4227 );								    // no auto-distance computation: use imposed distance (length of rod).
	my_system.AddLink(link_distLBL2);
	
	// .. create the spring between the truss and the spindle
	link_springLB = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
	link_springLB->Initialize(truss->GetBody(), spindleLB->GetBody(), 
		true, 
		ChVector<>( 0.31394,0.19399,-1.06243+passo)-truss_COG, 
		ChVector<>(-0.06983,-0.08962,-0.00777), 
		false, 0.316);										// no auto-distance computation: use imposed spring restlength
	link_springLB->Set_SpringK(28300);
	link_springLB->Set_SpringR(80);
	my_system.AddLink(link_springLB);

	// .. create the rod for avoid the steering of the wheel
	link_distLBlat = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right 
	link_distLBlat->Initialize(truss->GetBody(), spindleLB->GetBody(), 
		true, 
		ChVector<>( 0.25457,0.10598,-1.24879+passo)-truss_COG, 
		ChVector<>(-0.06661,0.09333,-0.10003),
		false, 0.2450);										// no auto-distance computation: use imposed distance (rod length)						
	my_system.AddLink(link_distLBlat);


	// Create also the motor sound
	//
	
	 // start the sound engine with default parameters
	sound_engine = irrklang::createIrrKlangDevice();

	if (!sound_engine)
	{
		GetLog() << "Cannot start sound engine Irrklang \n";
	}

	// To play a sound, we only to call play2D(). The second parameter
	// tells the engine to play it looped.

	// play some sound stream, looped
	if (sound_engine)
		car_sound = sound_engine->play2D("../data/carsound.ogg", true, false, true);
	else 
		car_sound = 0;

}



		// Delete the car object, deleting also all bodies corresponding to
		// the various parts and removing them from the physical system.  Also
		// removes constraints from the system.
MySimpleCar::~MySimpleCar()
{
	ChSystem* mysystem = spindleRF->GetBody()->GetSystem(); // trick to get the system here	

		// When a ChBodySceneNode is removed via ->remove() from Irrlicht 3D scene manager,
		// it is also automatically removed from the ChSystem (the ChSystem::RemoveBody() is
		// automatically called at Irrlicht node deletion - see ChBodySceneNode.h ).

		// For links, just remove them from the ChSystem using ChSystem::RemoveLink()
	mysystem->RemoveLink(link_revoluteRF);
	mysystem->RemoveLink(link_brakeRF);
	mysystem->RemoveLink(link_distRFU1);
	mysystem->RemoveLink(link_distRFU2);
	mysystem->RemoveLink(link_distRFL1);
	mysystem->RemoveLink(link_distRFL2);
	mysystem->RemoveLink(link_springRF);
	mysystem->RemoveLink(link_distRSTEER);

	mysystem->RemoveLink(link_revoluteLF);
	mysystem->RemoveLink(link_brakeLF);
	mysystem->RemoveLink(link_distLFU1);
	mysystem->RemoveLink(link_distLFU2);
	mysystem->RemoveLink(link_distLFL1);
	mysystem->RemoveLink(link_distLFL2);
	mysystem->RemoveLink(link_springLF);
	mysystem->RemoveLink(link_distLSTEER);

	mysystem->RemoveLink(link_revoluteRB);
	mysystem->RemoveLink(link_brakeRB);
	mysystem->RemoveLink(link_distRBU1);
	mysystem->RemoveLink(link_distRBU2);
	mysystem->RemoveLink(link_distRBL1);
	mysystem->RemoveLink(link_distRBL2);
	mysystem->RemoveLink(link_springRB);
	mysystem->RemoveLink(link_distRBlat);
	mysystem->RemoveLink(link_engineR);

	mysystem->RemoveLink(link_revoluteLB);
	mysystem->RemoveLink(link_brakeLB);
	mysystem->RemoveLink(link_distLBU1);
	mysystem->RemoveLink(link_distLBU2);
	mysystem->RemoveLink(link_distLBL1);
	mysystem->RemoveLink(link_distLBL2);
	mysystem->RemoveLink(link_springLB);
	mysystem->RemoveLink(link_distLBlat);
	mysystem->RemoveLink(link_engineL);

	truss->remove();
	spindleRF->remove();
	wheelRF->remove();
	spindleLF->remove();
	wheelLF->remove();
	spindleRB->remove();
	wheelRB->remove();
	spindleLB->remove();
	wheelLB->remove();

	sound_engine->drop(); 
}



		// This can be used, at each time step, to compute the actual value of torque
		// transmitted to the wheels, according to gas throttle / speed / gear value.
		// The following is a very simplified model (the torque curve of the motor is linear
		// and no latency or inertial or clutch effects in gear train are considered.)
double MySimpleCar::ComputeWheelTorque()
{
	// Assume clutch is never used. Given the kinematics of differential,
	// the speed of the engine transmission shaft is the average of the two wheel speeds,
	// multiplied the conic gear transmission ratio inversed:
	double shaftspeed = - (1.0/this->conic_tau) * 0.5 *
		(this->link_engineL->Get_mot_rot_dt()+this->link_engineR->Get_mot_rot_dt());
	// The motorspeed is the shaft speed multiplied by gear ratio inversed:
	this->motorspeed = (1.0/this->gear_tau)*shaftspeed;
	// The torque depends on speed-torque curve of the motor: 
	//this->motortorque = 60 - this->motorspeed*(60.0/800.0) ;
	this->motortorque = torque_curve.Get_y(this->motorspeed);
	//GetLog() << " motorspeed  " << this->motorspeed << " motortorque  " << this->motortorque << "\n";
	// Motor torque is linearly modulated by throttle gas value:
	this->motortorque = this->motortorque *  this->throttle;
	// The torque at motor shaft:
	double shafttorque =  this->motortorque * (1.0/this->gear_tau);
	// The torque at wheels - for each wheel, given the differential transmission, 
	// it is half of the shaft torque  (multiplied the conic gear transmission ratio)
	double singlewheeltorque = - 0.5 * shafttorque * (1.0/this->conic_tau);
	// Set the wheel torque in both 'engine' links, connecting the wheels to the truss;
	if (ChFunction_Const* mtorqueL= dynamic_cast<ChFunction_Const*> (this->link_engineL->Get_tor_funct()))
		mtorqueL->Set_yconst(singlewheeltorque);
	if (ChFunction_Const* mtorqueR= dynamic_cast<ChFunction_Const*> (this->link_engineR->Get_tor_funct()))
		mtorqueR->Set_yconst(singlewheeltorque);
	//debug:print infos on screen:
	   //GetLog() << "motor torque="<< motortorque<< "  speed=" << motorspeed << "  wheel torqe=" << singlewheeltorque <<"\n";

	static int stepsbetweensound = 0;
	// Update sound pitch
	if(car_sound)
	{
		stepsbetweensound ++;
		double soundspeed = motorspeed/(900);
		if (soundspeed <0.2) soundspeed = 0.2;
		if (stepsbetweensound > 10)
		{
			stepsbetweensound =0;
			car_sound->setPlaybackSpeed(soundspeed);
		}
	}

	// If needed, return also the value of wheel torque:
	return singlewheeltorque;
}


void  MySimpleCar::Update(double dt)
{
	// Phase --1--
	//
	//      Compute and apply the wheel torque

	this->ComputeWheelTorque();

	// Phase --2--
	//
	//      Compute wheel convergence, caster, etc.

	ChVector<> mdirLF =  Vcross(this->wheelLF->GetBody()->GetA()->Get_A_Yaxis() , VECT_Y ); 
	ChVector<> mdirRF = -Vcross(this->wheelRF->GetBody()->GetA()->Get_A_Yaxis() , VECT_Y ); 
	this->convergenza_anteriore = CH_C_RAD_TO_DEG * acos(  Vdot(mdirLF, mdirRF)  );
	if ((Vcross(mdirLF,mdirRF)).y > 0)
		this->convergenza_anteriore *= -1; // change sign if divergent

	ChVector<> mdirLB =  Vcross(this->wheelLB->GetBody()->GetA()->Get_A_Yaxis() , VECT_Y ); 
	ChVector<> mdirRB = -Vcross(this->wheelRB->GetBody()->GetA()->Get_A_Yaxis() , VECT_Y ); 
	this->convergenza_posteriore = CH_C_RAD_TO_DEG * acos(  Vdot(mdirLB, mdirRB)  );
	if ((Vcross(mdirLB,mdirRB)).y > 0)
		this->convergenza_posteriore *= -1; // change sign if divergent

	// Phase --3--
	//
	//      Do steering - not exceeding a limit on steer speed
	
	if (dt>0)
	{
		double newsteer = this->wanted_steer;
		// clamp: not too fast steering:
		if  ( (newsteer - this->actual_steer)/dt >  this->max_steer_speed )
			newsteer = this->actual_steer + this->max_steer_speed * dt;
		if  ( (newsteer - this->actual_steer)/dt < -this->max_steer_speed )
			newsteer = this->actual_steer - this->max_steer_speed * dt;
		this->link_distRSTEER->SetEndPoint1Rel( ChVector<>(-0.25457-newsteer,0.10598,-1.24879)- truss_COG );
		this->link_distLSTEER->SetEndPoint1Rel( ChVector<>( 0.25457-newsteer,0.10598,-1.24879)- truss_COG );
		this->actual_steer = newsteer;
	}

	//***TODO*** compute caster, etc.


	// Phase --4--
	//
	//      Record something to be shown in oscilloscope (ex. the speed of the car)

	speed_recorder.Set_max_amount(3000);  // record only last three seconds (3000 steps of 1ms each).
	speed_recorder.Set_dx(dt);
	speed_recorder.AddLastPoint(this->truss->GetBody()->GetSystem()->GetChTime(), 
								this->truss->GetBody()->GetPos_dt().Length() * (60 * 60 / 1000) );  // convert to km/h  

	motorspeed_recorder.Set_max_amount(3000); 
	motorspeed_recorder.Set_dx(dt);
	motorspeed_recorder.AddLastPoint(this->truss->GetBody()->GetSystem()->GetChTime(), 
								this->motorspeed * (60 / CH_C_2PI) );  // convert to RPM 

}


void MySimpleCar::SetBraking(double mbraking)
{
	braking = mbraking;
	this->link_brakeRB->Set_brake_torque(mbraking * this->max_brake_torque_post);
	this->link_brakeLB->Set_brake_torque(mbraking * this->max_brake_torque_post);
	this->link_brakeRF->Set_brake_torque(mbraking * this->max_brake_torque_ant);
	this->link_brakeLF->Set_brake_torque(mbraking * this->max_brake_torque_ant);
}


void MySimpleCar::ChangeGearUp()
{
	actual_gear++;
	if (actual_gear >5)
		actual_gear = 5;
	gear_tau = gears[actual_gear];
}

void MySimpleCar::ChangeGearDown()
{
	actual_gear--;
	if (actual_gear <0)
		actual_gear = 0;
	gear_tau = gears[actual_gear];
}
		
void MySimpleCar::ChangeGearN(int newgear)
{
	actual_gear = newgear;
	if (actual_gear >5)
		actual_gear = 5;
	if (actual_gear <0)
		actual_gear = 0; 
	gear_tau = gears[actual_gear];
}

void MySimpleCar::SaveFunctionsToDisk()
{
	double from_time = this->truss->GetBody()->GetChTime()-3;  // save last three seconds 
	double to_time   = this->truss->GetBody()->GetChTime();
	try 
	{
		ChStreamOutAsciiFile myfilespeeds("data_speedkmh.dat");
		this->speed_recorder.FileAsciiPairsSave(myfilespeeds,from_time,to_time,200);
		
		ChStreamOutAsciiFile myfilemotorspeeds("data_motorspeedrpm.dat");
		this->motorspeed_recorder.FileAsciiPairsSave(myfilemotorspeeds,from_time,to_time,200);

	} 
	catch (ChException myerror) 
	{
		GetLog() << "Error. Cannot save data to disk. \n";
	}
}
