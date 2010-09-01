///////////////////////////////////////////////////
//
//   Demo code about 
//
//     - modeling a complex mechanism (a quarter car model)
//     - using the ChLinkSpring to make spring-damper system
//     - using the ChLinkDistance class to reperesent 
//       long and thin massless rods, whose mass is negligible 
//       for dynamical analysis (as often happens in mechanisms)
//       so they can be modeled as 'distance' constraints
//       instead of making a thin body with small mass and two
//       spherical joints at the end (wihch would be much less
//       efficient from the computational point of view).
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
#include "physics/ChLinkDistance.h"
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



// First of all, define a class for the 'car' (that is, a set of
// bodies and links which are grouped within this class; so it is 
// easier to manage data structures in this example).

class MySimpleCar {
public:
		// THE DATA

	double throttle; // actual value 0...1 of gas throttle.
	double conic_tau; // the transmission ratio of the conic gears at the rear axle
	double gear_tau; // the actual tau of the gear
	double max_motor_torque; // the max torque of the motor [Nm];
	double max_motor_speed;	 // the max rotation speed of the motor [rads/s]

		// The parts making the car, as 3d Irrlicht scene nodes, each containing
		// the ChBody object
			// .. truss:
	ChBodySceneNode* truss;
			// .. right front suspension:
	ChBodySceneNode* spindleRF;
	ChBodySceneNode* wheelRF;
	ChSharedPtr<ChLinkLockRevolute> link_revoluteRF;
	ChSharedPtr<ChLinkDistance> link_distRFU1;
	ChSharedPtr<ChLinkDistance> link_distRFU2;
	ChSharedPtr<ChLinkDistance> link_distRFL1;
	ChSharedPtr<ChLinkDistance> link_distRFL2;
	ChSharedPtr<ChLinkSpring>   link_springRF;
	ChSharedPtr<ChLinkDistance> link_distRSTEER;
			// .. left front suspension:
	ChBodySceneNode* spindleLF;
	ChBodySceneNode* wheelLF;
	ChSharedPtr<ChLinkLockRevolute> link_revoluteLF;
	ChSharedPtr<ChLinkDistance> link_distLFU1;
	ChSharedPtr<ChLinkDistance> link_distLFU2;
	ChSharedPtr<ChLinkDistance> link_distLFL1;
	ChSharedPtr<ChLinkDistance> link_distLFL2;
	ChSharedPtr<ChLinkSpring>   link_springLF;
	ChSharedPtr<ChLinkDistance> link_distLSTEER;
			// .. right back suspension:
	ChBodySceneNode* spindleRB;
	ChBodySceneNode* wheelRB;
	ChSharedPtr<ChLinkLockRevolute> link_revoluteRB;
	ChSharedPtr<ChLinkDistance> link_distRBU1;
	ChSharedPtr<ChLinkDistance> link_distRBU2;
	ChSharedPtr<ChLinkDistance> link_distRBL1;
	ChSharedPtr<ChLinkDistance> link_distRBL2;
	ChSharedPtr<ChLinkSpring>   link_springRB;
	ChSharedPtr<ChLinkDistance> link_distRBlat;
	ChSharedPtr<ChLinkEngine>   link_engineL;
			// .. left back suspension:
	ChBodySceneNode* spindleLB;
	ChBodySceneNode* wheelLB;
	ChSharedPtr<ChLinkLockRevolute> link_revoluteLB;
	ChSharedPtr<ChLinkDistance> link_distLBU1;
	ChSharedPtr<ChLinkDistance> link_distLBU2;
	ChSharedPtr<ChLinkDistance> link_distLBL1;
	ChSharedPtr<ChLinkDistance> link_distLBL2;
	ChSharedPtr<ChLinkSpring>   link_springLB;
	ChSharedPtr<ChLinkDistance> link_distLBlat;
	ChSharedPtr<ChLinkEngine>   link_engineR;
		
		// THE FUNCTIONS

		// Build and initialize the car, creating all bodies corresponding to
		// the various parts and adding them to the physical system - also creating
		// and adding constraints to the system.
	MySimpleCar(ChSystem&  my_system,	///< the chrono::engine physical system 
				ISceneManager* msceneManager, ///< the Irrlicht scene manager for 3d shapes
				IVideoDriver* mdriver	///< the Irrlicht video driver
				)
			{
				throttle = 0; // initially, gas throttle is 0.
				conic_tau = 0.2;
				gear_tau = 0.3;
				max_motor_torque = 80; 
				max_motor_speed = 800;

				// --- The car body --- 

				 truss = (ChBodySceneNode*)addChBodySceneNode_easyBox(
														&my_system, msceneManager,
														150.0,
														ChVector<>(0, 1 ,0),
														QUNIT, 
														ChVector<>(1, 0.5, 3) );
				truss->GetBody()->SetInertiaXX(ChVector<>(4.8, 4.5, 1));
				truss->GetBody()->SetBodyFixed(false);
				truss->addShadowVolumeSceneNode();

				// --- Right Front suspension --- 

				// ..the car right-front spindle
				spindleRF = (ChBodySceneNode*)addChBodySceneNode_easyBox(
														&my_system, msceneManager,
														8.0,
														ChVector<>(1.3, 1 , 1),
														QUNIT, 
														ChVector<>(0.1, 0.4, 0.4) );
				spindleRF->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
				spindleRF->GetBody()->SetCollide(false);

				// ..the car right-front wheel
				wheelRF = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
														&my_system, msceneManager,
														3.0,
														ChVector<>(1.5, 1 , 1),
														chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Z), 
														ChVector<>(0.9, 0.3, 0.9) );
				wheelRF->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
				wheelRF->GetBody()->SetCollide(true);
				wheelRF->GetBody()->SetFriction(1.0);
					video::ITexture* cylinderMap = mdriver->getTexture("../data/bluwhite.png");
				wheelRF->setMaterialTexture(0,	cylinderMap);
				wheelRF->addShadowVolumeSceneNode();

				// .. create the revolute joint between the wheel and the spindle
				link_revoluteRF = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); // right, front, upper, 1
				link_revoluteRF->Initialize(wheelRF->GetBody(), spindleRF->GetBody(), 
					ChCoordsys<>(ChVector<>(1.5, 1, 1) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
				my_system.AddLink(link_revoluteRF);

				// .. impose distance between two parts (as a massless rod with two spherical joints at the end)
				link_distRFU1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, front, upper, 1
				link_distRFU1->Initialize(truss->GetBody(), spindleRF->GetBody(), false, ChVector<>(0.5,1.2,1.2), ChVector<>(1.25,1.2,1));
				my_system.AddLink(link_distRFU1);
			 
				link_distRFU2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, front, upper, 2
				link_distRFU2->Initialize(truss->GetBody(), spindleRF->GetBody(), false, ChVector<>(0.5,1.2,0.8), ChVector<>(1.25,1.2,1));
				my_system.AddLink(link_distRFU2);

				link_distRFL1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, front, lower, 1
				link_distRFL1->Initialize(truss->GetBody(), spindleRF->GetBody(), false, ChVector<>(0.5,0.8,1.2), ChVector<>(1.25,0.8,1));
				my_system.AddLink(link_distRFL1);

				link_distRFL2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, front, lower, 2
				link_distRFL2->Initialize(truss->GetBody(), spindleRF->GetBody(), false, ChVector<>(0.5,0.8,0.8), ChVector<>(1.25,0.8,1));
				my_system.AddLink(link_distRFL2);
				
				// .. create the spring between the truss and the spindle
				link_springRF = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
				link_springRF->Initialize(truss->GetBody(), spindleRF->GetBody(), false, ChVector<>(0.5,1.2,1.0), ChVector<>(1.25,0.8,1));
				link_springRF->Set_SpringK(28300);
				link_springRF->Set_SpringR(80);
				my_system.AddLink(link_springRF);

				// .. create the rod for steering the wheel
				link_distRSTEER = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right steer
				link_distRSTEER->Initialize(truss->GetBody(), spindleRF->GetBody(), false, ChVector<>(0.5,1.21,1.4), ChVector<>(1.25,1.21,1.3));
				my_system.AddLink(link_distRSTEER);


				// --- Left Front suspension --- 

				// ..the car right-front spindle
				spindleLF = (ChBodySceneNode*)addChBodySceneNode_easyBox(
														&my_system, msceneManager,
														8.0,
														ChVector<>(-1.3, 1 , 1),
														QUNIT, 
														ChVector<>(0.1, 0.4, 0.4) );
				spindleLF->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
				spindleLF->GetBody()->SetCollide(false);

				// ..the car left-front wheel
				wheelLF = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
														&my_system, msceneManager,
														3.0,
														ChVector<>(-1.5, 1 , 1),
														chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Z), 
														ChVector<>(0.9, 0.3, 0.9) );
				wheelLF->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
				wheelLF->GetBody()->SetCollide(true);
				wheelLF->GetBody()->SetFriction(1.0);
				wheelLF->setMaterialTexture(0,	cylinderMap);
				wheelLF->addShadowVolumeSceneNode();

				// .. create the revolute joint between the wheel and the spindle
				link_revoluteLF = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); // left, front, upper, 1
				link_revoluteLF->Initialize(wheelLF->GetBody(), spindleLF->GetBody(), 
					ChCoordsys<>(ChVector<>(-1.5, 1, 1) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
				my_system.AddLink(link_revoluteLF);

				// .. impose distance between two parts (as a massless rod with two spherical joints at the end)
				link_distLFU1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, front, upper, 1
				link_distLFU1->Initialize(truss->GetBody(), spindleLF->GetBody(), false, ChVector<>(-0.5,1.2,1.2), ChVector<>(-1.25,1.2,1));
				my_system.AddLink(link_distLFU1);
			 
				link_distLFU2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, front, upper, 2
				link_distLFU2->Initialize(truss->GetBody(), spindleLF->GetBody(), false, ChVector<>(-0.5,1.2,0.8), ChVector<>(-1.25,1.2,1));
				my_system.AddLink(link_distLFU2);

				link_distLFL1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, front, lower, 1
				link_distLFL1->Initialize(truss->GetBody(), spindleLF->GetBody(), false, ChVector<>(-0.5,0.8,1.2), ChVector<>(-1.25,0.8,1));
				my_system.AddLink(link_distLFL1);

				link_distLFL2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, front, lower, 2
				link_distLFL2->Initialize(truss->GetBody(), spindleLF->GetBody(), false, ChVector<>(-0.5,0.8,0.8), ChVector<>(-1.25,0.8,1));
				my_system.AddLink(link_distLFL2);
				
				// .. create the spring between the truss and the spindle
				link_springLF = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
				link_springLF->Initialize(truss->GetBody(), spindleLF->GetBody(), false, ChVector<>(-0.5,1.2,1.0), ChVector<>(-1.25,0.8,1));
				link_springLF->Set_SpringK(28300);
				link_springLF->Set_SpringR(80);
				my_system.AddLink(link_springLF);

				// .. create the rod for steering the wheel
				link_distLSTEER = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right steer
				link_distLSTEER->Initialize(truss->GetBody(), spindleLF->GetBody(), false, ChVector<>(-0.5,1.21,1.4), ChVector<>(-1.25,1.21,1.3));
				my_system.AddLink(link_distLSTEER);

				// --- Right Back suspension --- 

				// ..the car right-back spindle
				spindleRB = (ChBodySceneNode*)addChBodySceneNode_easyBox(
														&my_system, msceneManager,
														8.0,
														ChVector<>(1.3, 1 , -1),
														QUNIT, 
														ChVector<>(0.1, 0.4, 0.4) );
				spindleRB->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
				spindleRB->GetBody()->SetCollide(false);

				// ..the car right-back wheel
				wheelRB = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
														&my_system, msceneManager,
														3.0,
														ChVector<>(1.5, 1 , -1),
														chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Z), 
														ChVector<>(0.9, 0.3, 0.9) );
				wheelRB->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
				wheelRB->GetBody()->SetCollide(true);
				wheelRB->GetBody()->SetFriction(1.0);
					cylinderMap = mdriver->getTexture("../data/bluwhite.png");
				wheelRB->setMaterialTexture(0,	cylinderMap);
				wheelRB->addShadowVolumeSceneNode();

				// .. create the revolute joint between the wheel and the spindle
				link_revoluteRB = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); // right, back, upper, 1
				link_revoluteRB->Initialize(wheelRB->GetBody(), spindleRB->GetBody(), 
					ChCoordsys<>(ChVector<>(1.5, 1, -1) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
				my_system.AddLink(link_revoluteRB);

				// .. create the motor transmission joint between the wheel and the truss (assuming small changes of alignment)
				link_engineR = ChSharedPtr<ChLinkEngine>(new ChLinkEngine); 
				link_engineR->Initialize(wheelRB->GetBody(), truss->GetBody(), 
					ChCoordsys<>(ChVector<>(1.5, 1, -1) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
				link_engineR->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_CARDANO); // approx as a double Rzeppa joint
				link_engineR->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
				my_system.AddLink(link_engineR);

				// .. impose distance between two parts (as a massless rod with two spherical joints at the end)
				link_distRBU1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, back, upper, 1
				link_distRBU1->Initialize(truss->GetBody(), spindleRB->GetBody(), false, ChVector<>(0.5,1.2,-1.2), ChVector<>(1.25,1.2,-1));
				my_system.AddLink(link_distRBU1);
			 
				link_distRBU2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, back, upper, 2
				link_distRBU2->Initialize(truss->GetBody(), spindleRB->GetBody(), false, ChVector<>(0.5,1.2,-0.8), ChVector<>(1.25,1.2,-1));
				my_system.AddLink(link_distRBU2);

				link_distRBL1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, back, lower, 1
				link_distRBL1->Initialize(truss->GetBody(), spindleRB->GetBody(), false, ChVector<>(0.5,0.8,-1.2), ChVector<>(1.25,0.8,-1));
				my_system.AddLink(link_distRBL1);

				link_distRBL2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right, back, lower, 2
				link_distRBL2->Initialize(truss->GetBody(), spindleRB->GetBody(), false, ChVector<>(0.5,0.8,-0.8), ChVector<>(1.25,0.8,-1));
				my_system.AddLink(link_distRBL2);
				
				// .. create the spring between the truss and the spindle
				link_springRB = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
				link_springRB->Initialize(truss->GetBody(), spindleRB->GetBody(), false, ChVector<>(0.5,1.2,-1.0), ChVector<>(1.25,0.8,-1));
				link_springRB->Set_SpringK(28300);
				link_springRB->Set_SpringR(80);
				my_system.AddLink(link_springRB);

				// .. create the rod for avoid the steering of the wheel
				link_distRBlat = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right rod
				link_distRBlat->Initialize(truss->GetBody(), spindleRB->GetBody(), false, ChVector<>(0.5,1.21,-1.4), ChVector<>(1.25,1.21,-1.3));
				my_system.AddLink(link_distRBlat);


				// --- Left Back suspension --- 

				// ..the car right-back spindle
				spindleLB = (ChBodySceneNode*)addChBodySceneNode_easyBox(
														&my_system, msceneManager,
														8.0,
														ChVector<>(-1.3, 1 , -1),
														QUNIT, 
														ChVector<>(0.1, 0.4, 0.4) );
				spindleLB->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
				spindleLB->GetBody()->SetCollide(false);

				// ..the car left-back wheel
				wheelLB = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
														&my_system, msceneManager,
														3.0,
														ChVector<>(-1.5, 1 , -1),
														chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Z), 
														ChVector<>(0.9, 0.3, 0.9) );
				wheelLB->GetBody()->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
				wheelLB->GetBody()->SetCollide(true);
				wheelLB->GetBody()->SetFriction(1.0);
				wheelLB->setMaterialTexture(0,	cylinderMap);
				wheelLB->addShadowVolumeSceneNode();

				// .. create the revolute joint between the wheel and the spindle
				link_revoluteLB = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); // left, back, upper, 1
				link_revoluteLB->Initialize(wheelLB->GetBody(), spindleLB->GetBody(), 
					ChCoordsys<>(ChVector<>(-1.5, 1, -1) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
				my_system.AddLink(link_revoluteLB);

				// .. create the motor transmission joint between the wheel and the truss (assuming small changes of alignment)
				link_engineL = ChSharedPtr<ChLinkEngine>(new ChLinkEngine); 
				link_engineL->Initialize(wheelLB->GetBody(), truss->GetBody(), 
					ChCoordsys<>(ChVector<>(-1.5, 1, -1) , chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Y)) );
				link_engineL->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_CARDANO); // approx as a double Rzeppa joint
				link_engineL->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
				my_system.AddLink(link_engineL);

				// .. impose distance between two parts (as a massless rod with two spherical joints at the end)
				link_distLBU1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, front, upper, 1
				link_distLBU1->Initialize(truss->GetBody(), spindleLB->GetBody(), false, ChVector<>(-0.5,1.2,-1.2), ChVector<>(-1.25,1.2,-1));
				my_system.AddLink(link_distLBU1);
			 
				link_distLBU2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, back, upper, 2
				link_distLBU2->Initialize(truss->GetBody(), spindleLB->GetBody(), false, ChVector<>(-0.5,1.2,-0.8), ChVector<>(-1.25,1.2,-1));
				my_system.AddLink(link_distLBU2);

				link_distLBL1 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, back, lower, 1
				link_distLBL1->Initialize(truss->GetBody(), spindleLB->GetBody(), false, ChVector<>(-0.5,0.8,-1.2), ChVector<>(-1.25,0.8,-1));
				my_system.AddLink(link_distLBL1);

				link_distLBL2 = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // left, back, lower, 2
				link_distLBL2->Initialize(truss->GetBody(), spindleLB->GetBody(), false, ChVector<>(-0.5,0.8,-0.8), ChVector<>(-1.25,0.8,-1));
				my_system.AddLink(link_distLBL2);
				
				// .. create the spring between the truss and the spindle
				link_springLB = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
				link_springLB->Initialize(truss->GetBody(), spindleLB->GetBody(), false, ChVector<>(-0.5,1.2,-1.0), ChVector<>(-1.25,0.8,-1));
				link_springLB->Set_SpringK(28300);
				link_springLB->Set_SpringR(80);
				my_system.AddLink(link_springLB);

				// .. create the rod for avoid the steering of the wheel
				link_distLBlat = ChSharedPtr<ChLinkDistance>(new ChLinkDistance); // right 
				link_distLBlat->Initialize(truss->GetBody(), spindleLB->GetBody(), false, ChVector<>(-0.5,1.21,-1.4), ChVector<>(-1.25,1.21,-1.3));
				my_system.AddLink(link_distLBlat);

			}

		// Delete the car object, deleting also all bodies corresponding to
		// the various parts and removing them from the physical system.  Also
		// removes constraints from the system.
	~MySimpleCar()
			{
				ChSystem* mysystem = spindleRF->GetBody()->GetSystem(); // trick to get the system here				
					// When a ChBodySceneNode is removed via ->remove() from Irrlicht 3D scene manager,
					// it is also automatically removed from the ChSystem (the ChSystem::RemoveBody() is
					// automatically called at Irrlicht node deletion - see ChBodySceneNode.h ).

					// For links, just remove them from the ChSystem using ChSystem::RemoveLink()
				mysystem->RemoveLink(link_revoluteRF);				
				mysystem->RemoveLink(link_distRFU1);
				mysystem->RemoveLink(link_distRFU2);
				mysystem->RemoveLink(link_distRFL1);
				mysystem->RemoveLink(link_distRFL2);
				mysystem->RemoveLink(link_springRF);
				mysystem->RemoveLink(link_distRSTEER);

				mysystem->RemoveLink(link_revoluteLF);				
				mysystem->RemoveLink(link_distLFU1);
				mysystem->RemoveLink(link_distLFU2);
				mysystem->RemoveLink(link_distLFL1);
				mysystem->RemoveLink(link_distLFL2);
				mysystem->RemoveLink(link_springLF);
				mysystem->RemoveLink(link_distLSTEER);

				mysystem->RemoveLink(link_revoluteRB);				
				mysystem->RemoveLink(link_distRBU1);
				mysystem->RemoveLink(link_distRBU2);
				mysystem->RemoveLink(link_distRBL1);
				mysystem->RemoveLink(link_distRBL2);
				mysystem->RemoveLink(link_springRB);
				mysystem->RemoveLink(link_distRBlat);
				mysystem->RemoveLink(link_engineR);

				mysystem->RemoveLink(link_revoluteLB);				
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
			}

		// This can be used, at each time step, to compute the actual value of torque
		// transmitted to the wheels, according to gas throttle / speed / gear value.
		// The following is a very simplified model (the torque curve of the motor is linear
		// and no latency or inertial or clutch effects in gear train are considered.)
	double ComputeWheelTorque()
			{
				// Assume clutch is never used. Given the kinematics of differential,
				// the speed of the engine transmission shaft is the average of the two wheel speeds,
				// multiplied the conic gear transmission ratio inversed:
				double shaftspeed = (1.0/this->conic_tau) * 0.5 *
					(this->link_engineL->Get_mot_rot_dt()+this->link_engineR->Get_mot_rot_dt());
				// The motorspeed is the shaft speed multiplied by gear ratio inversed:
				double motorspeed = (1.0/this->gear_tau)*shaftspeed;
				// The torque depends on speed-torque curve of the motor: here we assume a 
				// very simplified model a bit like in DC motors:
				double motortorque = max_motor_torque - motorspeed*(max_motor_torque/max_motor_speed) ;
				// Motor torque is linearly modulated by throttle gas value:
				motortorque = motortorque *  this->throttle;
				// The torque at motor shaft:
				double shafttorque =  motortorque * (1.0/this->gear_tau);
				// The torque at wheels - for each wheel, given the differential transmission, 
				// it is half of the shaft torque  (multiplied the conic gear transmission ratio)
				double singlewheeltorque = 0.5 * shafttorque * (1.0/this->conic_tau);
				// Set the wheel torque in both 'engine' links, connecting the wheels to the truss;
				this->link_engineL->Get_tor_funct()->Set_yconst(singlewheeltorque);
				this->link_engineR->Get_tor_funct()->Set_yconst(singlewheeltorque);
				//debug:print infos on screen:
				   //GetLog() << "motor torque="<< motortorque<< "  speed=" << motorspeed << "  wheel torqe=" << singlewheeltorque <<"\n";
				// If needed, return also the value of wheel torque:
				return singlewheeltorque;
			}
};



// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface (the interface will
// be created with the basic -yet flexible- platform
// independent toolset of Irrlicht).

class MyEventReceiver : public IEventReceiver
{
public:

	MyEventReceiver(ChSystem* asystem,  
					IrrlichtDevice *adevice,
					MySimpleCar* acar)
			{
				// store pointer to physical system & other stuff so we can tweak them by user keyboard
				msystem = asystem;
				mdevice = adevice;
				mcar    = acar;

				adevice->setEventReceiver(this);

				// ..add a GUI slider to control gas throttle via mouse
				scrollbar_throttle = mdevice->getGUIEnvironment()->addScrollBar(
								true, rect<s32>(10, 85, 150, 100), 0, 100);
				scrollbar_throttle->setMax(100); 
				scrollbar_throttle->setPos(0);
				text_throttle = mdevice->getGUIEnvironment()->addStaticText(
							L"Throttle", rect<s32>(150,85,250,100), false);

				// ..add a GUI slider to control steering via mouse
				scrollbar_steer = mdevice->getGUIEnvironment()->addScrollBar(
								true, rect<s32>(10, 105, 150, 120), 0, 101);
				scrollbar_steer->setMax(100); 
				scrollbar_steer->setPos(50);

				// ..add a GUI text and GUI slider to control the stiffness
				scrollbar_FspringK = mdevice->getGUIEnvironment()->addScrollBar(
								true, rect<s32>(10, 125, 150, 140), 0, 102);
				scrollbar_FspringK->setMax(100); 
				scrollbar_FspringK->setPos(50 + 50.0*(acar->link_springRF->Get_SpringK()-80000.0)/60000.0  );
				text_FspringK = mdevice->getGUIEnvironment()->addStaticText(
								L"Spring K [N/m]:", rect<s32>(150,125,250,140), false);

				// ..add a GUI text and GUI slider to control the damping
				scrollbar_FdamperR = mdevice->getGUIEnvironment()->addScrollBar(
								true, rect<s32>(10, 145, 150, 160), 0, 103);
				scrollbar_FdamperR->setMax(100); 
				scrollbar_FdamperR->setPos(50 + 50.0*(acar->link_springRF->Get_SpringR()-800.0)/800.0  );
				text_FdamperR = mdevice->getGUIEnvironment()->addStaticText(
								L"Damper R [Ns/m]:", rect<s32>(150,145,250,160), false);

				// ..add a GUI text and GUI slider to control the original undeformed spring length
				scrollbar_FspringL = mdevice->getGUIEnvironment()->addScrollBar(
								true, rect<s32>(10, 165, 150, 180), 0, 104);
				scrollbar_FspringL->setMax(100); 
				scrollbar_FspringL->setPos(50 + 50.0*(acar->link_springRF->Get_SpringRestLenght()-0.9)/0.1  );
				text_FspringL = mdevice->getGUIEnvironment()->addStaticText(
								L"Spring L [m]:", rect<s32>(150,165,250,180), false);
			}

	bool OnEvent(const SEvent& event)
			{

				// check if user moved the sliders with mouse..
				if (event.EventType == EET_GUI_EVENT)
				{
					s32 id = event.GUIEvent.Caller->getID();
					IGUIEnvironment* env = mdevice->getGUIEnvironment();

					switch(event.GUIEvent.EventType)
					{
					case EGET_SCROLL_BAR_ChANGED:
							if (id == 101) // id of 'steer' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								double newsteer = 0.18*( ((double)(pos-50))/50.0 );
								// set the steering, moving horizontally the endpoints of the steer rod endpoint on truss.
								this->mcar->link_distRSTEER->SetEndPoint1Rel(ChVector<>( 0.5+newsteer,0.21,1.4));
								this->mcar->link_distLSTEER->SetEndPoint1Rel(ChVector<>(-0.5+newsteer,0.21,1.4));
							}
							if (id == 102) // id of 'spring stiffness' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								double newstiff = 80000 + 60000*( ((double)(pos-50))/50.0 );
								// set the stiffness of all 4 springs
								this->mcar->link_springRF->Set_SpringK(newstiff);
								this->mcar->link_springLF->Set_SpringK(newstiff);
								this->mcar->link_springRB->Set_SpringK(newstiff);
								this->mcar->link_springLB->Set_SpringK(newstiff);

								// show stiffness as formatted text in interface screen
								char message[50]; sprintf(message,"Spring K [N/m]: %g",newstiff);
								text_FspringK->setText(core::stringw(message).c_str());
							}
							if (id == 103) // id of 'damping' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								double newdamping = 800 + 800*( ((double)(pos-50))/50.0 );
								// set the damping of all 4 springs
								this->mcar->link_springRF->Set_SpringR(newdamping);
								this->mcar->link_springLF->Set_SpringR(newdamping);
								this->mcar->link_springRB->Set_SpringR(newdamping);
								this->mcar->link_springLB->Set_SpringR(newdamping);

								// show stiffness as formatted text in interface screen
								char message[50]; sprintf(message,"Damping R [Ns/m]: %g",newdamping);
								text_FdamperR->setText(core::stringw(message).c_str());
							}
							if (id == 104) // id of 'spring rest length' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								double newlength = 0.9 + 0.1*( ((double)(pos-50))/50.0 );
								// set the rest length of all 4 springs
								this->mcar->link_springRF->Set_SpringRestLenght(newlength);
								this->mcar->link_springLF->Set_SpringRestLenght(newlength);
								this->mcar->link_springRB->Set_SpringRestLenght(newlength);
								this->mcar->link_springLB->Set_SpringRestLenght(newlength);

								// show stiffness as formatted text in interface screen
								char message[50]; sprintf(message,"Spring L [m]: %g",newlength);
								text_FspringL->setText(core::stringw(message).c_str());
							}
							if (id == 100) // id of 'throttle' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								double newthrottle =  ((double)(pos))/100.0 ;
								// Set the throttle value of car (the torque transmitted
								// to wheels depends on throttle, speed, transmission gear, so 
								// it will sent to the link_engineR and link_engineL only when
								// computed by MySimplifiedCar::ComputeWheelTorque(),
								this->mcar->throttle=newthrottle;
							}
					break;
					}
					
				} 

				return false;
			}

private:
	ChSystem*       msystem;
	IrrlichtDevice* mdevice;
	MySimpleCar*    mcar;

	IGUIScrollBar*  scrollbar_steer;
	IGUIStaticText* text_FspringK;
	IGUIScrollBar*  scrollbar_FspringK;
	IGUIStaticText* text_FdamperR;
	IGUIScrollBar*  scrollbar_FdamperR;
	IGUIStaticText* text_FspringL;
	IGUIScrollBar*  scrollbar_FspringL;
	IGUIStaticText* text_throttle;
	IGUIScrollBar*  scrollbar_throttle;
};



//
// This is the program which is executed
//

int main(int argc, char* argv[])
{

	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed.
	DLL_CreateGlobals();

	// Create the IRRLICHT context (device, etc.)
	IrrlichtDevice* device = createDevice(video::EDT_DIRECT3D9, 
							core::dimension2d<u32>(800, 600),	// resolution
							32,									// 32 bit depth 
							false,								// full screen
							true);								// do shadows (might be slow on old PC!)
	if (device == 0)
	{
		GetLog() << "Cannot use DirectX - switch to OpenGL \n"; 
		device = createDevice(video::EDT_OPENGL, core::dimension2d<u32>(640, 480));
		if (!device) return 1;
	}

	device->setWindowCaption(L"Modeling a simplified suspension of a car, using a spring-damper");

	IVideoDriver* driver           = device->getVideoDriver();
	ISceneManager*	 msceneManager = device->getSceneManager();
	IGUIEnvironment* guienv        = device->getGUIEnvironment();

 
	// Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(device);
	ChIrrWizard::add_typical_Sky(device);
	ChIrrWizard::add_typical_Lights(device);
	ChIrrWizard::add_typical_Camera(device, core::vector3df(0,0,-6));


    // 
	// HERE YOU CREATE THE MEChANICAL SYSTEM OF CHRONO...
	// 


	// 1- Create a ChronoENGINE physical system: all bodies and constraints
	//    will be handled by this ChSystem object.
	ChSystem my_system;
 
	// 2- Create the rigid bodies of the simpified car suspension mechanical system
	//   maybe setting position/mass/inertias of
	//   their center of mass (COG) etc.
	
	// ..the world
	ChBodySceneNode* my_ground = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&my_system, msceneManager,
											1.0,
											ChVector<>(0,-1,0),
											QUNIT, 
											ChVector<>(60,2,60) );
	my_ground->GetBody()->SetBodyFixed(true);
	my_ground->GetBody()->SetCollide(true);
	my_ground->GetBody()->SetSfriction(1.0);
	my_ground->GetBody()->SetKfriction(1.0);
	video::ITexture* groundMap = driver->getTexture("../data/blu.png");
	my_ground->setMaterialTexture(0,groundMap);

	// ..some obstacles on the ground:
	for (int i=0; i<6; i++)
	{
		ChBodySceneNode* my_obstacle = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&my_system, msceneManager,
											3.0,
											ChVector<>(20*ChRandom(),2, 20*ChRandom()),
											QUNIT, 
											ChVector<>(1,0.08,0.5) );
	}
 
	// ..the car (this class - see above - is a 'set' of bodies and links, automatically added at creation)
	MySimpleCar* mycar = new MySimpleCar(my_system, msceneManager, driver);


	// 
	// CREATE A CUSTOM MATERIAL COMBINER
	//

	//  Suppose you want that some places have different friction coefficient values,
	//  how can you do this? By default, friction is the average of friction coefficient
	//  of the two bodies in contact, but you can create an 'callback object' inherited from the
	//  a ChCustomCollisionPointCallback class. This will be called per each contact point, and
	//  it can modify the friction as in the very simple example below:

	class MyContactCallback : public ChSystem::ChCustomCollisionPointCallback
	{
		public:	virtual void ContactCallback(
								const collision::ChCollisionInfo& mcontactinfo, ///< get info about contact (cannot change it)				
								ChMaterialCouple&  material )			  		///< you can modify this!	
		{
			if (mcontactinfo.vpA.x > 0)
				material.static_friction = 0.7; // On the right of the plane, less friction...
			else
				material.static_friction = 1.0; // On the left of the plane, more friction...
		};
	};

	MyContactCallback mycontact_callback;  // create the callback object
	my_system.SetCustomCollisionPointCallback(&mycontact_callback);	// tell the system to use that callback.



	//
	// USER INTERFACE
	//
	 
	// Create some graphical-user-interface (GUI) items to show on the screen.
	// This requires an event receiver object -see above.
	MyEventReceiver receiver(&my_system, device, mycar);



	//
	// SETTINGS 
	// 	

	my_system.SetIterLCPmaxItersSpeed(20); // the higher, the easier to keep the constraints 'mounted'.

	//my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_GPU); 
	//my_system.SetIterLCPomega(0.15); 

	//
	// THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
	//


	// This will help choosing an integration step which matches the
	// real-time step of the simulation..
	ChRealtimeStepTimer m_realtime_timer;

	while(device->run())
	{ 
		// Irrlicht must prepare frame to draw
		driver->beginScene(true, true, SColor(255,140,161,192));
	
		// Irrlicht now draws simple lines in 3D world representing a 
		// skeleton of the mechanism, in this instant:
		//
		// .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
		msceneManager->drawAll();

		// .. draw a grid (rotated so that it's horizontal)
		ChIrrTools::drawGrid(driver, 2, 2, 30,30, 
			ChCoordsys<>(ChVector<>(0,0.01,0), Q_from_AngX(CH_C_PI_2) ),
			video::SColor(255, 80,130,130), true);

		// .. draw GUI user interface items (sliders, buttons) belonging to Irrlicht screen, if any
		guienv->drawAll();

		// .. draw the distance constraints (the massless rods) as simplified lines
		std::list<chrono::ChLink*>::iterator iterlink =  my_system.Get_linklist()->begin();
		while(iterlink !=  my_system.Get_linklist()->end())
		{
			if (ChLinkDistance* mylinkdis = ChDynamicCast(ChLinkDistance,(*iterlink)))
				ChIrrTools::drawSegment(driver, 
					mylinkdis->GetEndPoint1Abs(), 
					mylinkdis->GetEndPoint2Abs(),
					video::SColor(255,   0,20,0), true);
			iterlink++;
		}

		// .. draw the spring constraints as simplified spring helix
		iterlink =  my_system.Get_linklist()->begin();
		while(iterlink !=  my_system.Get_linklist()->end())
		{
			if (ChLinkSpring* mylinkspri = ChDynamicCast(ChLinkSpring,(*iterlink)))
				ChIrrTools::drawSpring(driver, 0.03, 
					mylinkspri->GetEndPoint1Abs(),
					mylinkspri->GetEndPoint2Abs(),
					video::SColor(255,   150,20,20),   80,  5,  true);
			iterlink++;
		}
		

		// The torque applied to wheels, using the ChLinkEngine links between 
		// wheels and truss, depends on many parameters (gear, throttle, etc):
		mycar->ComputeWheelTorque();

		// HERE CHRONO INTEGRATION IS PERFORMED: THE 
		// TIME OF THE SIMULATION ADVANCES FOR A SINGLE
		// STEP:
		
		my_system.DoStepDynamics( m_realtime_timer.SuggestSimulationStep(0.005) );

		// Irrlicht must finish drawing the frame
		driver->endScene(); 
	}

	if (mycar) delete mycar;	

	// This safely delete every Irrlicht item..
	device->drop();


	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}


