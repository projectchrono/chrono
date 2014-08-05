#ifndef VEHICLE_CAR_H
#define VEHICLE_CAR_H

///////////////////////////////////////////////////
//
//   Vehicle 'car object'
//
///////////////////////////////////////////////////
 
 
#include "physics/ChSystem.h"
#include "physics/ChLinkDistance.h"
#include "physics/ChLinkBrake.h"
#include "unit_IRRLICHT/ChBodySceneNode.h"
#include "unit_IRRLICHT/ChBodySceneNodeTools.h" 

#include <irrlicht.h>
#include <irrKlang.h>


// Use the namespace of Chrono

using namespace chrono;

// Use the main namespaces of Irrlicht

using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

// Use also Irrklang for sound effects

using namespace irrklang;



// First of all, define a class for the 'car' (that is, a set of
// bodies and links which are grouped within this class; so it is 
// easier to manage data structures in this example).

class MySimpleCar {
public:
		// THE DATA

	double throttle; // actual value 0...1 of gas throttle.
	double conic_tau; // the transmission ratio of the conic gears at the rear axle
	double gear_tau; // the actual tau of the gear
	double gears[6]; // all the gears
	int    actual_gear; // the actual gear
	//double max_motor_torque; // the max torque of the motor [Nm];
	//double max_motor_speed;	 // the max rotation speed of the motor [rads/s]
	ChFunction_Recorder torque_curve; // the torque(speed) of motor, with speed in [rad/s] and roque in [Nm]
	double motorspeed;   // these will be updated at ComputeWheelTorque() ..
	double motortorque;

	double passo; //identifico variabile passo
	double carr;  //identifico la variabile carreggiata/2



	double wanted_steer;
	double actual_steer;
	double max_steer_speed;

	double convergenza_anteriore;	// solo calcolo e lettura
	double convergenza_posteriore;	// solo calcolo e lettura

	double max_brake_torque_post;
	double max_brake_torque_ant;
	double braking;

		// The parts making the car, as 3d Irrlicht scene nodes, each containing
		// the ChBody object
			// .. truss:
	ChBodySceneNode* truss;
	ChVector<> truss_COG; 
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
		
	ChSharedPtr<ChLinkBrake> link_brakeRB;
	ChSharedPtr<ChLinkBrake> link_brakeLB;
	ChSharedPtr<ChLinkBrake> link_brakeRF;
	ChSharedPtr<ChLinkBrake> link_brakeLF;

	ChFunction_Oscilloscope	speed_recorder;
	ChFunction_Oscilloscope	motorspeed_recorder;

	irrklang::ISoundEngine* sound_engine;   // Sound player
	irrklang::ISound* car_sound;			// Sound 

		// THE FUNCTIONS

		// Build and initialize the car, creating all bodies corresponding to
		// the various parts and adding them to the physical system - also creating
		// and adding constraints to the system.
	MySimpleCar(ChSystem&  my_system,	///< the chrono::engine physical system 
				irr::scene::ISceneManager* msceneManager, ///< the Irrlicht scene manager for 3d shapes
				irr::video::IVideoDriver* mdriver	///< the Irrlicht video driver
				);

		// Delete the car object, deleting also all bodies corresponding to
		// the various parts and removing them from the physical system.  Also
		// removes constraints from the system.
	~MySimpleCar();

		// This function updates many things in the car. For example
		// calls ComputeWheelTorque() to update the torque to be applied to
		// wheels, then it computes values of wheel convergence, caster, etc.
	void Update(double dt);
		
		// This can be used, at each time step, to compute the actual value of torque
		// transmitted to the wheels, according to gas throttle / speed / gear value.
		// The following is a very simplified model (the torque curve of the motor is linear
		// and no latency or inertial or clutch effects in gear train are considered.)
	double ComputeWheelTorque();

		// Set the braking (this will automatically change the actual braking value in the
		// four brakes of the four wheels). The mbraking parameter should range from 0...1
		// where 1 means max braking torque - as set with max_brake_torque_post or _ant.
	void SetBraking(double mbraking);
		// Return actual braking (in 0..1 range).
	double GetBraking() {return braking;};

		// The following are used for changing the gear, as in sequential gears.
	void ChangeGearUp();
	void ChangeGearDown();
		// The following are used to select whatever gear
	void ChangeGearN(int newgear);
		
		// Save the recorded functions to disk as X-Y files
	void SaveFunctionsToDisk();

};


#endif // end of include guard - to avoid double .h inclusion