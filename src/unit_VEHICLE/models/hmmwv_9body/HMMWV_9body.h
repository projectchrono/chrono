#ifndef HMMWV_9BODY_H
#define HMMWV_9BODY_H

// C::E types
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "physics/ChLinkDistance.h"
// obsolete, replaced
// #include "ChAssetHelper_initialization.h"
#include "physics/ChBodyEasy.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <map>

// wheel class
#include "ChWheel.h"
// suspension class
#include "DoubleAarm.h"
#include "ChVehicleDriver.h"

namespace chrono {

/// HMMWV with four wheels, four lumped mass suspension elements, and a single chassis.
///	Controlled by prescribing an input throttle and steer angle to the driver class each timestep.
/// Author: Justin Madsen
class HMMWV_9body {
public:
	// driver
	ChVehicleDriver* driver;
	// Chassis
	ChSharedPtr<ChBody> chassis;
	//  right front (RF) suspension:
	DoubleAarm* suspension_RF;
	// ChSharedPtr<ChBody> wheelRF;
	ChWheel* wheelRF;

	// left front (LF) suspension:
	DoubleAarm* suspension_LF;
	// ChSharedPtr<ChBody> wheelLF;
	ChWheel* wheelLF;

	// right back (RB) suspension:
	DoubleAarm* suspension_RB;
	// ChSharedPtr<ChBody> wheelRB;
	ChWheel* wheelRB;

	// left back (LB) suspension:
	DoubleAarm* suspension_LB;
	// ChSharedPtr<ChBody> wheelLB;
	ChWheel* wheelLB;

	// use these instead of revolute joints between the spindle and wheel
	ChSharedPtr<ChLinkEngine>   link_engineL;
	ChSharedPtr<ChLinkEngine>   link_engineR;
	ChSharedPtr<ChLinkLockRevolute> spindle_joint_LF;
	ChSharedPtr<ChLinkLockRevolute> spindle_joint_RF;
	ChSharedPtr<ChLinkLockRevolute> spindle_joint_LB;
	ChSharedPtr<ChLinkLockRevolute> spindle_joint_RB;

	// @brief Build and initialize the car, creating all bodies relative to
	// the CM position, and orientation. x-positive is forward, y-positive is up.
	// @param chassisCM		global position of the chassis
	// @param chassisRot	QUNIT = x-positive is fowrard
	// @param tireMesh		use a mesh and associated collision shape
	// @param tireMeshFile		if non-empty, load the mesh file
	HMMWV_9body(ChSystem&             my_system,
	            const ChVector<>&     chassisCM = ChVector<>(0,0,0),
	            const ChQuaternion<>& chassisRot = ChQuaternion<>(1,0,0,0),
	            const bool            fixed = false,
              const std::string&    tireMeshFile = "");

	ChSharedPtr<ChBody> create_wheel(ChVector<>& mpos);

	// Delete the car object, deleting also all bodies corresponding to
	// the various parts and removing them from the physical system.  Also
	// removes constraints from the system.
	~HMMWV_9body();

	// This can be used, at each time step, to compute the actual value of torque
	// transmitted to the wheels, according to gas throttle / speed / gear value.
	// The following is a very simplified model (the torque curve of the motor is linear
	// and no latency or inertial or clutch effects in gear train are considered.)
	// Appends: currTorque
	double ComputeWheelTorque();

	// @brief update the inner location of the markers attached to the tie rod on the 
	//			front and rear suspensions
	// @return displacement on the right steer rod marker position
	double ComputeSteerDisplacement();

	// write output data this step to a text file, w/ outFileName as the prefix
	void write_OutData(const double simTime);

	// are we using a torque to drive the wheel?
	bool useTorque() {
		return this->is_torqueDriven;
	}

	// @brief if using a custom tire model, this is where you set the calculated force/moment on the wheel body
	void applyHub_FM(const std::vector<ChVector<>>& F_hub, const std::vector<ChVector<>>& M_hub);


	// helper funtions, for tire data
	ChVector<> getCM_pos(int tire_idx);
	ChQuaternion<> getCM_q(int tire_idx);
	ChVector<> getCM_RPY(int tire_idx);
	ChVector<> getCM_vel(int tire_idx);
	ChVector<> getCM_w(int tire_idx);
	ChVector<> getCM_acc(int tire_idx);
	// other helper functions
	ChVector<> getCM_pos_chassis();
	ChVector<> getCM_vel_chassis();
	// are we writing output data?
	bool writeData();
	// set the output data
	void set_writeOutData(bool out);


private:
	// THE DATA
	// double throttle; // actual value 0...1 of gas throttle. MOVED
	double conic_tau; // the transmission ratio of the conic gears at the rear axle
	double gear_tau; // the actual tau of the gear
	double max_motor_torque; // the max torque of the motor [Nm];
	double max_motor_speed;	 // the max rotation speed of the motor [rads/s]


	bool is_torqueDriven;	// drive it with a torque? else, drive with a motion
	double currTorque;		// current torque applied to wheel

	// for saving test mechanism data
	bool writeOutData;
	std::string outFilename;
	std::ofstream outFile;	// write to this outFile location
	int outWritten_nTimes;	// get init'd to 0 upon initialization, indicates how many times output data is written

	ChSystem* m_sys;


  // Mass properties
  static const double chassisMass;
  static const double spindleMass;
  static const ChVector<> chassisInertia;
  static const ChVector<> spindleInertia;


};

std::ostream& operator << (std::ostream& output, const chrono::ChVector<>& v);


}	// namespace chrono {

#endif		// HMMWV_H
