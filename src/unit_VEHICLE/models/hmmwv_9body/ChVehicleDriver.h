#ifndef CHVEHICLEDRIVER_H
#define CHVEHICLEDRIVER_H

#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"

using namespace chrono;

/// Controls: Gas pedal throttle, steer angle
class ChVehicleDriver
{
private:
	// the input data
	double throttle;	// between 0-1
	double steer;		// between -1 and 1
	double wheelOmega;	// wheel angular/spin velocity, in [rad/sec]
	double last_simtime;	// sim time value at previous sim step
	// double brake;
public:
	// Constructors, destruct
	ChVehicleDriver():throttle(0), steer(0), last_simtime(-1) {}
	~ChVehicleDriver(){}

	// get the list of bodies as const refs
	const double getThrottle(){
		return this->throttle;
	}

	// set the throttle position, between 0 and 1
	void setThrottle(const double thNew) {
		if(thNew < 0 || thNew > 1) {
			// throttle setting out of range, not setting.
		} else {
			this->throttle = thNew;
		}
	}
	// get the steer position, between -1 (far left) and 1 (far right)
	const double getSteer() {
		return this->steer;
	}

	// set the steer position
	void setSteer(const double stNew) {
		if(stNew < -1 || stNew > 1) {
			// input steer position out of bounds, not setting...
		} else {
			this->steer = stNew;
		}
	}

	// at each new timestep, update 
	// rotational motion [rad/sec] applied to rear wheels
	void set_wheelOmega(double wheel_omega) {
		this->wheelOmega = wheel_omega;
	}

	const double get_wheelOmega() {
		return this->wheelOmega;
	}

};

#endif