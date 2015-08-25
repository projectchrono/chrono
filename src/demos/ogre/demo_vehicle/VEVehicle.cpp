#include "VEVehicle.h"

namespace VehicleEnvironment {

	VEVehicle::VEVehicle() {
		throttle = 0;
		steer = 0;
	}

	VEVehicle::VEVehicle(ChOgre::ChOgreApplication* App) {
		setApp(App);
		throttle = 0;
		steer = 0;
	}

	VEVehicle::~VEVehicle() {

	}

	void VEVehicle::setApp(ChOgre::ChOgreApplication* App) {
		m_pApp = App;
	}

}