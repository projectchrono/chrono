/*
Author: Charles Ricchio

This is the basic vehicle class, which all vehicles should inherit from
*/

#pragma once

#include <chrono_ogre/Core/ChOgreApplication.h>
#include <chrono_ogre/Graphics/ChOgreBody.h>

namespace VehicleEnvironment {

	class VEVehicle {

	public:

		VEVehicle();
		VEVehicle(ChOgre::ChOgreApplication* App);
		~VEVehicle();

		virtual void setApp(ChOgre::ChOgreApplication* App);
		virtual void build(const chrono::ChVector<>& Pos) =0;
		virtual void update() = 0;
		virtual void reset(const chrono::ChVector<>& Pos) =0;
		virtual void shift(uint8_t gear) =0; // 0 should be reverse
		virtual void brake() =0;

		double throttle;
		double steer;

		uint8_t gear;

	protected:

		ChOgre::ChOgreApplication* m_pApp;

	private:



	};

}