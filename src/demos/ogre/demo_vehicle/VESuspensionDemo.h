/*
Author: Charles Ricchio

The Chrono suspension demo implemented with the VEVehicle class
*/

#pragma once

#include "VEVehicle.h"

using namespace chrono;

namespace VehicleEnvironment {

	class VESuspensionDemo : public VEVehicle {

	public:

		VESuspensionDemo();
		VESuspensionDemo(ChOgre::ChOgreApplication* App);
		~VESuspensionDemo();

		virtual void build(const chrono::ChVector<>& Pos);
		virtual void update();
		virtual void reset(const chrono::ChVector<>& Pos);

		virtual void shift(uint8_t gear);
		virtual void brake();

        virtual std::shared_ptr<ChBody> getChassis();
		virtual chrono::ChVector<> getPos();
		virtual chrono::ChQuaternion<> getRot();

		double conic_tau; // the transmission ratio of the conic gears at the rear axle
		double gear_tau; // the actual tau of the gear
		double max_motor_torque; // the max torque of the motor [Nm];
		double max_motor_speed;	 // the max rotation speed of the motor [rads/s]

	protected:

		ChOgre::ChOgreBodyHandle truss;

		ChOgre::ChOgreBodyHandle spindleRF;
		ChOgre::ChOgreBodyHandle wheelRF;

		std::shared_ptr<chrono::ChLinkLockRevolute> link_revoluteRF;
		std::shared_ptr<chrono::ChLinkDistance> link_distRFU1;
		std::shared_ptr<chrono::ChLinkDistance> link_distRFU2;
		std::shared_ptr<chrono::ChLinkDistance> link_distRFL1;
		std::shared_ptr<chrono::ChLinkDistance> link_distRFL2;
		std::shared_ptr<chrono::ChLinkSpring>   link_springRF;
		std::shared_ptr<chrono::ChLinkDistance> link_distRSTEER;

		ChOgre::ChOgreBodyHandle spindleLF;
		ChOgre::ChOgreBodyHandle wheelLF;

		std::shared_ptr<chrono::ChLinkLockRevolute> link_revoluteLF;
		std::shared_ptr<chrono::ChLinkDistance> link_distLFU1;
		std::shared_ptr<chrono::ChLinkDistance> link_distLFU2;
		std::shared_ptr<chrono::ChLinkDistance> link_distLFL1;
		std::shared_ptr<chrono::ChLinkDistance> link_distLFL2;
		std::shared_ptr<chrono::ChLinkSpring>   link_springLF;
		std::shared_ptr<chrono::ChLinkDistance> link_distLSTEER;

		ChOgre::ChOgreBodyHandle spindleRB;
		ChOgre::ChOgreBodyHandle wheelRB;

		std::shared_ptr<chrono::ChLinkLockRevolute> link_revoluteRB;
		std::shared_ptr<chrono::ChLinkDistance> link_distRBU1;
		std::shared_ptr<chrono::ChLinkDistance> link_distRBU2;
		std::shared_ptr<chrono::ChLinkDistance> link_distRBL1;
		std::shared_ptr<chrono::ChLinkDistance> link_distRBL2;
		std::shared_ptr<chrono::ChLinkSpring>   link_springRB;
		std::shared_ptr<chrono::ChLinkDistance> link_distRBlat;
		std::shared_ptr<chrono::ChLinkEngine>   link_engineL;

		ChOgre::ChOgreBodyHandle spindleLB;
		ChOgre::ChOgreBodyHandle wheelLB;

		std::shared_ptr<chrono::ChLinkLockRevolute> link_revoluteLB;
		std::shared_ptr<chrono::ChLinkDistance> link_distLBU1;
		std::shared_ptr<chrono::ChLinkDistance> link_distLBU2;
		std::shared_ptr<chrono::ChLinkDistance> link_distLBL1;
		std::shared_ptr<chrono::ChLinkDistance> link_distLBL2;
		std::shared_ptr<chrono::ChLinkSpring>   link_springLB;
		std::shared_ptr<chrono::ChLinkDistance> link_distLBlat;
		std::shared_ptr<chrono::ChLinkEngine>   link_engineR;

	private:



	};

}
