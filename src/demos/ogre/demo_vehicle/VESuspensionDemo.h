/*
Author: Charles Ricchio

The Chrono suspension demo implemented with the VEVehicle class
*/

#pragma once

#include "VEVehicle.h"

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

        virtual chrono::ChSharedPtr<ChBody> getChassis();
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

		chrono::ChSharedPtr<chrono::ChLinkLockRevolute> link_revoluteRF;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distRFU1;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distRFU2;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distRFL1;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distRFL2;
		chrono::ChSharedPtr<chrono::ChLinkSpring>   link_springRF;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distRSTEER;

		ChOgre::ChOgreBodyHandle spindleLF;
		ChOgre::ChOgreBodyHandle wheelLF;

		chrono::ChSharedPtr<chrono::ChLinkLockRevolute> link_revoluteLF;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distLFU1;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distLFU2;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distLFL1;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distLFL2;
		chrono::ChSharedPtr<chrono::ChLinkSpring>   link_springLF;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distLSTEER;

		ChOgre::ChOgreBodyHandle spindleRB;
		ChOgre::ChOgreBodyHandle wheelRB;

		chrono::ChSharedPtr<chrono::ChLinkLockRevolute> link_revoluteRB;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distRBU1;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distRBU2;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distRBL1;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distRBL2;
		chrono::ChSharedPtr<chrono::ChLinkSpring>   link_springRB;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distRBlat;
		chrono::ChSharedPtr<chrono::ChLinkEngine>   link_engineL;

		ChOgre::ChOgreBodyHandle spindleLB;
		ChOgre::ChOgreBodyHandle wheelLB;

		chrono::ChSharedPtr<chrono::ChLinkLockRevolute> link_revoluteLB;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distLBU1;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distLBU2;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distLBL1;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distLBL2;
		chrono::ChSharedPtr<chrono::ChLinkSpring>   link_springLB;
		chrono::ChSharedPtr<chrono::ChLinkDistance> link_distLBlat;
		chrono::ChSharedPtr<chrono::ChLinkEngine>   link_engineR;

	private:



	};

}