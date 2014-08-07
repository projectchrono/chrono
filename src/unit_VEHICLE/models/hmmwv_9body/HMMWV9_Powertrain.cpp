// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Simple powertrain model for the HMMWV vehicle.
// - RWD only
// - trivial speed-torque curve
// - no differential
//
// =============================================================================

#include "HMMWV9_Powertrain.h"

using namespace chrono;

namespace hmmwv9 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double HMMWV9_Powertrain::m_max_torque = 2400 / 8.851;
const double HMMWV9_Powertrain::m_max_speed  = 2000;
const double HMMWV9_Powertrain::m_conic_tau = 0.2;
const double HMMWV9_Powertrain::m_gear_tau = 0.3;


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV9_Powertrain::HMMWV9_Powertrain(HMMWV9_Vehicle* car)
: ChPowertrain(car, RWD),
  m_wheelTorque(0),
  m_motorSpeed(0),
  m_motorTorque(0)
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double HMMWV9_Powertrain::GetWheelTorque(ChWheelId which) const
{
  switch (which) {
  case FRONT_LEFT:
    return 0;
  case FRONT_RIGHT:
    return 0;
  case REAR_LEFT:
    return m_wheelTorque;
  case REAR_RIGHT:
    return m_wheelTorque;
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV9_Powertrain::Update(double time,
                               double throttle)
{
  // Get wheel angular speeds.
  double wheelSpeedL = m_car->GetWheelAngSpeed(REAR_LEFT);
  double wheelSpeedR = m_car->GetWheelAngSpeed(REAR_RIGHT);

  // Assume clutch is never used. Given the kinematics of differential, the
  // speed of the engine transmission shaft is the average of the two wheel
  // speeds, multiplied the conic gear transmission ratio inversed:
  double shaftSpeed = (1.0 / m_conic_tau) * 0.5 * (wheelSpeedL + wheelSpeedR);

  // The motorspeed is the shaft speed multiplied by gear ratio inversed:
  m_motorSpeed = shaftSpeed * (1.0 / m_gear_tau);

  // The torque depends on speed-torque curve of the motor: here we assume a 
  // very simplified model a bit like in DC motors:
  m_motorTorque = m_max_torque - m_motorSpeed*(m_max_torque / m_max_speed);

  // Motor torque is linearly modulated by throttle gas value:
  m_motorTorque *= throttle;

  // The torque at motor shaft:
  double shaftTorque = m_motorTorque * (1.0 / m_gear_tau);

  // The torque at wheels - for each wheel, given the differential transmission, 
  // it is half of the shaft torque  (multiplied the conic gear transmission ratio)
  m_wheelTorque = 0.5 * shaftTorque * (1.0 / m_conic_tau);

}







// =============================================================================
// Authors: Alessandro Tasora, Radu Serban, Justin Madsen
// =============================================================================
//
// Simple powertrain model for the HMMWV vehicle.
// - based on 1D primitives of ChShaft type
// - advanced speed-torque curve
// - torque converter
// - differential
//
// =============================================================================




// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV9_Powertrain_NEW::HMMWV9_Powertrain_NEW(HMMWV9_Vehicle* car)
: ChPowertrain(car, RWD)
{
	// Initialize array of gear ratios
	this->gear_ratios.push_back(-0.1); // 0: reverse gear;
	this->gear_ratios.push_back( 0.2); // 1: 1st gear;
	this->gear_ratios.push_back( 0.3); // 2: 2nd gear;
	this->gear_ratios.push_back( 0.5); // 3: 3rd gear;
	this->current_gear=1;

	// Initialize the direction of the motor block, ie. direction of crankshaft,
	// in chassis local coords.
	// This is needed because ChShaftsBody could transfer rolling torque to the chassis.
	this->dir_motor_block = ChVector<>(1,0,0);

	// Initialize the direction of the (rear) axle, i.e. output of the conic gear pair,
	// in chassis local coords.
	// This is needed because ChShaftsBody could transfer pitch torque to the chassis.
	this->dir_axle = ChVector<>(0,1,0);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV9_Powertrain_NEW::Initialize(ChSharedPtr<ChBody> mchassis, ChSharedPtr<ChBody> mspindle_L, ChSharedPtr<ChBody> mspindle_R)
{
	assert(mchassis);
	assert(mspindle_L);
	assert(mspindle_R);
	assert(mchassis->GetSystem());
	ChSystem* my_system = mchassis->GetSystem();


	// CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
	// In this case it is the motor block. This because the ChShaftsThermalEngine
	// needs two 1dof items to apply the torque in-between them (the other will be
	// the crankshaft). In simplier models, one could just use SetShaftFixed() on
	// this object, but here we prefer to leave it free and use a ChShaftsBody
	// constraint to connect it to the car chassis (so the car can 'roll' when 
	// pressing the throttle, like in muscle cars)
	m_motorblock = ChSharedPtr<ChShaft>(new ChShaft);
	m_motorblock->SetInertia(10.5);
	my_system->Add(m_motorblock);


	// CREATE  a connection between the motor block and the 3D rigid body that
	// represents the chassis. This allows to get the effect of the car 'rolling'
	// when the longitudinal engine accelerates suddenly.
	m_motorblock_to_body = ChSharedPtr<ChShaftsBody>(new ChShaftsBody);
	m_motorblock_to_body->Initialize(
								m_motorblock,
								mchassis,
								dir_motor_block);
	my_system->Add(m_motorblock_to_body);

	// CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
	// This represents the crankshaft plus flywheel.
	m_crankshaft = ChSharedPtr<ChShaft>(new ChShaft);
	m_crankshaft->SetInertia(1.1);
	my_system->Add(m_crankshaft);
	

	// CREATE  a thermal engine model beteen motor block and crankshaft (both
	// receive the torque, but with opposite sign).
	m_engine = ChSharedPtr<ChShaftsThermalEngine>(new ChShaftsThermalEngine);
	m_engine->Initialize(
							m_crankshaft, 
							m_motorblock);
	my_system->Add(m_engine);
		// The thermal engine requires a torque curve: 
	ChSharedPtr<ChFunction_Recorder> mTw(new ChFunction_Recorder);
	mTw->AddPoint(  -5,  300);  //   [rad/s],  [Nm]  
	mTw->AddPoint(   0,  360);
	mTw->AddPoint( 200,  440);
	mTw->AddPoint( 400,  480);
	mTw->AddPoint( 500,  440);
	mTw->AddPoint( 600,  240);
	mTw->AddPoint( 700, -200);   // torque curve must be defined beyond max speed too - engine might be 'pulled' 
	m_engine->SetTorqueCurve(mTw);


	// CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
	// This represents the shaft that collects all inertias from torque converter to the gear.
	m_shaft_ingear = ChSharedPtr<ChShaft>(new ChShaft);
	m_shaft_ingear->SetInertia(0.3);
	my_system->Add(m_shaft_ingear);


	// CREATE a torque converter and connect the shafts: 
	// A (input),B (output), C(truss stator). 
	// The input is the m_crankshaft, output is m_shaft_ingear; for stator, reuse the motor block 1D item.
	m_torqueconverter = ChSharedPtr<ChShaftsTorqueConverter>(new ChShaftsTorqueConverter);
	m_torqueconverter->Initialize(
							m_crankshaft, 
							m_shaft_ingear, 
							m_motorblock);
	my_system->Add(m_torqueconverter);
	 // To complete the setup of the torque converter, a capacity factor curve is needed:
	ChSharedPtr<ChFunction_Recorder> mK(new ChFunction_Recorder);
	mK->AddPoint(0.0,  15);
	mK->AddPoint(0.25, 15);
	mK->AddPoint(0.50, 15);
	mK->AddPoint(0.75, 16);
	mK->AddPoint(0.90, 18);
	mK->AddPoint(1.00, 35);
	m_torqueconverter->SetCurveCapacityFactor(mK);
	 // To complete the setup of the torque converter, a torque ratio curve is needed:	
	ChSharedPtr<ChFunction_Recorder> mT(new ChFunction_Recorder);
	mT->AddPoint(0.0,  2.00);
	mT->AddPoint(0.25, 1.80);
	mT->AddPoint(0.50, 1.50);
	mT->AddPoint(0.75, 1.15);
	mT->AddPoint(1.00, 1.00);
	m_torqueconverter->SetCurveTorqueRatio(mT);


	// CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
	// This represents the shaft that collects all inertias from the gear to the differential bevel gear.
	m_shaft_outgear = ChSharedPtr<ChShaft>(new ChShaft);
	m_shaft_outgear->SetInertia(0.5);
	my_system->Add(m_shaft_outgear);


	// CREATE a gearbox, i.e a transmission ratio constraint between two
	// shafts. Note that differently from the basic ChShaftsGear, this also provides
	// the possibility of transmitting a reaction torque to the box (the truss).
	m_gears = ChSharedPtr<ChShaftsGearbox>(new ChShaftsGearbox);
	m_gears->Initialize(	m_shaft_ingear, 
							m_shaft_outgear, 
							mchassis, 
							this->dir_motor_block);
	m_gears->SetTransmissionRatio(this->gear_ratios[this->current_gear]);
	my_system->Add(m_gears);
	

	// CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
	// This represents the inertia of the rotating box of the differential.
	m_shaft_rear_differentialbox = ChSharedPtr<ChShaft>(new ChShaft);
	m_shaft_rear_differentialbox->SetInertia(0.6);
	my_system->Add(m_shaft_rear_differentialbox);


	// CREATE an angled gearbox, i.e a transmission ratio constraint between two
	// non parallel shafts. This is the case of the 90° bevel gears in the differential.
	// Note that differently from the basic ChShaftsGear, this also provides
	// the possibility of transmitting a reaction torque to the box (the truss).
	m_rear_conicalgear = ChSharedPtr<ChShaftsGearboxAngled>(new ChShaftsGearboxAngled);
	m_rear_conicalgear->Initialize(
							m_shaft_outgear,
							m_shaft_rear_differentialbox,
							mchassis,
							this->dir_motor_block,
							this->dir_axle);
	m_rear_conicalgear->SetTransmissionRatio(-0.2);
	my_system->Add(m_rear_conicalgear);


	// CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
	// This represents the inertia of the LEFT axle, exiting from the differential.
	m_shaft_rear_L_axle = ChSharedPtr<ChShaft>(new ChShaft);
	m_shaft_rear_L_axle->SetInertia(0.4);
	my_system->Add(m_shaft_rear_L_axle);
	

	// CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
	// This represents the inertia of the RIGHT axle, exiting from the differential.
	m_shaft_rear_R_axle = ChSharedPtr<ChShaft>(new ChShaft);
	m_shaft_rear_R_axle->SetInertia(0.4);
	my_system->Add(m_shaft_rear_R_axle);


	// CREATE a differential, i.e. an apicycloidal mechanism that connects three 
	// rotating members. This class of mechanisms can be simulated using 
	// ChShaftsPlanetary; a proper 'ordinary' transmission ratio t0 must be assigned according
	// to Willis formula. The case of the differential is simple: t0=-1.
	m_rear_differential = ChSharedPtr<ChShaftsPlanetary>(new ChShaftsPlanetary);
	m_rear_differential->Initialize(
							m_shaft_rear_differentialbox, // the carrier
							m_shaft_rear_L_axle, 
							m_shaft_rear_R_axle);
	m_rear_differential->SetTransmissionRatioOrdinary(-1);
	my_system->Add(m_rear_differential);


	// CREATE  a connection between the motor block and the 3D rigid body that
	// represents the LEFT spindle 
	m_shaft_rear_L_axle_to_body = ChSharedPtr<ChShaftsBody>(new ChShaftsBody);
	m_shaft_rear_L_axle_to_body->Initialize(
								m_shaft_rear_L_axle,
								mspindle_L,
								ChVector<>(0,1,0) ); // spindle rot axis on its Y
	my_system->Add(m_shaft_rear_L_axle_to_body);


	// CREATE  a connection between the motor block and the 3D rigid body that
	// represents the RIGHT spindle 
	m_shaft_rear_R_axle_to_body = ChSharedPtr<ChShaftsBody>(new ChShaftsBody);
	m_shaft_rear_R_axle_to_body->Initialize(
								m_shaft_rear_R_axle,
								mspindle_R,
								ChVector<>(0,1,0) ); // spindle rot axis on its Y
	my_system->Add(m_shaft_rear_R_axle_to_body);


	// -------
	// Finally, update the gear ratio according to the selected gear in the 
	// array of gear ratios:
	this->SetSelectedGear(1);
}


void HMMWV9_Powertrain_NEW::SetSelectedGear(int igear) 
{
	assert(igear>=0);
	assert(igear<gear_ratios.size());

	current_gear = igear;
	if (m_gears)
	{
		m_gears->SetTransmissionRatio( this->gear_ratios[igear] );
	}
}



// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double HMMWV9_Powertrain_NEW::GetWheelTorque(ChWheelId which) const
{
  switch (which) {
  case FRONT_LEFT:
    return 0;
  case FRONT_RIGHT:
    return 0;
  case REAR_LEFT:
    return 0; //this->m_shaft_rear_L_axle_to_body->GetTorqueReactionOnShaft();
  case REAR_RIGHT:
    return 0; //this->m_shaft_rear_R_axle_to_body->GetTorqueReactionOnShaft();
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV9_Powertrain_NEW::Update(double time,
                               double throttle)
{
 
	// Just update the throttle level in the thermal engine
	m_engine->SetThrottle(throttle);
}








} // end namespace hmmwv9
