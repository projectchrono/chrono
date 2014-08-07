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

#ifndef HMMWV9_POWERTRAIN_H
#define HMMWV9_POWERTRAIN_H

#include "subsys/ChPowertrain.h"

#include "HMMWV9_Vehicle.h"

#include "physics/ChShaftsGear.h" 
#include "physics/ChShaftsGearbox.h"
#include "physics/ChShaftsGearboxAngled.h"
#include "physics/ChShaftsClutch.h"
#include "physics/ChShaftsPlanetary.h"
#include "physics/ChShaftsBody.h"
#include "physics/ChShaftsTorsionSpring.h"
#include "physics/ChShaftsTorqueConverter.h"
#include "physics/ChShaftsMotor.h"
#include "physics/ChShaftsTorque.h"
#include "physics/ChShaftsThermalEngine.h"

namespace hmmwv9 {

// Forward reference
class HMMWV9_Vehicle;

class HMMWV9_Powertrain : public chrono::ChPowertrain {
public:

  HMMWV9_Powertrain(HMMWV9_Vehicle* car);

  ~HMMWV9_Powertrain() {}

  virtual double GetWheelTorque(chrono::ChWheelId which) const;
  virtual void Update(double time, double throttle);

  double GetMotorSpeed() const { return m_motorSpeed; }
  double GetMotorTorque() const { return m_motorTorque; }

private:

  double  m_motorSpeed;
  double  m_motorTorque;
  double  m_wheelTorque;

  static const double m_conic_tau;   // the transmission ratio of the conic gears at the rear axle
  static const double m_gear_tau;    // the actual tau of the gear
  static const double m_max_torque;  // the max torque of the motor [Nm];
  static const double m_max_speed;   // the max rotation speed of the motor [rads/s]
};


////////// NEW POWERTRAIN BASED ON ChShaft OBJECTS


class HMMWV9_Powertrain_NEW : public chrono::ChPowertrain {
public:

  HMMWV9_Powertrain_NEW(HMMWV9_Vehicle* car);

  ~HMMWV9_Powertrain_NEW() {}

	/// To be called after creation, to create all the wrapped ChShaft objects 
	/// and their constraints, torques etc. 
  virtual void Initialize(chrono::ChSharedPtr<chrono::ChBody> mchassis,
						  chrono::ChSharedPtr<chrono::ChBody> mspindle_L, 
					      chrono::ChSharedPtr<chrono::ChBody> mspindle_R);

	/// Use this function to shift from one gear to another.
	/// A zero latency shift is assumed.
	/// Note, index starts from 0.
  void SetSelectedGear(int igear);

	/// Tell which is the actual gear number.
  int  GetSelectedGear() {return current_gear;}

	/// Use this if you want to change the setup of the gears
	/// i.e. the transmission ratios of the various gears.
	/// Suggestion: use 0 for reverse, 1,2,3... for others.
  std::vector<double>& GearRatios() {return gear_ratios;}


	// NOTE. THE FOLLOWING FUNCTIONS COULD BE REMOVED
	// It could be more straightforward to access directly the public: data below.
  virtual double GetWheelTorque(chrono::ChWheelId which) const;
  virtual void Update(double time, double throttle);
  double GetMotorSpeed()  { return  m_crankshaft->GetPos_dt();}
  double GetMotorTorque() { return  m_engine->GetTorqueReactionOn1(); }

public:
	chrono::ChSharedPtr<chrono::ChShaftsBody>				m_motorblock_to_body;
	chrono::ChSharedPtr<chrono::ChShaft>					m_motorblock;
	chrono::ChSharedPtr<chrono::ChShaftsThermalEngine>		m_engine;
	chrono::ChSharedPtr<chrono::ChShaft>					m_crankshaft;
	chrono::ChSharedPtr<chrono::ChShaftsTorqueConverter>	m_torqueconverter;
	chrono::ChSharedPtr<chrono::ChShaft>					m_shaft_ingear;
	chrono::ChSharedPtr<chrono::ChShaftsGearbox>			m_gears;
	chrono::ChSharedPtr<chrono::ChShaft>					m_shaft_outgear;
	chrono::ChSharedPtr<chrono::ChShaftsGearboxAngled>		m_rear_conicalgear;
	chrono::ChSharedPtr<chrono::ChShaft>					m_shaft_rear_differentialbox;
	chrono::ChSharedPtr<chrono::ChShaftsPlanetary>			m_rear_differential;
	chrono::ChSharedPtr<chrono::ChShaft>					m_shaft_rear_L_axle;
	chrono::ChSharedPtr<chrono::ChShaft>					m_shaft_rear_R_axle;
	chrono::ChSharedPtr<chrono::ChShaftsBody>				m_shaft_rear_L_axle_to_body;
	chrono::ChSharedPtr<chrono::ChShaftsBody>				m_shaft_rear_R_axle_to_body;

private:
	std::vector<double> gear_ratios;
	int	current_gear;
	chrono::ChVector<> dir_motor_block;
	chrono::ChVector<> dir_axle;
};






} // end namespace hmmwv9


#endif
