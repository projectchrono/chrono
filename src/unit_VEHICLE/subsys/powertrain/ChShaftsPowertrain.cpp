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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Powertrain model template based on ChShaft objects.
//
// =============================================================================

#include "physics/ChSystem.h"

#include "subsys/powertrain/ChShaftsPowertrain.h"

namespace chrono {


// -----------------------------------------------------------------------------
// dir_motor_block specifies the direction of the motor block, i.e. the
// direction of the crankshaft, in chassis local coords. This is needed because
// ChShaftsBody could transfer rolling torque to the chassis.
// -----------------------------------------------------------------------------
ChShaftsPowertrain::ChShaftsPowertrain(ChVehicle*         car,
                                       const ChVector<>&  dir_motor_block)
: ChPowertrain(car),
  m_dir_motor_block(dir_motor_block)
{
  drive_mode = ChPowertrain::FORWARD;
  last_time_gearshift = 0;
  gear_shift_latency = 0.5;
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChShaftsPowertrain::Initialize(ChSharedPtr<ChBody>  chassis,
                                    ChSharedPtr<ChShaft> driveshaft)
{
  assert(chassis);
  assert(driveshaft);
  assert(chassis->GetSystem());

  ChSystem* my_system = chassis->GetSystem();


  // Let the derived class specify the gear ratios
  SetGearRatios(m_gear_ratios);
  assert(m_gear_ratios.size() > 1);
  m_current_gear = 1;

  drive_mode = FORWARD;

  // CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
  // In this case it is the motor block. This because the ChShaftsThermalEngine
  // needs two 1dof items to apply the torque in-between them (the other will be
  // the crankshaft). In simplier models, one could just use SetShaftFixed() on
  // this object, but here we prefer to leave it free and use a ChShaftsBody
  // constraint to connect it to the car chassis (so the car can 'roll' when 
  // pressing the throttle, like in muscle cars)
  m_motorblock = ChSharedPtr<ChShaft>(new ChShaft);
  m_motorblock->SetInertia(GetMotorBlockInertia());
  my_system->Add(m_motorblock);


  // CREATE  a connection between the motor block and the 3D rigid body that
  // represents the chassis. This allows to get the effect of the car 'rolling'
  // when the longitudinal engine accelerates suddenly.
  m_motorblock_to_body = ChSharedPtr<ChShaftsBody>(new ChShaftsBody);
  m_motorblock_to_body->Initialize(m_motorblock,
                                   chassis,
                                   m_dir_motor_block);
  my_system->Add(m_motorblock_to_body);

  // CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
  // This represents the crankshaft plus flywheel.
  m_crankshaft = ChSharedPtr<ChShaft>(new ChShaft);
  m_crankshaft->SetInertia(GetCrankshaftInertia());
  my_system->Add(m_crankshaft);
  

  // CREATE  a thermal engine model beteen motor block and crankshaft (both
  // receive the torque, but with opposite sign).
  m_engine = ChSharedPtr<ChShaftsThermalEngine>(new ChShaftsThermalEngine);
  m_engine->Initialize(m_crankshaft,
                       m_motorblock);
  my_system->Add(m_engine);
    // The thermal engine requires a torque curve: 
  ChSharedPtr<ChFunction_Recorder> mTw(new ChFunction_Recorder);
  SetEngineTorqueMap(mTw);
  m_engine->SetTorqueCurve(mTw);

  // CREATE  an engine brake model that represents the losses of the engine because
  // of inner frictions/turbolences/etc. Without this, the engine at 0% throttle
  // in neutral position would rotate forever at contant speed.
  m_engine_losses = ChSharedPtr<ChShaftsThermalEngine>(new ChShaftsThermalEngine);
  m_engine_losses->Initialize(m_crankshaft,
                       m_motorblock);
  my_system->Add(m_engine_losses);
    // The engine brake model requires a torque curve: 
  ChSharedPtr<ChFunction_Recorder> mTw_losses(new ChFunction_Recorder);
  SetEngineLossesMap(mTw_losses);
  m_engine_losses->SetTorqueCurve(mTw_losses);


  // CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
  // This represents the shaft that collects all inertias from torque converter to the gear.
  m_shaft_ingear = ChSharedPtr<ChShaft>(new ChShaft);
  m_shaft_ingear->SetInertia(GetIngearShaftInertia());
  my_system->Add(m_shaft_ingear);


  // CREATE a torque converter and connect the shafts: 
  // A (input),B (output), C(truss stator). 
  // The input is the m_crankshaft, output is m_shaft_ingear; for stator, reuse the motor block 1D item.
  m_torqueconverter = ChSharedPtr<ChShaftsTorqueConverter>(new ChShaftsTorqueConverter);
  m_torqueconverter->Initialize(m_crankshaft,
                                m_shaft_ingear,
                                m_motorblock);
  my_system->Add(m_torqueconverter);
   // To complete the setup of the torque converter, a capacity factor curve is needed:
  ChSharedPtr<ChFunction_Recorder> mK(new ChFunction_Recorder);
  SetTorqueConverterCapacityFactorMap(mK);
  m_torqueconverter->SetCurveCapacityFactor(mK);
   // To complete the setup of the torque converter, a torque ratio curve is needed:	
  ChSharedPtr<ChFunction_Recorder> mT(new ChFunction_Recorder);
  SetTorqeConverterTorqueRatioMap(mT);
  m_torqueconverter->SetCurveTorqueRatio(mT);


  // CREATE a gearbox, i.e a transmission ratio constraint between two
  // shafts. Note that differently from the basic ChShaftsGear, this also provides
  // the possibility of transmitting a reaction torque to the box (the truss).
  m_gears = ChSharedPtr<ChShaftsGearbox>(new ChShaftsGearbox);
  m_gears->Initialize(m_shaft_ingear,
                      driveshaft,
                      chassis,
                      m_dir_motor_block);
  m_gears->SetTransmissionRatio(m_gear_ratios[m_current_gear]);
  my_system->Add(m_gears);


  // -------
  // Finally, update the gear ratio according to the selected gear in the 
  // array of gear ratios:
  SetSelectedGear(1);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChShaftsPowertrain::SetSelectedGear(int igear)
{
  assert(igear >= 0);
  assert(igear < m_gear_ratios.size());

  m_current_gear = igear;
  if (m_gears)
  {
    m_gears->SetTransmissionRatio(m_gear_ratios[igear]);
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChShaftsPowertrain::SetDriveMode(ChPowertrain::DriveMode mmode)
{
  if (this->drive_mode == mmode) return;

  // disallow switching if motor is spinning too fast
  //if (this->GetMotorSpeed() > 1500*CH_C_2PI/60.0)
  //  return;

  this->drive_mode = mmode;

  if(this->drive_mode == FORWARD)
  {
    if (m_gears)
    {
      this->SetSelectedGear(1);
    }
  }

  if(this->drive_mode == NEUTRAL)
  {
    if (m_gears)
    {
      m_gears->SetTransmissionRatio(1e20);
    }
  }

  if(this->drive_mode == REVERSE)
  {
    if (m_gears)
    {
      this->SetSelectedGear(0);
    }
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChShaftsPowertrain::Update(double time,
                                double throttle)
{
  // Just update the throttle level in the thermal engine
  m_engine->SetThrottle(throttle);


  // Shift the gear if needed, automatically shifting up or down with 
  // a very simple logic, for instance as in the following fixed latency 
  // state machine:

  double now_time = this->m_car->GetChassis()->GetChTime();

  if(now_time-last_time_gearshift > gear_shift_latency) // avoids bursts of gear shifts
  {
	  if(this->drive_mode == FORWARD)
	  {
		  //if (this->GetMotorSpeed() > 2500*CH_C_2PI/60.0)
		  if (this->m_shaft_ingear->GetPos_dt() > 2500*CH_C_2PI/60.0)
		  {
			  if (this->GetSelectedGear()+1 < this->m_gear_ratios.size())
			  {
				  GetLog() << "SHIFT UP " << this->GetSelectedGear() << "\n";
				  this->SetSelectedGear(this->GetSelectedGear()+1);
				  last_time_gearshift = now_time;
			  }
		  }
		  //if (this->GetMotorSpeed() < 1500*CH_C_2PI/60.0)
		  if (this->m_shaft_ingear->GetPos_dt() < 1500*CH_C_2PI/60.0)
		  {
			  if (this->GetSelectedGear()-1 > 0)
			  {
				  GetLog() << "SHIFT DOWN " << this->GetSelectedGear() << "\n";
				  this->SetSelectedGear(this->GetSelectedGear()-1);
				  last_time_gearshift = now_time;
			  }
		  }
	  }
  }

}


} // end namespace chrono
