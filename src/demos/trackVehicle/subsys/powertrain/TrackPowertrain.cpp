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
// Authors: Justin Madsen, Alessandro Tasora, Radu Serban
// =============================================================================
//
// Powertrain model template based on ChShaft objects, for a tracked vehicle
//
// =============================================================================

#include "physics/ChSystem.h"

#include "subsys/powertrain/TrackPowertrain.h"

using namespace chrono;

const double TrackPowertrain::m_motorblock_inertia = 10.5;
const double TrackPowertrain::m_crankshaft_inertia = 1.1;
const double TrackPowertrain::m_ingear_shaft_inertia = 0.3;


// -----------------------------------------------------------------------------
// dir_motor_block specifies the direction of the motor block, i.e. the
// direction of the crankshaft, in chassis local coords. This is needed because
// ChShaftsBody could transfer rolling torque to the chassis.
// -----------------------------------------------------------------------------
TrackPowertrain::TrackPowertrain(const std::string& name,
                                 const ChVector<>&  dir_motor_block)
: m_drive_mode(NEUTRAL),
  m_dir_motor_block(dir_motor_block),
  m_last_time_gearshift(0),
  m_gear_shift_latency(0.5)
{
  // hard-code gear ratios
  SetGearRatios(m_gear_ratios);
  assert(m_gear_ratios.size() > 1);
  m_current_gear = 1;

  //  1 d.o.f. object: a 'shaft' with rotational inertia.
  // In this case it is the motor block. This because the ChShaftsThermalEngine
  // needs two 1dof items to apply the torque in-between them (the other will be
  // the crankshaft). In simplier models, one could just use SetShaftFixed() on
  // this object, but here we prefer to leave it free and use a ChShaftsBody
  // constraint to connect it to the car chassis (so the car can 'roll' when 
  // pressing the throttle, like in muscle cars)
  m_motorblock = ChSharedPtr<ChShaft>(new ChShaft);
  m_motorblock->SetInertia(GetMotorBlockInertia());

  // connection between the motor block and the 3D rigid body that
  // represents the chassis. This allows to get the effect of the car 'rolling'
  // when the longitudinal engine accelerates suddenly.
  m_motorblock_to_body = ChSharedPtr<ChShaftsBody>(new ChShaftsBody);

  // crankshaft plus flywheel.
  m_crankshaft = ChSharedPtr<ChShaft>(new ChShaft);
  m_crankshaft->SetInertia(GetCrankshaftInertia());

  // thermal engine model beteen motor block and crankshaft (both
  // receive the torque, but with opposite sign).
  m_engine = ChSharedPtr<ChShaftsThermalEngine>(new ChShaftsThermalEngine);

  // The thermal engine requires a torque curve: 
  ChSharedPtr<ChFunction_Recorder> mTw(new ChFunction_Recorder);
  SetEngineTorqueMap(mTw);
  m_engine->SetTorqueCurve(mTw);

  // engine brake model that represents the losses of the engine because
  // of inner frictions/turbolences/etc. Without this, the engine at 0% throttle
  // in neutral position would rotate forever at contant speed.
  m_engine_losses = ChSharedPtr<ChShaftsThermalEngine>(new ChShaftsThermalEngine);

  
  // The engine brake model requires a torque curve: 
  ChSharedPtr<ChFunction_Recorder> mTw_losses(new ChFunction_Recorder);
  SetEngineLossesMap(mTw_losses);
  m_engine_losses->SetTorqueCurve(mTw_losses);

  // This represents the shaft that collects all inertias from torque converter to the gear.
  m_shaft_ingear = ChSharedPtr<ChShaft>(new ChShaft);
  m_shaft_ingear->SetInertia(GetIngearShaftInertia());

  // torque converter and connect the shafts: 
  // A (input),B (output), C(truss stator). 
  // The input is the m_crankshaft, output is m_shaft_ingear; for stator, reuse the motor block 1D item.
  m_torqueconverter = ChSharedPtr<ChShaftsTorqueConverter>(new ChShaftsTorqueConverter);

   // To complete the setup of the torque converter, a capacity factor curve is needed:
  ChSharedPtr<ChFunction_Recorder> mK(new ChFunction_Recorder);
  SetTorqueConverterCapacityFactorMap(mK);
  m_torqueconverter->SetCurveCapacityFactor(mK);
   // To complete the setup of the torque converter, a torque ratio curve is needed:	
  ChSharedPtr<ChFunction_Recorder> mT(new ChFunction_Recorder);
  SetTorqeConverterTorqueRatioMap(mT);
  m_torqueconverter->SetCurveTorqueRatio(mT);

  
  // gearbox, i.e a transmission ratio constraint between two
  // shafts. Note that differently from the basic ChShaftsGear, this also provides
  // the possibility of transmitting a reaction torque
  m_gears = ChSharedPtr<ChShaftsGearbox>(new ChShaftsGearbox);
  m_gears->SetTransmissionRatio(m_gear_ratios[m_current_gear]);
}


void TrackPowertrain::Initialize(ChSharedPtr<ChBody>  chassis,
                                    ChSharedPtr<ChShaft> driveshaft)
{
  assert(chassis);
  assert(driveshaft);
  assert(chassis->GetSystem());

  ChSystem* my_system = chassis->GetSystem();

  // add motor block shaft to system
  my_system->Add(m_motorblock);

  // initialize connection, add to system
  m_motorblock_to_body->Initialize(m_motorblock,
                                   chassis,
                                   m_dir_motor_block);
  my_system->Add(m_motorblock_to_body);

 // add the crankshaft to the system
  my_system->Add(m_crankshaft);
  
  // init. engine between motor block and crankshaft. Add to system.
  m_engine->Initialize(m_crankshaft,
                       m_motorblock);
  my_system->Add(m_engine);

  // init. engine brake
  m_engine_losses->Initialize(m_crankshaft,
                              m_motorblock);
  my_system->Add(m_engine_losses);

  // add gear shaft to system
  my_system->Add(m_shaft_ingear);

  // init. torque converter, add to system.
  m_torqueconverter->Initialize(m_crankshaft,
                                m_shaft_ingear,
                                m_motorblock);
  my_system->Add(m_torqueconverter);

  // init. gearbox shaft, add it to the system
  m_gears->Initialize(m_shaft_ingear,
                      driveshaft,
                      chassis,
                      m_dir_motor_block);
  my_system->Add(m_gears);

  // update the gear ratio according to the selected gear in the 
  // array of gear ratios:
  SetSelectedGear(1);
}


void TrackPowertrain::SetSelectedGear(int igear)
{
  assert(igear >= 0);
  assert(igear < m_gear_ratios.size());

  m_current_gear = igear;
  if (m_gears)
    m_gears->SetTransmissionRatio(m_gear_ratios[igear]);
}


void TrackPowertrain::SetDriveMode(DriveMode mmode)
{
  if (m_drive_mode == mmode) return;

  m_drive_mode = mmode;

  if (!m_gears) return;

  switch (m_drive_mode) {
  case FORWARD: SetSelectedGear(1); break;
  case NEUTRAL: m_gears->SetTransmissionRatio(1e20); break;
  case REVERSE: SetSelectedGear(0); break;
  }
}


void TrackPowertrain::Update(double time,
                             double throttle_Right,
                             double throttle_Left,
                             double shaft_speed)
{
  // Just update the throttle level in the thermal engine
  m_engine->SetThrottle( (abs(throttle_Right)+abs(throttle_Left) ) / 2.0);

  // To avoid bursts of gear shifts, do nothing if the last shift was too recent
  if (time - m_last_time_gearshift < m_gear_shift_latency)
    return;

  // Shift the gear if needed, automatically shifting up or down with 
  // a very simple logic, for instance as in the following fixed latency 
  // state machine:
  if (m_drive_mode != FORWARD)
    return;

  double gearshaft_speed = m_shaft_ingear->GetPos_dt();

  if (gearshaft_speed > 2500 * CH_C_2PI / 60.0) {
    // upshift if possible
    if (m_current_gear + 1 < m_gear_ratios.size()) {
      GetLog() << "SHIFT UP " << m_current_gear << " -> " << m_current_gear + 1 << "\n";
      SetSelectedGear(m_current_gear + 1);
      m_last_time_gearshift = time;
    }
  }
  else if (gearshaft_speed < 1500 * CH_C_2PI / 60.0) {
    // downshift if possible
    if (m_current_gear - 1 > 0) {
      GetLog() << "SHIFT DOWN " << m_current_gear << " -> " << m_current_gear - 1 << "\n";
      SetSelectedGear(m_current_gear - 1);
      m_last_time_gearshift = time;
    }
  }

}

// Initialize vector of gear ratios
void TrackPowertrain::SetGearRatios(std::vector<double>& gear_ratios)
{
  gear_ratios.push_back(-0.1); // 0: reverse gear;
  gear_ratios.push_back( 0.2); // 1: 1st gear;
  gear_ratios.push_back( 0.4); // 2: 2nd gear;
  gear_ratios.push_back( 0.8); // 3: 3rd gear;
}


// -----------------------------------------------------------------------------
// Set the engine and torque converter maps:
//
// (1) engine speed [rad/s] - torque [Nm] map
//     must be defined beyond max speed too - engine might be 'pulled'
//
// (2) TC capacity factor map
//
// (3) TC torque ratio map
//
// -----------------------------------------------------------------------------
void TrackPowertrain::SetEngineTorqueMap(ChSharedPtr<ChFunction_Recorder>& map)
{
  double rpm_to_radsec = CH_C_2PI / 60.;

  map->AddPoint(-100*rpm_to_radsec ,    300); // to start engine
  map->AddPoint(800*rpm_to_radsec ,     382);
  map->AddPoint(900*rpm_to_radsec ,     490);
  map->AddPoint(1000*rpm_to_radsec ,    579);
  map->AddPoint(1100*rpm_to_radsec ,    650);
  map->AddPoint(1200*rpm_to_radsec ,    706);
  map->AddPoint(1300*rpm_to_radsec ,    746);
  map->AddPoint(1400*rpm_to_radsec ,    774);
  map->AddPoint(1500*rpm_to_radsec ,    789);
  map->AddPoint(1600*rpm_to_radsec ,    793);
  map->AddPoint(1700*rpm_to_radsec ,    788);
  map->AddPoint(1800*rpm_to_radsec ,    774);
  map->AddPoint(1900*rpm_to_radsec ,    754);
  map->AddPoint(2000*rpm_to_radsec ,    728);
  map->AddPoint(2100*rpm_to_radsec ,    697);
  map->AddPoint(2200*rpm_to_radsec ,    664);
  map->AddPoint(2300*rpm_to_radsec ,    628);
  map->AddPoint(2400*rpm_to_radsec ,    593);
  map->AddPoint(2500*rpm_to_radsec ,    558);
  map->AddPoint(2700*rpm_to_radsec ,   -400); // fading out of engine torque
}

void TrackPowertrain::SetEngineLossesMap(ChSharedPtr<ChFunction_Recorder>& map)
{
  double rpm_to_radsec = CH_C_2PI / 60.;

  map->AddPoint( -50*rpm_to_radsec ,     30); // it should never work in negative direction, anyway..
  map->AddPoint(   0*rpm_to_radsec ,      0);
  map->AddPoint(  50*rpm_to_radsec ,    -30);
  map->AddPoint(1000*rpm_to_radsec ,    -50);
  map->AddPoint(2000*rpm_to_radsec ,    -70);
  map->AddPoint(3000*rpm_to_radsec ,    -90);
}

void TrackPowertrain::SetTorqueConverterCapacityFactorMap(ChSharedPtr<ChFunction_Recorder>& map)
{
  map->AddPoint(0.0, 15);
  map->AddPoint(0.25, 15);
  map->AddPoint(0.50, 15);
  map->AddPoint(0.75, 16);
  map->AddPoint(0.90, 18);
  map->AddPoint(1.00, 35);
/*
    map->AddPoint(0     ,   81.0000);
	map->AddPoint(0.1000,   81.1589);
	map->AddPoint(0.2000,   81.3667);
	map->AddPoint(0.3000,   81.6476);
	map->AddPoint(0.4000,   82.0445);
	map->AddPoint(0.5000,   82.6390);
	map->AddPoint(0.6000,   83.6067);
	map->AddPoint(0.7000,   85.3955);
	map->AddPoint(0.8000,   89.5183);
	map->AddPoint(0.9000,  105.1189);
	map->AddPoint(0.9700,  215.5284);
	map->AddPoint(1.0000,  235.5284);
*/
}

void TrackPowertrain::SetTorqeConverterTorqueRatioMap(ChSharedPtr<ChFunction_Recorder>& map)
{
  map->AddPoint(0.0, 2.00);
  map->AddPoint(0.25, 1.80);
  map->AddPoint(0.50, 1.50);
  map->AddPoint(0.75, 1.15);
  map->AddPoint(1.00, 1.00);
/*
	map->AddPoint(0,        1.7500);  
	map->AddPoint(0.1000,    1.6667);  
	map->AddPoint(0.2000,    1.5833);   
	map->AddPoint(0.3000,    1.5000);   
	map->AddPoint(0.4000,    1.4167);   
	map->AddPoint(0.5000,    1.3334);   
	map->AddPoint(0.6000,    1.2500);   
	map->AddPoint(0.7000,    1.1667);   
	map->AddPoint(0.8000,    1.0834);   
	map->AddPoint(0.9000,    1.0000);  
	map->AddPoint(0.9700,    1.0000);  
	map->AddPoint(1.0000,    1.0000); 
*/
}


