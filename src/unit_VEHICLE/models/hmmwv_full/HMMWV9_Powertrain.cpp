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
// HMMWV9 powertrain model based on ChShaft objects.
//
// =============================================================================

#include "HMMWV9_Powertrain.h"

using namespace chrono;

namespace hmmwv9 {


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
  const double HMMWV9_Powertrain::m_motorblock_inertia = 10.5;
  const double HMMWV9_Powertrain::m_crankshaft_inertia = 1.1;
  const double HMMWV9_Powertrain::m_ingear_shaft_inertia = 0.3;
  const double HMMWV9_Powertrain::m_outgear_shaft_inertia = 0.5;
  const double HMMWV9_Powertrain::m_differentialbox_inertia = 0.6;

  const double HMMWV9_Powertrain::m_conicalgear_ratio = -0.2;
  const double HMMWV9_Powertrain::m_differential_ratio = -1;


// -----------------------------------------------------------------------------
// Constructor of the HMMW9_Powertrain.
// the direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
HMMWV9_Powertrain::HMMWV9_Powertrain(HMMWV9_Vehicle* car)
: ChShaftsPowertrain(car, ChVector<>(1, 0, 0), ChVector<>(0, 1, 0))
{
}


// -----------------------------------------------------------------------------
// Initialize vector of gear ratios
// -----------------------------------------------------------------------------
void HMMWV9_Powertrain::SetGearRatios(std::vector<double>& gear_ratios)
{
  gear_ratios.push_back(-0.1); // 0: reverse gear;
  gear_ratios.push_back( 0.2); // 1: 1st gear;
  gear_ratios.push_back( 0.3); // 2: 2nd gear;
  gear_ratios.push_back( 0.5); // 3: 3rd gear;
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
void HMMWV9_Powertrain::SetEngineTorqueMap(ChSharedPtr<ChFunction_Recorder>& map)
{
  map->AddPoint(-5, 300);
  map->AddPoint(0, 360);
  map->AddPoint(200, 440);
  map->AddPoint(400, 480);
  map->AddPoint(500, 440);
  map->AddPoint(600, 240);
  map->AddPoint(700, -200);
}

void HMMWV9_Powertrain::SetTorqueConverterCapacityFactorMap(ChSharedPtr<ChFunction_Recorder>& map)
{
  map->AddPoint(0.0, 15);
  map->AddPoint(0.25, 15);
  map->AddPoint(0.50, 15);
  map->AddPoint(0.75, 16);
  map->AddPoint(0.90, 18);
  map->AddPoint(1.00, 35);
}

void HMMWV9_Powertrain::SetTorqeConverterTorqueRatioMap(ChSharedPtr<ChFunction_Recorder>& map)
{
  map->AddPoint(0.0, 2.00);
  map->AddPoint(0.25, 1.80);
  map->AddPoint(0.50, 1.50);
  map->AddPoint(0.75, 1.15);
  map->AddPoint(1.00, 1.00);
}


} // end namespace hmmwv9
