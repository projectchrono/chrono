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
// Driveline model template based on ChShaft objects. Models the left and right
// driveline to the drive sprockets of a tracked vehicle
//
// =============================================================================

#include "physics/ChSystem.h"

#include "subsys/driveline/TrackDriveline.h"

using namespace chrono;

// -----------------------------------------------------------------------------
// Static variables
const double TrackDriveline::m_driveshaft_inertia = 0.5;
const double TrackDriveline::m_differentialbox_inertia = 0.6;

const double TrackDriveline::m_conicalgear_ratio = -0.2;
const double TrackDriveline::m_differential_ratio = -1;


// -----------------------------------------------------------------------------
// dir_motor_block specifies the direction of the driveshaft, i.e. the input of
// the conic gear pair, in chassis local coords.
//
// dir_axle specifies the direction of the axle, i.e. the output of the conic
// conic gear pair, in chassis local coords. This is needed because ChShaftsBody
// could transfer pitch torque to the chassis.
// -----------------------------------------------------------------------------
TrackDriveline::TrackDriveline(const std::string& name)
: m_dir_motor_block(ChVector<>(1, 0, 0)),
  m_dir_axle(ChVector<>(0, 1, 0))
{
  // Create the driveshaft, a 1 d.o.f. object with rotational inertia which 
  // represents the connection of the driveline to the transmission box.
  m_driveshaft = ChSharedPtr<ChShaft>(new ChShaft);
  m_driveshaft->SetInertia(GetDriveshaftInertia());

  // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
  // This represents the inertia of the rotating box of the differential.
  m_differentialbox = ChSharedPtr<ChShaft>(new ChShaft);
  m_differentialbox->SetInertia(GetDifferentialBoxInertia());

  // Create an angled gearbox, i.e a transmission ratio constraint between two
  // non parallel shafts. This is the case of the 90° bevel gears in the
  // differential. Note that, differently from the basic ChShaftsGear, this also
  // provides the possibility of transmitting a reaction torque to the box
  // (the truss).
  m_conicalgear = ChSharedPtr<ChShaftsGearboxAngled>(new ChShaftsGearboxAngled);
  m_conicalgear->SetTransmissionRatio(GetConicalGearRatio());

  // TODO:  does this apply to a tracked vehicle powertrain/driveline system?

  // Create a differential, i.e. an apicycloidal mechanism that connects three 
  // rotating members. This class of mechanisms can be simulated using 
  // ChShaftsPlanetary; a proper 'ordinary' transmission ratio t0 must be
  // assigned according to Willis formula. The case of the differential is
  // simple: t0=-1.
  m_differential = ChSharedPtr<ChShaftsPlanetary>(new ChShaftsPlanetary);
  m_differential->SetTransmissionRatioOrdinary(GetDifferentialRatio());

}

// -----------------------------------------------------------------------------
// Initialize the driveline subsystem.
// This function connects this driveline subsystem to the DriveGear subsystems.
// Add all the shaft components to the system after finished w/ init.
void TrackDriveline::Initialize(ChSharedPtr<ChBody>     chassis,
                                ChSharedPtr<DriveGear>  drivegear_L,
                                ChSharedPtr<DriveGear>  drivegear_R)
{
  // m_driven_axles = driven_axles;
  ChSystem* my_system = chassis->GetSystem();

  // add driveshaft to system
  my_system->Add(m_driveshaft);

  // add differential to system
  my_system->Add(m_differentialbox);

  // initialize conical gear, and add to system
  m_conicalgear->Initialize(m_driveshaft,
                            m_differentialbox,
                            chassis,
                            m_dir_motor_block,
                            m_dir_axle);

  my_system->Add(m_conicalgear);

  // inititalize differential, add to system
  m_differential->Initialize(m_differentialbox,
    drivegear_L->GetAxle(),
    drivegear_R->GetAxle() );

  my_system->Add(m_differential);
}


double TrackDriveline::GetGearTorque(const int gear_id) const
{
  // TODO:  This may not be correct
  if (gear_id) {
    return -m_differential->GetTorqueReactionOn2();
  } else
  {
    return -m_differential->GetTorqueReactionOn3();
  }
}


