// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
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
// Provides power to a single output shaft/gear.
//
// =============================================================================

#include "physics/ChSystem.h"

#include "TrackDriveline_1WD.h"

using namespace chrono;

// -----------------------------------------------------------------------------
// Static variables
const double TrackDriveline_1WD::m_driveshaft_inertia = 0.5;
const double TrackDriveline_1WD::m_outshaft_inertia = 0.6;

const double TrackDriveline_1WD::m_conicalgear_ratio = -0.2;

// -----------------------------------------------------------------------------
// dir_motor_block specifies the direction of the driveshaft, i.e. the input of
// the conic gear pair, in chassis local coords.
//
// dir_axle specifies the direction of the axle, i.e. the output of the conic
// conic gear pair, in chassis local coords. This is needed because ChShaftsBody
// could transfer pitch torque to the chassis.
// -----------------------------------------------------------------------------
TrackDriveline_1WD::TrackDriveline_1WD(const std::string& name)
: m_dir_motor_block(ChVector<>(1, 0, 0)),
  m_dir_axle(ChVector<>(0, 1, 0))
{
  // Create the driveshaft, a 1 d.o.f. object with rotational inertia which 
  // represents the connection of the driveline to the transmission box.
  m_driveshaft = ChSharedPtr<ChShaft>(new ChShaft);
  m_driveshaft->SetInertia(GetDriveshaftInertia());

  // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
  // This represents the inertia of the rotating box of the differential.
  m_outshaft = ChSharedPtr<ChShaft>(new ChShaft);
  m_outshaft->SetInertia( GetOutshaftInertia() );

  // Create an angled gearbox, i.e a transmission ratio constraint between two
  // non parallel shafts. This is the case of the 90° bevel gears in the
  // differential. Note that, differently from the basic ChShaftsGear, this also
  // provides the possibility of transmitting a reaction torque to the box
  // (the truss).
  m_conicalgear = ChSharedPtr<ChShaftsGearboxAngled>(new ChShaftsGearboxAngled);
  m_conicalgear->SetTransmissionRatio(GetConicalGearRatio());

  // no differential for a 1WD system
}

// -----------------------------------------------------------------------------
// Initialize the driveline subsystem.
// This function connects this driveline subsystem to the DriveGear subsystems.
// Add all the shaft components to the system after finished w/ init.
void TrackDriveline_1WD::Initialize(ChSharedPtr<ChBody>     chassis,
                                ChSharedPtr<DriveGear>  drivegear)
{
  // m_driven_axles = driven_axles;
  ChSystem* my_system = chassis->GetSystem();

  // add driveshaft to system
  my_system->Add(m_driveshaft);

  // add differential to system
  my_system->Add(m_outshaft);

  // initialize conical gear, and add to system
  m_conicalgear->Initialize(m_driveshaft,
                            m_outshaft,
                            chassis,
                            m_dir_motor_block,
                            m_dir_axle);

  my_system->Add(m_conicalgear);

  // conical gear attaches to left axle, rather than a diff.
}


double TrackDriveline_1WD::GetGearTorque(const int onBody) const
{
  if (onBody > 0) {
    return -m_conicalgear->GetTorqueReactionOn2();
  } else
  {
    return  -m_conicalgear->GetTorqueReactionOn1();
  }
}


