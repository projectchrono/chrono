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
// 2WD driveline model template based on ChShaft objects. This template can be
// used to model either a FWD or a RWD driveline.
//
// =============================================================================

#include "physics/ChSystem.h"

#include "subsys/driveline/ChShaftsDriveline2WD.h"

namespace chrono {


// -----------------------------------------------------------------------------
// dir_motor_block specifies the direction of the driveshaft, i.e. the input of
// the conic gear pair, in chassis local coords.
//
// dir_axle specifies the direction of the axle, i.e. the output of the conic
// conic gear pair, in chassis local coords. This is needed because ChShaftsBody
// could transfer pitch torque to the chassis.
// -----------------------------------------------------------------------------
ChShaftsDriveline2WD::ChShaftsDriveline2WD()
: ChDriveline(ChDriveline::TWO_WHEEL_DRIVE),
  m_dir_motor_block(ChVector<>(1, 0, 0)),
  m_dir_axle(ChVector<>(0, 1, 0))
{
}

// -----------------------------------------------------------------------------
// Initialize the driveline subsystem.
// This function connects this driveline subsystem to the axles of the provided
// suspension subsystems.  Note that it is the responsibility of the caller to
// provide a number of suspension subsystems consistent with the driveline type
// (in this case a single suspension).
// -----------------------------------------------------------------------------
void ChShaftsDriveline2WD::Initialize(ChSharedPtr<ChBody>     chassis,
                                      const ChSuspensionList& suspensions)
{
  assert(chassis);
  assert(suspensions.size() > 0);
  assert(suspensions[0]->GetAxle(LEFT));
  assert(suspensions[0]->GetAxle(RIGHT));
  assert(chassis->GetSystem());

  ChSystem* my_system = chassis->GetSystem();


  // CREATE  the driveshaft, a 1 d.o.f. object with rotational inertia which 
  // represents the connection of the driveline to the transmission box.
  m_driveshaft = ChSharedPtr<ChShaft>(new ChShaft);
  m_driveshaft->SetInertia(GetDriveshaftInertia());
  my_system->Add(m_driveshaft);

  // CREATE  a 1 d.o.f. object: a 'shaft' with rotational inertia.
  // This represents the inertia of the rotating box of the differential.
  m_differentialbox = ChSharedPtr<ChShaft>(new ChShaft);
  m_differentialbox->SetInertia(GetDifferentialBoxInertia());
  my_system->Add(m_differentialbox);

  // CREATE an angled gearbox, i.e a transmission ratio constraint between two
  // non parallel shafts. This is the case of the 90° bevel gears in the
  // differential. Note that, differently from the basic ChShaftsGear, this also
  // provides the possibility of transmitting a reaction torque to the box
  // (the truss).
  m_conicalgear = ChSharedPtr<ChShaftsGearboxAngled>(new ChShaftsGearboxAngled);
  m_conicalgear->Initialize(m_driveshaft,
    m_differentialbox,
    chassis,
    m_dir_motor_block,
    m_dir_axle);
  m_conicalgear->SetTransmissionRatio(GetConicalGearRatio());
  my_system->Add(m_conicalgear);

  // CREATE a differential, i.e. an apicycloidal mechanism that connects three 
  // rotating members. This class of mechanisms can be simulated using 
  // ChShaftsPlanetary; a proper 'ordinary' transmission ratio t0 must be
  // assigned according to Willis formula. The case of the differential is
  // simple: t0=-1.
  m_differential = ChSharedPtr<ChShaftsPlanetary>(new ChShaftsPlanetary);
  m_differential->Initialize(m_differentialbox,
                             suspensions[0]->GetAxle(LEFT),
                             suspensions[0]->GetAxle(RIGHT));
  m_differential->SetTransmissionRatioOrdinary(GetDifferentialRatio());
  my_system->Add(m_differential);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChShaftsDriveline2WD::GetWheelTorque(const ChWheelID& wheel_id) const
{
  switch (wheel_id.side()) {
  case LEFT:  return -m_differential->GetTorqueReactionOn2();
  case RIGHT: return -m_differential->GetTorqueReactionOn3();
  default:    return -1;
  }
}


} // end namespace chrono
