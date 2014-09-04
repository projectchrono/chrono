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
// Authors: Radu Serban, Aki Mikkola
// =============================================================================
//
// Template for LuGre tire model
//
// =============================================================================


#include "ChLugreTire.h"


namespace chrono {


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChLugreTire::ChLugreTire(const ChTerrain& terrain)
: ChTire(terrain)
{
  m_tireForce.force = ChVector<>(0, 0, 0);
  m_tireForce.point = ChVector<>(0, 0, 0);
  m_tireForce.moment = ChVector<>(0, 0, 0);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLugreTire::Initialize()
{
  m_data.resize(getNumDiscs());
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLugreTire::Update(double              time,
                         const ChBodyState&  wheel_state)
{
  double disc_radius = getRadius();
  const double* disc_locs = getDiscLocations();

  // Clear the force accumulators and set the application point to the wheel
  // center.
  m_tireForce.force = ChVector<>(0, 0, 0);
  m_tireForce.moment = ChVector<>(0, 0, 0);
  m_tireForce.point = wheel_state.pos;

  // Extract the wheel normal (expressed in global frame)
  ChMatrix33<> A(wheel_state.rot);
  ChVector<> disc_normal = A.Get_A_Yaxis();

  // Loop over all discs, check contact with terrain and accumulate normal tire
  // forces.
  ChVector<> normal;
  ChVector<> ptD;
  ChVector<> ptT;
  double depth;

  for (int id = 0; id < getNumDiscs(); id++) {
    // Calculate center of disk (expressed in global frame)
    ChVector<> disc_center = wheel_state.pos + disc_locs[id] * disc_normal;

    // Check contact with terrain and calculate contact points.
    m_data[id].contact = disc_terrain_contact(disc_center, disc_normal, disc_radius,
                                              ptD, ptT, normal, depth);
    if (!m_data[id].contact)
      continue;

    // Relative velocity at contact point (expressed in global frame)
    ChVector<> vel = wheel_state.lin_vel + Vcross(wheel_state.ang_vel, ptD - wheel_state.pos);

    // Generate normal contact force and add to accumulators (recall, all forces
    // are reduced to the wheel center)
    double     normalVel_mag = Vdot(vel, normal);
    double     Fn_mag = getNormalStiffness() * depth - getNormalDamping() * normalVel_mag;
    ChVector<> Fn = Fn_mag * normal;

    m_tireForce.force += Fn;
    m_tireForce.moment += Vcross(ptD - m_tireForce.point, Fn);

    // Calculate and store relative sliding velocity
    m_data[id].slidingVel = vel - normalVel_mag * normal;

  }

}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLugreTire::Advance(double step)
{
  for (int id = 0; id < getNumDiscs(); id++) {
    if (!m_data[id].contact)
      continue;


  }
}



} // end namespace chrono
